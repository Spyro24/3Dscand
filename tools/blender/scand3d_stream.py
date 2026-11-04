"""Streams the live 3Dscand sparse map into Blender 4.x.

The phone writes ``cloud.ply`` into its capture session directory every time the live
reconstruction finished an update (``app/.../reconstruct/CloudPlyWriter.kt``). This add-on polls
that file over adb, in a background thread, and keeps one mesh object updated on Blender's main
thread, so the cloud grows while the scan is running.

The file is a binary little-endian PLY in the reconstruction's own frame: the first keyframe's
camera frame (x right, y down, z forward) with free scale (DECISIONS.md D028). The add-on converts
the axes to Blender's Z-up convention and, by default, normalises the cloud to a comparable size.
"""

bl_info = {
    "name": "3Dscand Point Cloud Stream",
    "author": "3Dscand",
    "version": (0, 1, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > 3Dscand",
    "description": "Streams the live sparse point cloud from the 3Dscand phone app over adb",
    "category": "Import-Export",
}

import math
import os
import struct
import subprocess
import threading

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    StringProperty,
)
from bpy.types import Operator, Panel, PropertyGroup

PACKAGE_NAME = "com.scand3d"
SESSION_ROOT = "files/sessions"
CLOUD_FILE_NAME = "cloud.ply"
OBJECT_NAME = "3Dscand Cloud"
GROUP_NAME = "3Dscand Cloud"
MATERIAL_NAME = "3Dscand Cloud"
CAMERA_NAME = "3Dscand View"

# A map that suddenly claims millions of points is not a cloud we can show; refuse it instead of
# allocating gigabytes on the main thread.
MAX_POINTS = 5_000_000

_PLY_CODES = {
    "char": "b",
    "int8": "b",
    "uchar": "B",
    "uint8": "B",
    "short": "h",
    "int16": "h",
    "ushort": "H",
    "uint16": "H",
    "int": "i",
    "int32": "i",
    "uint": "I",
    "uint32": "I",
    "float": "f",
    "float32": "f",
    "double": "d",
    "float64": "d",
}


class StreamError(RuntimeError):
    """A transport or session problem, shown in the panel instead of raised to the user."""


class PlyError(ValueError):
    """The bytes are not a point-cloud PLY this reader understands."""


# --------------------------------------------------------------------------------------------
# PLY
# --------------------------------------------------------------------------------------------


def parse_ply(data):
    """Returns the vertex positions of a PLY file as a list of (x, y, z) tuples."""
    if not data.startswith(b"ply"):
        raise PlyError("not a PLY file")

    header_end = data.find(b"end_header")
    if header_end < 0:
        raise PlyError("PLY header is truncated")
    body_start = data.find(b"\n", header_end)
    if body_start < 0:
        raise PlyError("PLY header has no line after end_header")
    body_start += 1

    layout = None
    count = 0
    properties = []
    element = None
    for line in data[:header_end].decode("ascii", "replace").splitlines()[1:]:
        fields = line.split()
        if not fields:
            continue
        keyword = fields[0]
        if keyword == "format" and len(fields) > 1:
            layout = fields[1]
        elif keyword == "element" and len(fields) > 2:
            element = fields[1]
            if element == "vertex":
                count = int(fields[2])
        elif keyword == "property" and element == "vertex" and len(fields) > 2:
            if fields[1] == "list":
                raise PlyError("vertex list properties are not supported")
            properties.append((fields[1], fields[2]))

    if layout is None:
        raise PlyError("PLY header has no format line")
    if not properties:
        raise PlyError("PLY header has no vertex properties")
    if count < 0 or count > MAX_POINTS:
        raise PlyError("PLY claims %i points, which is outside 0..%i" % (count, MAX_POINTS))

    names = [name for _, name in properties]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise PlyError("PLY has no '%s' vertex property" % axis)
    axis_indices = (names.index("x"), names.index("y"), names.index("z"))

    if layout == "ascii":
        rows = data[body_start:].decode("ascii", "replace").splitlines()
        points = []
        for row in rows:
            values = row.split()
            if not values:
                continue
            if len(values) < len(properties):
                raise PlyError("PLY body has a short row")
            points.append(tuple(float(values[index]) for index in axis_indices))
            if len(points) == count:
                break
        if len(points) != count:
            raise PlyError("PLY body holds %i points, the header says %i" % (len(points), count))
        return points

    if layout != "binary_little_endian":
        raise PlyError("unsupported PLY format '%s'" % layout)

    codes = []
    for type_name, _ in properties:
        if type_name not in _PLY_CODES:
            raise PlyError("unsupported PLY property type '%s'" % type_name)
        codes.append(_PLY_CODES[type_name])
    record = struct.Struct("<" + "".join(codes))
    if len(data) - body_start < record.size * count:
        raise PlyError("PLY body is shorter than the header says")

    points = []
    for index in range(count):
        values = record.unpack_from(data, body_start + index * record.size)
        points.append(tuple(values[axis] for axis in axis_indices))
    return points


# --------------------------------------------------------------------------------------------
# Transport
# --------------------------------------------------------------------------------------------


def _adb_argv(adb_path, serial, arguments):
    argv = [adb_path]
    if serial.strip():
        argv.extend(["-s", serial.strip()])
    argv.extend(arguments)
    return argv


def _run(argv, timeout_s):
    try:
        completed = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
        )
    except FileNotFoundError:
        raise StreamError("adb not found: %s (set the path in the panel)" % argv[0])
    except subprocess.TimeoutExpired:
        raise StreamError("adb did not answer within %i s" % int(timeout_s))
    except OSError as exc:
        raise StreamError("could not start adb: %s" % exc)
    if completed.returncode != 0:
        message = completed.stderr.decode("utf-8", "replace").strip()
        raise StreamError(message or "adb exited with code %i" % completed.returncode)
    return completed.stdout


def latest_session(adb_path, serial):
    """Name of the newest capture session directory inside the app's files directory."""
    listing = _run(
        _adb_argv(adb_path, serial, ["shell", "run-as", PACKAGE_NAME, "ls", SESSION_ROOT]),
        15.0,
    )
    names = [
        line.strip().rstrip("/").rsplit("/", 1)[-1]
        for line in listing.decode("utf-8", "replace").splitlines()
        if line.strip()
    ]
    if not names:
        raise StreamError("no capture session on the phone yet - run a scan first")
    return sorted(names)[-1]


def _read_device_cloud(adb_path, serial, session):
    device_path = "%s/%s/%s" % (SESSION_ROOT, session, CLOUD_FILE_NAME)
    # exec-out keeps the binary body intact; `shell cat` would rewrite line endings.
    return _run(
        _adb_argv(adb_path, serial, ["exec-out", "run-as", PACKAGE_NAME, "cat", device_path]),
        20.0,
    )


class CloudStream(threading.Thread):
    """Polls the cloud off the main thread and publishes the newest parse for the timer to apply."""

    def __init__(self, settings, interval):
        super().__init__(name="3dscand-cloud-stream", daemon=True)
        self._settings = settings
        self._interval = interval
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._points = None
        self._error = None
        self._pending = False
        self._label = ""
        self._resolved_session = ""

    def stop(self):
        self._stop_event.set()

    def take(self):
        """Returns (points, error, label, changed) and marks the pending result as consumed."""
        with self._lock:
            changed = self._pending
            self._pending = False
            return self._points, self._error, self._label, changed

    def run(self):
        while not self._stop_event.is_set():
            try:
                points, label = self._fetch()
            except (StreamError, PlyError) as exc:
                with self._lock:
                    self._error = str(exc)
                    self._pending = True
            else:
                with self._lock:
                    self._points = points
                    self._error = None
                    self._label = label
                    self._pending = True
            self._stop_event.wait(max(0.2, float(self._interval())))

    def _fetch(self):
        if self._settings["source"] == "FILE":
            path = self._settings["file_path"]
            if not path:
                raise StreamError("no cloud file selected")
            if not os.path.isfile(path):
                raise StreamError("cloud file not found: %s" % path)
            with open(path, "rb") as handle:
                return parse_ply(handle.read()), os.path.basename(path)

        adb_path = self._settings["adb_path"] or "adb"
        serial = self._settings["serial"]
        session = self._settings["session"].strip()
        if not session:
            if self._resolved_session:
                try:
                    data = _read_device_cloud(adb_path, serial, self._resolved_session)
                except StreamError:
                    # The scan was restarted and its directory is gone; pick the new newest one.
                    self._resolved_session = ""
                else:
                    return parse_ply(data), self._resolved_session
            self._resolved_session = latest_session(adb_path, serial)
            session = self._resolved_session
        return parse_ply(_read_device_cloud(adb_path, serial, session)), session


# --------------------------------------------------------------------------------------------
# Scene updates (main thread only)
# --------------------------------------------------------------------------------------------


def blender_points(points):
    """OpenCV camera frame (+x right, +y down, +z forward) to Blender's Z-up frame."""
    return [(x, z, -y) for (x, y, z) in points]


def normalised_points(points, target_size_m):
    """Scales the cloud so its median distance from the origin is `target_size_m` (D028)."""
    distances = sorted(math.sqrt(x * x + y * y + z * z) for (x, y, z) in points)
    median = distances[len(distances) // 2]
    if not math.isfinite(median) or median <= 0.0:
        return points
    scale = target_size_m / median
    return [(x * scale, y * scale, z * scale) for (x, y, z) in points]


def cloud_object(scene):
    obj = bpy.data.objects.get(OBJECT_NAME)
    if obj is not None and obj.type == "MESH":
        return obj
    mesh = bpy.data.meshes.new(OBJECT_NAME)
    obj = bpy.data.objects.new(OBJECT_NAME, mesh)
    scene.collection.objects.link(obj)
    return obj


def _set_vertices(mesh, coordinates):
    mesh.clear_geometry()
    mesh.vertices.add(len(coordinates))
    mesh.vertices.foreach_set("co", [value for point in coordinates for value in point])
    mesh.update()


def _ensure_material(colour):
    material = bpy.data.materials.get(MATERIAL_NAME)
    if material is None:
        material = bpy.data.materials.new(MATERIAL_NAME)
        material.use_nodes = True
    set_material_colour(material, colour)
    return material


def set_material_colour(material, colour):
    rgba = (colour[0], colour[1], colour[2], 1.0)
    material.diffuse_color = rgba
    tree = material.node_tree
    if tree is None:
        return
    principled = next((node for node in tree.nodes if node.type == "BSDF_PRINCIPLED"), None)
    if principled is not None:
        principled.inputs["Base Color"].default_value = rgba
        principled.inputs["Roughness"].default_value = 0.65


def build_node_group(material):
    """Points -> one small sphere per point, so the cloud shows in Eevee, Cycles and the viewport."""
    group = bpy.data.node_groups.new(GROUP_NAME, "GeometryNodeTree")
    group.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    group.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")

    nodes = group.nodes
    links = group.links

    group_in = nodes.new("NodeGroupInput")
    group_in.location = (-700.0, 0.0)
    group_out = nodes.new("NodeGroupOutput")
    group_out.location = (500.0, 0.0)

    to_points = nodes.new("GeometryNodeMeshToPoints")
    to_points.location = (-480.0, 0.0)

    sphere = nodes.new("GeometryNodeMeshIcoSphere")
    sphere.location = (-480.0, -220.0)
    sphere.inputs["Radius"].default_value = 1.0
    sphere.inputs["Subdivisions"].default_value = 1

    point_size = nodes.new("ShaderNodeValue")
    point_size.name = "Point Size"
    point_size.label = "Point radius (m)"
    point_size.location = (-480.0, -400.0)
    point_size.outputs[0].default_value = 0.004

    instance = nodes.new("GeometryNodeInstanceOnPoints")
    instance.location = (-120.0, 0.0)

    set_material = nodes.new("GeometryNodeSetMaterial")
    set_material.location = (140.0, 0.0)
    set_material.inputs["Material"].default_value = material

    links.new(group_in.outputs["Geometry"], to_points.inputs["Mesh"])
    links.new(to_points.outputs["Points"], instance.inputs["Points"])
    links.new(sphere.outputs["Mesh"], instance.inputs["Instance"])
    links.new(point_size.outputs[0], instance.inputs["Scale"])
    links.new(instance.outputs["Instances"], set_material.inputs["Geometry"])
    links.new(set_material.outputs["Geometry"], group_out.inputs["Geometry"])
    return group


def ensure_visualisation(obj, settings):
    material = _ensure_material(settings.point_color)
    group = bpy.data.node_groups.get(GROUP_NAME)
    if group is None:
        group = build_node_group(material)
    if not any(mod.type == "NODES" and mod.node_group is group for mod in obj.modifiers):
        modifier = obj.modifiers.new(GROUP_NAME, "NODES")
        modifier.node_group = group
    size = group.nodes.get("Point Size")
    if size is not None:
        size.outputs[0].default_value = settings.point_radius_m


def ensure_camera(scene):
    camera_data = bpy.data.cameras.new(CAMERA_NAME)
    camera_data.lens = 30.0
    camera = bpy.data.objects.new(CAMERA_NAME, camera_data)
    camera.location = (0.0, -2.5, 0.0)
    # Looking down +Y with +Z up, which is the phone's view direction in this frame.
    camera.rotation_euler = (math.pi / 2.0, 0.0, 0.0)
    scene.collection.objects.link(camera)
    scene.camera = camera


def apply_points(scene, points, settings):
    if not points:
        raise StreamError("the map is empty")
    coordinates = blender_points(points)
    if settings.normalise_scale:
        coordinates = normalised_points(coordinates, settings.target_size_m)
    obj = cloud_object(scene)
    _set_vertices(obj.data, coordinates)
    ensure_visualisation(obj, settings)
    if settings.create_camera and scene.camera is None:
        ensure_camera(scene)


# --------------------------------------------------------------------------------------------
# Properties
# --------------------------------------------------------------------------------------------


def on_poll_changed(self, context):
    # The worker reads the interval on every pass, so a change applies without a restart.
    _STATE.interval = self.poll_seconds


def on_appearance_changed(self, context):
    obj = bpy.data.objects.get(OBJECT_NAME)
    if obj is not None:
        ensure_visualisation(obj, self)


class Scand3dSettings(PropertyGroup):
    source: EnumProperty(
        name="Source",
        items=(
            ("ADB", "Phone (adb)", "Poll the live cloud from the phone over the adb connection"),
            ("FILE", "Local PLY file", "Read a PLY file on this machine, for example a pulled cloud"),
        ),
        default="ADB",
    )
    adb_path: StringProperty(
        name="adb",
        description="Path of the adb executable, 'adb' when it is on PATH",
        default="adb",
    )
    serial: StringProperty(
        name="Serial",
        description="Device serial for 'adb -s'; empty uses the only connected device",
        default="",
    )
    session: StringProperty(
        name="Session",
        description="Capture session directory name, for example 20261104-151236; empty uses the newest",
        default="",
    )
    file_path: StringProperty(
        name="Cloud file",
        description="Point cloud PLY to read when the source is a local file",
        subtype="FILE_PATH",
        default="",
    )
    poll_seconds: FloatProperty(
        name="Interval (s)",
        description="How often the cloud is re-read",
        default=1.0,
        min=0.2,
        soft_max=10.0,
        update=on_poll_changed,
    )
    point_radius_m: FloatProperty(
        name="Point radius (m)",
        description="Radius of the sphere drawn for each point",
        default=0.004,
        min=0.0002,
        soft_max=0.05,
        subtype="DISTANCE",
        update=on_appearance_changed,
    )
    normalise_scale: BoolProperty(
        name="Normalise scale",
        description="Scale the cloud so its median size is the target size; the map's own scale is free",
        default=True,
    )
    target_size_m: FloatProperty(
        name="Target size (m)",
        description="Median distance from the cloud origin when the scale is normalised",
        default=1.0,
        min=0.001,
        subtype="DISTANCE",
    )
    point_color: FloatVectorProperty(
        name="Point colour",
        description="Colour of the point spheres; the stream carries geometry only for now",
        subtype="COLOR",
        size=4,
        min=0.0,
        max=1.0,
        default=(0.95, 0.62, 0.15, 1.0),
        update=on_appearance_changed,
    )
    create_camera: BoolProperty(
        name="Add a view camera",
        description="Add a camera looking at the cloud when the scene has none",
        default=True,
    )
    status: StringProperty(name="Status", default="")
    last_error: StringProperty(name="Error", default="")


class _State:
    def __init__(self):
        self.stream = None
        self.interval = 1.0


_STATE = _State()


def _settings(props):
    return {
        "source": props.source,
        "adb_path": props.adb_path.strip(),
        "serial": props.serial.strip(),
        "session": props.session.strip(),
        "file_path": bpy.path.abspath(props.file_path) if props.file_path else "",
    }


# --------------------------------------------------------------------------------------------
# Operators
# --------------------------------------------------------------------------------------------


class SCAND3D_OT_start_stream(Operator):
    bl_idname = "scand3d.start_stream"
    bl_label = "Start Streaming"
    bl_description = "Poll the point cloud and keep the cloud object updated"

    def execute(self, context):
        stop_stream()
        props = context.scene.scand3d
        stream = CloudStream(_settings(props), lambda: _STATE.interval)
        _STATE.interval = props.poll_seconds
        stream.start()
        _STATE.stream = stream
        props.last_error = ""
        props.status = "connecting"
        if not bpy.app.timers.is_registered(timer_tick):
            bpy.app.timers.register(timer_tick, first_interval=0.05)
        return {"FINISHED"}


class SCAND3D_OT_stop_stream(Operator):
    bl_idname = "scand3d.stop_stream"
    bl_label = "Stop Streaming"
    bl_description = "Stop polling; the cloud object keeps the last map it received"

    def execute(self, context):
        stop_stream()
        context.scene.scand3d.status = "stopped"
        return {"FINISHED"}


class SCAND3D_OT_fetch_once(Operator):
    bl_idname = "scand3d.fetch_once"
    bl_label = "Fetch Once"
    bl_description = "Read the cloud once and update the object, without streaming"

    def execute(self, context):
        props = context.scene.scand3d
        try:
            points, label = _fetch_once(_settings(props))
            apply_points(context.scene, points, props)
        except (StreamError, PlyError) as exc:
            props.last_error = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        props.status = "%i points from %s" % (len(points), label)
        props.last_error = ""
        return {"FINISHED"}


def _fetch_once(settings):
    if settings["source"] == "FILE":
        path = settings["file_path"]
        if not path:
            raise StreamError("no cloud file selected")
        if not os.path.isfile(path):
            raise StreamError("cloud file not found: %s" % path)
        with open(path, "rb") as handle:
            return parse_ply(handle.read()), os.path.basename(path)
    adb_path = settings["adb_path"] or "adb"
    serial = settings["serial"]
    session = settings["session"] or latest_session(adb_path, serial)
    return parse_ply(_read_device_cloud(adb_path, serial, session)), session


def stop_stream():
    stream = _STATE.stream
    _STATE.stream = None
    if stream is not None:
        stream.stop()
    if bpy.app.timers.is_registered(timer_tick):
        bpy.app.timers.unregister(timer_tick)


def timer_tick():
    stream = _STATE.stream
    if stream is None:
        return None
    scene = bpy.context.scene if bpy.context is not None else None
    props = getattr(scene, "scand3d", None) if scene is not None else None
    if props is None:
        return 0.25

    points, error, label, changed = stream.take()
    if changed:
        if error is None and points is not None:
            try:
                apply_points(scene, points, props)
            except StreamError as exc:
                props.last_error = str(exc)
            except RuntimeError as exc:
                props.last_error = str(exc)
            else:
                props.status = "%i points from %s" % (len(points), label)
                props.last_error = ""
        else:
            props.last_error = error or "no cloud"

    if not stream.is_alive():
        _STATE.stream = None
        props.status = "stopped"
        return None
    return max(0.2, props.poll_seconds)


# --------------------------------------------------------------------------------------------
# Panel
# --------------------------------------------------------------------------------------------


class SCAND3D_PT_panel(Panel):
    bl_label = "3Dscand Point Cloud"
    bl_idname = "SCAND3D_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "3Dscand"

    def draw(self, context):
        layout = self.layout
        props = context.scene.scand3d
        streaming = _STATE.stream is not None and _STATE.stream.is_alive()

        layout.prop(props, "source")
        if props.source == "ADB":
            column = layout.column(align=True)
            column.prop(props, "adb_path")
            column.prop(props, "serial")
            column.prop(props, "session")
        else:
            layout.prop(props, "file_path")

        layout.prop(props, "poll_seconds")

        row = layout.row(align=True)
        row.enabled = not streaming
        row.operator("scand3d.start_stream", icon="PLAY")
        row = row.row(align=True)
        row.enabled = streaming
        row.operator("scand3d.stop_stream", icon="PAUSE")
        layout.operator("scand3d.fetch_once", icon="FILE_REFRESH")

        box = layout.box()
        box.label(text="Display")
        box.prop(props, "point_radius_m")
        box.prop(props, "point_color")
        box.prop(props, "normalise_scale")
        column = box.column(align=True)
        column.enabled = props.normalise_scale
        column.prop(props, "target_size_m")
        box.prop(props, "create_camera")

        if props.status:
            layout.label(text=props.status, icon="INFO")
        if props.last_error:
            layout.label(text=props.last_error, icon="ERROR")


# --------------------------------------------------------------------------------------------
# Registration
# --------------------------------------------------------------------------------------------

_CLASSES = (
    Scand3dSettings,
    SCAND3D_OT_start_stream,
    SCAND3D_OT_stop_stream,
    SCAND3D_OT_fetch_once,
    SCAND3D_PT_panel,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.scand3d = bpy.props.PointerProperty(type=Scand3dSettings)


def unregister():
    stop_stream()
    if hasattr(bpy.types.Scene, "scand3d"):
        del bpy.types.Scene.scand3d
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
