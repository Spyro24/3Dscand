"""Headless check of the 3Dscand Blender add-on.

Run with Blender itself, which is the point: it exercises the parts that need `bpy` (geometry
nodes, material, camera) and the adb transport through a stand-in executable, so the add-on can be
verified without a phone.

    blender --background --factory-startup --python tools/blender/selftest.py

Exits with status 1 and prints `SELFTEST FAILED: ...` on the first failed check.
"""

import importlib.util
import os
import struct
import sys
import tempfile

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
ADDON_PATH = os.path.join(HERE, "scand3d_stream.py")

CHECKS = []


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    CHECKS.append(message)


def load_addon():
    spec = importlib.util.spec_from_file_location("scand3d_stream", ADDON_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_binary_ply(path, points):
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex %i\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ) % len(points)
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        for point in points:
            handle.write(struct.pack("<fff", *point))


def write_mixed_ascii_ply(path, points):
    """Colour first and positions last, which a strict reader would get wrong."""
    lines = [
        "ply",
        "format ascii 1.0",
        "element vertex %i" % len(points),
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "property float z",
        "property float y",
        "property float x",
        "end_header",
    ]
    for (x, y, z) in points:
        lines.append("10 20 30 %f %f %f" % (z, y, x))
    with open(path, "w", encoding="ascii") as handle:
        handle.write("\n".join(lines) + "\n")


def write_fake_adb(path, ply_path, sessions):
    """A stand-in adb that answers the two calls the add-on makes."""
    listing = "\\n".join(sessions)
    script = (
        "#!/bin/sh\n"
        "# Stand-in for adb used by the add-on selftest.\n"
        'while [ "$1" = "-s" ]; do shift 2; done\n'
        'if [ "$1" = "shell" ]; then\n'
        "  printf '%s\\n' '__SESSIONS__'\n"
        "  exit 0\n"
        "fi\n"
        'if [ "$1" = "exec-out" ]; then\n'
        "  shift 4\n"
        '  case "$1" in\n'
        "    *cloud.ply) cat '__PLY__'; exit 0 ;;\n"
        '    *) echo "cat: $1: No such file or directory" >&2; exit 1 ;;\n'
        "  esac\n"
        "fi\n"
        'echo "unsupported fake adb call: $*" >&2\n'
        "exit 2\n"
    ).replace("__SESSIONS__", listing).replace("__PLY__", ply_path)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(script)
    os.chmod(path, 0o755)


def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    scene = bpy.context.scene
    scene.camera = None
    world = scene.world or bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    background = world.node_tree.nodes.get("Background")
    if background is not None:
        background.inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)
        background.inputs["Strength"].default_value = 1.0


def instance_count(obj):
    """Geometry-node instances of `obj`: `to_mesh()` does not realise them, the depsgraph counts them."""
    depsgraph = bpy.context.evaluated_depsgraph_get()
    return sum(
        1
        for instance in depsgraph.object_instances
        if instance.is_instance and instance.parent is not None and instance.parent.name == obj.name
    )


def main():
    module = load_addon()
    directory = tempfile.mkdtemp(prefix="scand3d-blender-")
    ply_path = os.path.join(directory, "cloud.ply")
    ascii_path = os.path.join(directory, "mixed.ply")
    fake_adb = os.path.join(directory, "adb")

    # Four points on the camera axes plus one off-axis, values chosen so a swapped axis shows.
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    ]
    write_binary_ply(ply_path, points)
    write_mixed_ascii_ply(ascii_path, points)

    parsed = module.parse_ply(open(ply_path, "rb").read())
    check(parsed == points, "binary PLY parses to the points it holds")

    mixed = module.parse_ply(open(ascii_path, "rb").read())
    check(mixed == points, "ascii PLY properties are read by name, not by position")

    short = open(ply_path, "rb").read()[:-4]
    try:
        module.parse_ply(short)
    except module.PlyError:
        CHECKS.append("a truncated binary body is rejected")
    else:
        raise AssertionError("a truncated binary body was accepted")

    try:
        module.parse_ply(b"<html>not a cloud</html>")
    except module.PlyError:
        CHECKS.append("a non-PLY body is rejected")
    else:
        raise AssertionError("a non-PLY body was accepted")

    check(
        module.blender_points([(1.0, 2.0, 3.0)]) == [(1.0, 3.0, -2.0)],
        "camera axes (+x right, +y down, +z forward) become Blender Z-up",
    )
    scaled = module.normalised_points(module.blender_points([(0.0, 0.0, 4.0), (0.0, 0.0, 8.0)]), 1.0)
    check(
        abs(max(abs(value) for point in scaled for value in point) - 1.0) < 1e-9,
        "normalising puts the cloud at the target size",
    )

    module.register()
    clear_scene()
    scene = bpy.context.scene
    settings = scene.scand3d

    # Local file source: the cloud must land on one mesh object with converted coordinates.
    settings.source = "FILE"
    settings.file_path = ply_path
    settings.normalise_scale = False
    settings.create_camera = True
    result = bpy.ops.scand3d.fetch_once()
    check(result == {"FINISHED"}, "fetch once from a local PLY file finishes")
    obj = bpy.data.objects.get(module.OBJECT_NAME)
    check(obj is not None and obj.type == "MESH", "the cloud object is created")
    check(len(obj.data.vertices) == 4, "the cloud object holds every point")
    coordinates = [tuple(vertex.co) for vertex in obj.data.vertices]
    check(
        any(abs(co[1] - 1.0) < 1e-6 and abs(co[2]) < 1e-6 for co in coordinates),
        "camera +z (forward) maps to Blender +y",
    )
    check(scene.camera is not None, "a view camera is added when the scene has none")
    check(instance_count(obj) == 4, "geometry nodes instance one sphere per point")

    # The same cloud through the adb transport, with the newest session picked automatically.
    write_fake_adb(fake_adb, ply_path, ["20261104-101010", "20261104-120000"])
    settings.source = "ADB"
    settings.adb_path = fake_adb
    settings.serial = "TESTDEVICE"
    settings.session = ""
    result = bpy.ops.scand3d.fetch_once()
    check(result == {"FINISHED"}, "fetch once over the fake adb finishes")
    check("20261104-120000" in settings.status, "the newest session is selected")
    check(settings.last_error == "", "no error is left behind after a successful fetch")

    settings.session = "20261104-101010"
    result = bpy.ops.scand3d.fetch_once()
    check(result == {"FINISHED"}, "an explicitly named session is used")

    # Streaming: start the worker, then drive the timer by hand because Blender runs no timers
    # without an event loop in the background.
    settings.session = ""
    settings.poll_seconds = 0.2
    settings.point_radius_m = 0.02
    result = bpy.ops.scand3d.start_stream()
    check(result == {"FINISHED"}, "streaming starts")
    for _ in range(200):
        module.timer_tick()
        if "points" in settings.status:
            break
        import time

        time.sleep(0.05)
    check(
        settings.status.startswith("4 points"),
        "the stream applies a cloud from the phone (%s)" % settings.status,
    )
    bpy.ops.scand3d.stop_stream()

    # Render proof: only the cloud exists, so any lit pixel is an instanced point sphere.
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 4
    # This Blender is built without OpenImageDenoise, and rendering fails if denoising asks for it.
    scene.cycles.use_denoising = False
    scene.render.resolution_x = 64
    scene.render.resolution_y = 64
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    render_path = os.path.join(directory, "cloud.png")
    scene.render.filepath = render_path
    bpy.ops.render.render(write_still=True)
    image = bpy.data.images.load(render_path)
    pixels = list(image.pixels)
    check(
        any(pixels[index] > 0.05 for index in range(0, len(pixels), 4)),
        "the cloud renders at least one lit pixel",
    )

    module.unregister()
    print("SELFTEST OK: %i checks" % len(CHECKS))
    return 0


try:
    sys.exit(main())
except AssertionError as failure:
    print("SELFTEST FAILED: %s" % failure)
    sys.exit(1)
except Exception as error:  # noqa: BLE001 - any crash is a failed test
    import traceback

    traceback.print_exc()
    print("SELFTEST FAILED: %s" % error)
    sys.exit(1)
