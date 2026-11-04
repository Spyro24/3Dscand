# Blender add-on: live point cloud stream

`scand3d_stream.py` is a Blender 4.0+ add-on that shows the sparse map the phone builds during a
scan. The app writes its live cloud to `files/sessions/<stamp>/cloud.ply` on every reconstruction
update (`app/src/main/java/com/scand3d/reconstruct/CloudPlyWriter.kt`); the add-on polls that file
over adb and keeps one mesh object updated, so the cloud grows in the viewport while you move the
phone (D044).

## Install

Blender 4.0 or newer.

1. `Edit > Preferences > Add-ons > Install...` and pick `scand3d_stream.py` (or copy it into
   `scripts/addons/` of your Blender configuration), then enable **3Dscand Point Cloud Stream**.
2. Open the 3D viewport sidebar (`N`) and switch to the **3Dscand** tab.

## Use

Phone side:

1. Install the debug APK and start a scan (the **SCAN** capture). The live map is written while the
   capture runs, and again for the final keyframes when it stops.
2. Keep the phone connected over USB with debugging authorised. `adb devices` must list it.

Blender side, source **Phone (adb)**:

- `adb`: the executable, or a full path when `adb` is not on `PATH`.
- `Serial`: empty when exactly one device is connected.
- `Session`: empty uses the newest session on the phone, which is what you want while scanning; a
  name like `20261104-151236` pins one capture.
- **Start Streaming** polls every interval and updates the cloud object. **Fetch Once** does a
  single read. **Stop Streaming** leaves the last cloud in place.

Source **Local PLY file** reads a file on this machine instead, for example a cloud pulled by hand:

```sh
adb exec-out run-as com.scand3d cat files/sessions/<stamp>/cloud.ply > cloud.ply
```

## What the cloud is

The file is a binary little-endian PLY with `x`, `y`, `z` per point, in the reconstruction's own
frame: the first keyframe's camera (x right, y down, z forward), with free scale (D028). The add-on
rotates it into Blender's Z-up frame (`x, z, -y`) and, with **Normalise scale** on, scales it so the
median point distance is **Target size**. That keeps the cloud at a comparable size between scans;
turn it off to see the reconstruction's raw units, which mean nothing metric yet.

Each point is drawn as a small ico sphere instanced by a geometry-nodes modifier, so the cloud
shows in the viewport, Eevee and Cycles. Point radius, colour and the optional view camera are in
the panel.

## Limits

- **Positions only.** The stream carries no colour: the overlay colours live in the native scene
  and there is no bridge call that reads them back (adding one is a public-interface change under
  `agents.md` rule 5; D044).
- **The map is sparse and capped.** The app's live loop keeps at most eight views and re-matches
  every pair on each run, so expect tens to a few hundred points, not a dense cloud.
- **One scan at a time.** Points stay in the frame of the first keyframe of the session being
  streamed; a new capture is a new cloud.
- **adb only.** agents.md forbids network calls during capture, so the transport is the USB adb
  connection (a `run-as` debug read), not a socket.

## Check without a phone

`selftest.py` runs the add-on inside Blender against a fixture PLY and a stand-in `adb`:

```sh
blender --background --factory-startup --python tools/blender/selftest.py
```

It checks the PLY reader (binary, ascii with reordered properties, truncated and non-PLY bodies),
the axis conversion and normalisation, the object/geometry-nodes/material setup, session selection
and fetching over a fake adb, the streaming worker, and renders one frame to prove the points are
visible. It prints `SELFTEST OK: <n> checks` and exits non-zero on the first failure.
