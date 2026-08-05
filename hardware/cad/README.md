# CAD

⬜ **Not yet populated.** This directory is referenced by
[docs/hardware.md](../../docs/hardware.md); the files are being prepared.

## What belongs here

| File | What |
|---|---|
| `anchor_mount.step` + native | Anchor bracket / mast. The mount height is the single most consequential physical parameter in a deployment — see the multipath discussion in docs/hardware.md. |
| `tag_mount.step` + native | Tag mount for the rover, including the **0.550 m** antenna-to-antenna baseline that the rigid estimator is built on. |
| `rover_*.step` | Rocker–bogie chassis, if you are reproducing the platform. |
| `battery_holder.step` | Optional. |

## Conventions

- Ship **STEP (AP214)** for interchange *and* the native format (`.f3d`, `.SLDPRT`) so the
  parametric history is not lost. STEP alone cannot be edited meaningfully.
- Units in **millimetres**.
- Name the origin and the datum in a comment or a companion note — a mount whose origin is
  ambiguous cannot be measured against the anchor frame.
- If a part is 3D printed, note material, layer height and infill. Antenna position
  repeatability depends on the print.
- Add a one-line entry to the table above with every part.
