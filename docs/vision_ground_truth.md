# Overhead-camera ground truth

**Entirely optional.** DUNE localizes without it. This subsystem exists to *score* DUNE —
to answer "how wrong is it?" with something better than a tape measure.

It is what produced the 13.5 mm registration RMSE that floors every accuracy number in the
paper.

## Two jobs, often confused

| Job | Method | Used for |
|---|---|---|
| **Placement** | click a pixel + assumed floor plane → world coordinate | putting anchor coordinates into `config/anchors.json` without a tape measure |
| **Tracking** | AprilTag detection + PnP | continuous rover truth during a driven run |

Placement is plain pixel back-projection (`python/vision/backproject.py`) — **no AprilTag
involved**. AprilTag detection is used for tracking, and in
`python/vision/height_calibration.py` to validate the hand-measured mounting height.

## Hardware

| | |
|---|---|
| Camera | Kinect v2 |
| Mount | overhead, optical axis **straight down** |
| Height | measured, then camera-validated (4.0656 m in the reported deployment) |
| Fiducial | AprilTag `tag36h11`, 0.1778 m printed / 0.1700 m effective |
| Truth rate | ≈1.25 Hz |

**Real USB 3.0 is required.** The Kinect v2 needs genuine USB3 isochronous throughput. Intel
and Renesas host controllers are reliably fine; some cheaper VIA/ASMedia chipsets drop
frames.

> The mathematics throughout assumes **zero tilt**. The reported deployment measured 3.06°
> from vertical and corrects for it, but errors grow with distance from the image centre if
> your mount is not near-nadir. Get it as close to straight down as you can.

**Colour stream only — no depth stream is ever opened.** Placement is monocular: colour
frames, calibrated intrinsics, and a known mounting height.

## Install (Ubuntu)

`libfreenect2` — the open-source Kinect v2 driver — must be built first. Then:

```bash
pip install "cython<3.0"
pip install --no-build-isolation pylibfreenect2
pip install -e "python/[vision]"        # opencv-python, pupil-apriltags
```

`pylibfreenect2` is deliberately **not** in `pyproject.toml`'s vision extra: its `setup.py`
needs Cython<3.0 already importable, and pip's build isolation will not supply it in time. A
dependency that cannot be installed by the tool that lists it does not belong in the list.

Linux is what is documented and verified here. It runs on Windows and macOS, but you are on
your own for the driver.

## Procedure

### 1. Intrinsics

```bash
python python/vision/generate_checkerboard.py     # print this
python python/vision/capture_checkerboard.py      # ~20 poses, fill the frame
python python/vision/intrinsic_calibration.py
```

Captures land in `python/vision/calib_images/` and are gitignored — they are large and
regenerable. The fitted result goes to `config/kinect_vision.json`, which **is** tracked.

### 2. Mounting height

```bash
python python/vision/height_calibration.py
```

Place an AprilTag of known size flat on the floor; PnP recovers the camera height and
corrects your tape measurement. Effective tag size differs from printed size — measure the
black border, not the paper.

### 3. World registration

Register the camera frame into the **anchor** frame, so camera truth and UWB estimates are
directly comparable.

```matlab
registerWorldFrameClicks     % click known points
clickAnchorsWorld            % click anchors → config/anchors.json
clickTagTruth                % click tag truth → config/tag_truth.json
```

Python equivalent: `python/vision/world_registration.py`.

Registration RMSE is reported — **13.5 mm** in the deployment. That number is the error floor
of every accuracy figure downstream, so check it before trusting any score. If it is much
worse, your intrinsics, height or clicks are off.

### 4. During a run

```bash
python python/run_localization.py --vision      # adds the VISION panel
```

or capture with truth from the GUI's RUN panel, or MATLAB's `rover_teleop_uwb`. The capture
writes UWB sweeps and AprilTag truth stamped by **one clock**, so alignment is exact rather
than inferred.

### 5. Scoring

```python
from reports.rover import rover_report
r = rover_report('data/rover_runs/....mat', 'results/out',
                 anchors_file='config/anchors_kinect_clicked_20260724.json', tag_z=0.24)
```

The report fits a **no-scale** rigid transform between estimate and truth. No-scale is not
an oversight: a free scale factor would absorb exactly the global error you are trying to
measure. It also refuses reflections, and reports the fit's own RMSE and conditioning so you
can tell a good alignment from a flattering one.

## Honest limits

- **1.25 Hz truth** against ~4 Hz UWB sweeps. Truth is interpolated, and fast motion is where
  that interpolation is worst.
- **Click-based placement** inherits your clicking accuracy and the floor-plane assumption.
- **Not a tracker.** One on-demand snapshot per click; there is no continuous 30 Hz overlay
  and no `get_pose()` API. Continuous tracking would be a materially bigger feature —
  per-frame PnP, velocity by finite differences, live overlay.
- The rover ran at **under 0.1 m/s**, which is unfavourable for moving-accuracy metrics: the
  platform advances about a centimetre between fixes while per-fix noise is several
  centimetres.

## Using different ground truth

Nothing in DUNE requires a Kinect. If you have a motion-capture system or a total station,
the interface is simply truth samples with timestamps on the same clock as the sweeps. Look
at what `python/rover/capture.py` writes and produce the same fields.
