#!/usr/bin/env python3
"""
record_video.py — record the overhead Kinect v2 color feed to a video file.

Standalone bring-up/documentation tool, same family as debug_view.py: it
reuses KinectCapture (color stream only, no depth) and writes frames out with
cv2.VideoWriter. Nothing in the live product path records video — the GUI's
VISION step grabs single JPEG snapshots on demand.

Usage:
    # 30 s of the raw 1920x1080 feed
    python python/vision/record_video.py --out kinect.mp4 --duration 30

    # half-size, live preview window, stop early with 'q' or Ctrl-C
    python python/vision/record_video.py --out kinect.mp4 --scale 0.5 --preview

    # frame-for-frame lossless-ish capture (bigger file, much cheaper to encode)
    python python/vision/record_video.py --out kinect.avi --codec MJPG

Remember the driver's library path before running (docs/kinect_vision_setup.md):
    export LD_LIBRARY_PATH=$HOME/freenect2/lib:$LD_LIBRARY_PATH

Timing note: the container is written at a fixed nominal --fps, but the Kinect
delivers ~30 fps only when the USB3 link and the encoder both keep up. The
measured rate is printed at exit; if it differs from nominal the playback speed
is off, and the ffmpeg command to retime the file without re-encoding is
printed for you.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.vision_config import VisionConfig, DEFAULT_CONFIG_PATH
from vision.kinect_capture import KinectCapture
from vision.backproject import mirror_correct

# Container extension -> a fourcc that opencv-python ships a writer for.
DEFAULT_CODECS = {'.mp4': 'mp4v', '.avi': 'MJPG', '.mkv': 'mp4v', '.mov': 'mp4v'}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default='kinect_recording.mp4',
                   help='output video path (default: %(default)s)')
    p.add_argument('--duration', type=float, default=0.0,
                   help='seconds to record; 0 = until q/Ctrl-C (default: %(default)s)')
    p.add_argument('--fps', type=float, default=30.0,
                   help='nominal frame rate written into the container (default: %(default)s)')
    p.add_argument('--scale', type=float, default=1.0,
                   help='resize factor applied before encoding (default: %(default)s)')
    p.add_argument('--codec', default=None,
                   help='fourcc override, e.g. MJPG, XVID, avc1 (default: from --out extension)')
    p.add_argument('--preview', action='store_true',
                   help='show a live window while recording; q to stop')
    p.add_argument('--mirror-correct', action='store_true',
                   help='undo the color stream mirror, matching what the vision '
                        'pipeline clicks on (default: record the raw feed)')
    p.add_argument('--config', default=None,
                   help=f'vision config JSON (default: {DEFAULT_CONFIG_PATH})')
    return p.parse_args()


def main():
    args = parse_args()
    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cfg = VisionConfig.load(args.config)
    fourcc_str = args.codec or DEFAULT_CODECS.get(out_path.suffix.lower(), 'mp4v')

    cap = KinectCapture()
    cap.start()

    writer = None
    frames_written = 0
    device_frames = 0          # frames the device actually produced
    last_seq = 0
    t_start = None
    try:
        print("waiting for the first frame...")
        while True:
            frame, seq = cap.get_latest_frame_seq()
            if frame is None or seq == last_seq:
                # Poll faster than the ~30 Hz device rate, but only encode a
                # frame the capture thread hasn't handed us before.
                time.sleep(0.005)
                continue
            if last_seq:
                device_frames += seq - last_seq
            last_seq = seq

            if args.mirror_correct:
                frame = mirror_correct(frame, cfg)
            if args.scale != 1.0:
                frame = cv2.resize(frame, None, fx=args.scale, fy=args.scale,
                                   interpolation=cv2.INTER_AREA)

            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(
                    str(out_path), cv2.VideoWriter_fourcc(*fourcc_str),
                    args.fps, (w, h))
                if not writer.isOpened():
                    raise RuntimeError(
                        f"cv2.VideoWriter could not open {out_path} with fourcc "
                        f"'{fourcc_str}' — try --codec MJPG with a .avi output")
                t_start = time.monotonic()
                print(f"recording {w}x{h} @ nominal {args.fps:g} fps "
                      f"({fourcc_str}) -> {out_path}")
                print("press 'q' in the preview window (or Ctrl-C) to stop"
                      if args.preview else "press Ctrl-C to stop")

            writer.write(frame)
            frames_written += 1

            if args.preview:
                cv2.imshow('Kinect recording', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            elapsed = time.monotonic() - t_start
            if args.duration > 0 and elapsed >= args.duration:
                break
            if frames_written % 30 == 0:
                print(f"\r  {frames_written} frames, {elapsed:6.1f} s, "
                      f"{frames_written / max(elapsed, 1e-6):5.1f} fps", end='', flush=True)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        cap.stop()
        if writer is not None:
            writer.release()
        if args.preview:
            cv2.destroyAllWindows()

    if not frames_written:
        print("no frames captured — nothing written")
        return 1

    elapsed = time.monotonic() - t_start
    measured = frames_written / max(elapsed, 1e-6)
    size_mb = out_path.stat().st_size / 1e6
    print(f"\nwrote {frames_written} frames in {elapsed:.1f} s "
          f"({measured:.1f} fps measured) -> {out_path} ({size_mb:.1f} MB)")
    # device_frames counts seq deltas *after* the first frame, so compare it
    # against the frames written after the first one.
    dropped = device_frames - (frames_written - 1)
    if dropped > 0:
        print(f"note: the encoder fell behind the device by {dropped} frames — "
              f"use --scale 0.5 or --codec MJPG (.avi) for a lighter encode")
    if abs(measured - args.fps) > 2.0:
        print(f"note: measured rate differs from the {args.fps:g} fps written into "
              f"the container, so playback speed is off. Retime without re-encoding:\n"
              f"  ffmpeg -r {measured:.2f} -i {out_path} -c copy retimed{out_path.suffix}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
