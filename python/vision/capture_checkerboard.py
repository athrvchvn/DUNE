#!/usr/bin/env python3
"""
capture_checkerboard.py — interactive frame grabber for intrinsic calibration.

Shows the live Kinect color feed; SPACE saves the current frame as a numbered
PNG into --out, 'q' quits. Feed the saved images to intrinsic_calibration.py.

Usage:
    python capture_checkerboard.py --out calib_images/

For an overhead mount, holding the board close to the lens usually means
being up a ladder with both hands full — no free hand for SPACE. Use
--auto-interval instead: it saves on a timer, no keypress needed. Get in
position before starting the script (or before climbing), then just hold/move
the board; Ctrl+C from the keyboard when you're done and back down.

    python capture_checkerboard.py --out calib_images_close/ --auto-interval 2
"""
import sys
import time
import argparse
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.kinect_capture import KinectCapture


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='calib_images', help='output directory for saved frames')
    ap.add_argument('--auto-interval', type=float, default=None,
                     help='auto-save a frame every N seconds instead of waiting for '
                          'SPACE (for when your hands are full holding the board) — '
                          'Ctrl+C to stop')
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = KinectCapture()
    cap.start()
    if args.auto_interval:
        print(f"AUTO mode: saving a frame every {args.auto_interval}s — no keypress needed.")
        print("Get in position and hold/move the board; Ctrl+C here when done.")
    else:
        print("SPACE = save frame, 'q' = quit. Move the checkerboard between saves "
              "(varied position/tilt/distance) — aim for 15-20 images.")
    n = 0
    got_first_frame = False
    last_auto_save = 0.0
    try:
        while True:
            frame = cap.get_latest_frame()
            if frame is None:
                time.sleep(0.02)   # yield instead of hammering the lock/GIL
                continue
            if not got_first_frame:
                got_first_frame = True
                mode = "auto-capturing" if args.auto_interval else "opening window"
                print(f"first frame received: {frame.shape} — {mode}...")
            preview = frame.copy()
            cv2.putText(preview, f"saved: {n}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('capture checkerboard (SPACE=save, q=quit)', preview)
            key = cv2.waitKey(1) & 0xFF

            if args.auto_interval:
                now = time.time()
                if now - last_auto_save >= args.auto_interval:
                    path = out_dir / f"calib_{n:03d}.png"
                    cv2.imwrite(str(path), frame)
                    print(f"\asaved {path}")   # \a = terminal bell, audible cue
                    n += 1
                    last_auto_save = now
                if key == ord('q'):
                    break
            else:
                if key == ord(' '):
                    path = out_dir / f"calib_{n:03d}.png"
                    cv2.imwrite(str(path), frame)
                    print(f"saved {path}")
                    n += 1
                elif key == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.stop()
        cv2.destroyAllWindows()

    print(f"Done. {n} images in {out_dir}")


if __name__ == '__main__':
    main()
