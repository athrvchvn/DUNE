#!/usr/bin/env python3
"""
generate_checkerboard.py — render a printable checkerboard calibration target.

Produces a PNG sized so that printing it at 100% scale ("actual size", NOT
"fit to page") gives squares of --square-mm millimetres on paper, at --dpi.
Printer scaling varies by driver/printer, so ALWAYS measure a printed square
with a ruler afterward and pass the REAL measured value (in metres) to
intrinsic_calibration.py's --square — don't just trust the nominal size here.

Defaults match the --cols 9 --rows 6 --square 0.025 example used elsewhere in
this subsystem: 9x6 INNER corners = 10x7 squares, 25 mm/square.

Usage:
    python generate_checkerboard.py --out checkerboard.png
"""
import argparse

import numpy as np
import cv2


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row (squares = cols+1)')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column (squares = rows+1)')
    ap.add_argument('--square-mm', type=float, default=25.0, help='square side length, millimetres')
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--margin-mm', type=float, default=15.0, help='white margin around the board')
    ap.add_argument('--out', default='checkerboard.png')
    args = ap.parse_args()

    squares_x = args.cols + 1
    squares_y = args.rows + 1
    px_per_mm = args.dpi / 25.4
    sq_px = int(round(args.square_mm * px_per_mm))
    margin_px = int(round(args.margin_mm * px_per_mm))

    board_w = squares_x * sq_px
    board_h = squares_y * sq_px
    img = np.full((board_h + 2 * margin_px, board_w + 2 * margin_px), 255, dtype=np.uint8)

    for r in range(squares_y):
        for c in range(squares_x):
            if (r + c) % 2 == 0:
                y0 = margin_px + r * sq_px
                x0 = margin_px + c * sq_px
                img[y0:y0 + sq_px, x0:x0 + sq_px] = 0

    cv2.imwrite(args.out, img)
    print(f"Wrote {args.out}: {squares_x}x{squares_y} squares "
          f"({args.cols}x{args.rows} inner corners), {args.square_mm:.1f} mm/square "
          f"nominal @ {args.dpi} DPI  ({img.shape[1]}x{img.shape[0]} px)")
    print("Print at 100% / 'actual size' (NOT 'fit to page'), then measure a printed "
          "square with a ruler and use the REAL measured value (metres) as --square "
          "in intrinsic_calibration.py.")


if __name__ == '__main__':
    main()
