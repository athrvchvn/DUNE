"""
kinect_capture.py — background color-stream reader for the Kinect v2, via
pylibfreenect2 (Linux, libfreenect2). Depth is never opened — pose/placement
in this subsystem is monocular-only (see docs/kinect_vision_setup.md).

pylibfreenect2 is imported lazily inside start(), so importing this module
(and the rest of python/vision/) never requires it to be installed; the main
UWB pipeline only pays that cost if --vision is passed AND a grab is
attempted.

The device is opened once and a background thread keeps a single "latest
frame" slot filled, so the GUI's on-demand "Grab frame" click reads whatever
is already there instead of paying the multi-second device-init cost per
click.

NOTE: pylibfreenect2's exact frame-access API has varied across releases
(dict-style `frames["color"]` vs. `frames[FrameType.Color]`, and
`device.start()` vs. `device.startStreams(rgb=..., depth=...)` for a
color-only stream). This has not been exercised against real hardware in this
environment — verify against your installed pylibfreenect2 version per
docs/kinect_vision_setup.md and adjust `_loop`/`start` if the API differs.
"""
import threading
from typing import Optional

import numpy as np


class KinectCapture:
    def __init__(self):
        self._frame: Optional[np.ndarray] = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fn = None
        self._device = None
        self._listener = None

    def start(self) -> None:
        try:
            from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType
            from pylibfreenect2 import CpuPacketPipeline
        except ImportError as e:
            raise RuntimeError(
                "pylibfreenect2 not installed — see docs/kinect_vision_setup.md "
                "(build libfreenect2, then pip install pylibfreenect2)"
            ) from e

        fn = Freenect2()
        if fn.enumerateDevices() == 0:
            raise RuntimeError("No Kinect v2 device found (check the USB 3.0 connection)")

        serial = fn.getDeviceSerialNumber(0)
        # Explicit CPU pipeline: color-only capture doesn't need GPU depth
        # acceleration, and this avoids a hard dependency on OpenGL/OpenCL
        # drivers being present just to open the device.
        device = fn.openDevice(serial, pipeline=CpuPacketPipeline())
        listener = SyncMultiFrameListener(FrameType.Color)
        device.setColorFrameListener(listener)

        if hasattr(device, 'startStreams'):
            device.startStreams(rgb=True, depth=False)
        else:
            device.start()

        self._fn, self._device, self._listener = fn, device, listener
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            # milliseconds must be passed by keyword — the first positional
            # slot is an optional pre-allocated FrameMap, not the timeout.
            frames = self._listener.waitForNewFrame(milliseconds=1000)
            if frames is None:
                continue
            color = frames["color"]
            # Kinect color frames are BGRX (4 channels) — drop the alpha/pad byte.
            arr = np.asarray(color.asarray())[:, :, :3].astype(np.uint8).copy()
            with self._lock:
                self._frame = arr
                self._seq += 1
            self._listener.release(frames)

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Return a copy of the most recently captured BGR frame, or None."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def get_latest_frame_seq(self) -> tuple[Optional[np.ndarray], int]:
        """Same as get_latest_frame(), plus a counter that increments once per
        captured frame. A consumer that polls faster than the ~30 Hz device
        rate compares the counter to tell a genuinely new frame from a repeat
        of the one it already has (see record_video.py)."""
        with self._lock:
            frame = None if self._frame is None else self._frame.copy()
            return frame, self._seq

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._device:
            self._device.stop()
            self._device.close()
