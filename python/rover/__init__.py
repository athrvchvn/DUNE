"""Rover drive layer: (V, omega) in, Teensy JSON out.

Reimplemented from the MMS codebase, whose wire protocol is fully documented by
its own firmware source. Three modules, deliberately separated by what can be
tested without hardware:

    ackermann.py   pure geometry — exhaustively testable against a golden table
    teensy.py      the newline-JSON link — testable against a fake serial port
    controller.py  the 10 Hz loop and the dead-man — needs one of the above

SAFETY, stated once here because it governs the whole layer: the firmware has NO
command watchdog. If this host dies, the motors keep running. Every stop path is
therefore a host-side responsibility, and none of them is safety-rated. The only
true emergency stop is an inline power switch.
"""
