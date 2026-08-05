"""Operator-facing command-line tools.

Each module here is a script you run directly — anchor self-survey, calibration
capture, bias analysis. They sit apart from the library (``dune/``) and the
live host (``host/``, ``run_localization.py``) because they are entered from a
shell, one at a time, by a person standing next to the hardware.

Run them from the repository root, e.g. ``python python/tools/run_survey.py``.
"""
