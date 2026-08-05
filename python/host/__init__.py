"""Host application scaffolding: context object, message bus, job registry, ingest.

The live app used to be a single 1904-line script with ~18 module-level globals
and every long operation running inline on the event loop. That shape cannot
carry the ~29 tools the GUI needs to drive, so the machinery moved here:

    bus.py      the single outbound choke point + the NDJSON session recorder
    jobs.py     JobRegistry — long operations off the event loop, with progress
    ingest.py   transport reader threads + TagLink (shared access to the tag)
    app.py      App — the context object that replaces the globals

Nothing in this package may hold module-level mutable state. Handlers and job
bodies take `app` as their first argument.
"""
