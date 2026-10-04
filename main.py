"""WeaveEngine desktop application.

    python main.py [board.dsn]

Build a standalone app with:

    pyinstaller WeaveEngine.spec
"""
import multiprocessing
import sys

if __name__ == "__main__":
    # First thing: in a packaged app, worker processes start by re-running this
    # executable, and this call is where they turn into workers.
    multiprocessing.freeze_support()
    from weaveengine.app import main

    sys.exit(main())
