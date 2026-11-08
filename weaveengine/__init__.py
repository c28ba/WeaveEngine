"""WeaveEngine: a topological PCB autorouter."""
import sys

__version__ = "1.0.1"  # (and pyproject.toml's)
HOME = "https://github.com/c28ba/WeaveEngine"


def safe_output() -> None:
    """Characters the console's encoding cannot show (Windows, with the output
    redirected) are printed as replacements instead of stopping the program."""
    for stream in (sys.stdout, sys.stderr):
        if stream is not None and hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
