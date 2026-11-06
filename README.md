# WeaveEngine

A PCB autorouter. It reads a board as a Specctra DSN file, routes it, and
writes the traces back as a Specctra session (SES) that KiCad, or any tool that
speaks Specctra, imports.

WeaveEngine plans the routing as topology first (which side of each pad and
trace every connection passes, in a triangulation of the board), so traces
never cross and always fit, and only then works out the geometry: taut traces
with the design rules' clearances, rounded corners and teardrops. How it works,
in detail: [docs/design.md](docs/design.md).

## How this was made

**Most of WeaveEngine was written by large language models.** Its code, tests
and documentation were produced in large part by an AI (Anthropic's Claude,
working in Claude Code), directed, reviewed and tested by the project's author.
The algorithms were chosen, measured and accepted or thrown out by the two
together, and every change was checked against real boards and KiCad's own
design rules check, but you should know how it was made when you decide how
far to trust it, and check what it gives you as you would any autorouter's
output: run KiCad's DRC before you send a board to be made.

## Using it with KiCad

1. In the PCB editor: **File → Export → Specctra DSN…** Save the `.dsn` next to
   the `.kicad_pro`, with the same name, so WeaveEngine finds the project's
   board-edge clearance (a DSN does not carry it).
2. In WeaveEngine: **Open DSN…**, then **Route**. You can watch the routing
   take shape; **Stop** at any time keeps the best result so far.
3. **Export SES…**
4. Back in KiCad: **File → Import → Specctra Session…**, then refill the zones
   (**B**) and run the design rules check.

Copper pours are left to KiCad: a net that has a zone (GND, say) is not routed
as traces, and its pads are connected by the pour when KiCad fills it.

## Settings

**Settings…** shows every setting with a description, grouped as Routing,
Rules, Teardrops and View, and the design rules found in the DSN (warning about
any it does not carry). The ones you are most likely to change:

| Setting | What it does |
|---|---|
| Trace width, clearance, via size | Taken from the DSN unless you set them |
| Copper to board edge | From the KiCad project beside the DSN, else the clearance |
| Safety margin | Extra clearance routed with, so nothing sits exactly on a limit |
| Use vias | Off: every connection stays on one layer |
| Undo long detours, Straighten vias, Round sharp corners | Clean-up after routing; all on by default |
| Teardrops | Added where traces meet pads and vias; written to the SES as fans of traces |

Settings are kept in `settings.json` next to the application.

## Command line

```
weaveengine board.dsn [-o board.ses] [--svg board.svg] [--kicad-pro board.kicad_pro]
```

`weaveengine --help` lists every option. After routing, the command line also
measures the written SES against the rules and says whether it is clean.

## Known limitations

<!-- filled in from the 1.0 board sweep -->

## Building from source

Python 3.13.

```
python -m venv .venv
.venv/bin/pip install -r requirements-build.txt     # Windows: .venv\Scripts\pip ...
.venv/bin/pip install -e . --no-deps
.venv/bin/python main.py                             # the app
.venv/bin/python -m pytest -q                        # the tests
.venv/bin/pyinstaller --noconfirm WeaveEngine.spec   # a standalone app in dist/
```

The app is built on the system it is for: `dist/WeaveEngine.app` on macOS,
`dist/WeaveEngine/` on Windows and Linux. The GitHub Actions workflow in
`.github/workflows/build.yml` builds all three when a version tag (`v1.0.0`) is
pushed, or when started by hand from the Actions tab.

## License

WeaveEngine is free software under the GNU General Public License, version 3
or (at your option) any later version: see [LICENSE](LICENSE). You may use,
change and share it; if you share a changed version, it has to stay under the
same license, with its source.
