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

`weaveengine --help` lists every option, and `weaveengine --version` prints the
version. After routing, the command line also measures the written SES against
the rules and says whether it is clean. Pads no trace can reach are named, each
with the reason: outside the board, over the edge clearance, in a keepout, or
too close to other copper.

## Known limitations

Found by routing 44 open-source KiCad boards and checking each result with
KiCad's own design rules check.

- **Keep the KiCad files beside the DSN.** A DSN leaves out much of what KiCad
  checks: the hole clearance and drill sizes, local clearances of parts and
  pads, text and drawings on copper layers, what a rule area forbids, the
  board's own minimum clearance, track width and via ring, and the true shape
  of custom pads (the DSN gives only the convex hull of their copper, which
  can cover a neighbouring pin). WeaveEngine reads these from the
  `.kicad_pcb` and `.kicad_pro` of the same name next to the DSN, and says what
  it took. Without them (a DSN from another tool, or moved elsewhere) the
  result can break those rules.
- **Fine-pitch parts.** Where a net's pins cannot be reached at its class's
  trace width, WeaveEngine routes that net narrower, down to the board's
  minimum track width (never below 0.127 mm), and says which nets it narrowed.
  Settings can turn this off.
- **Custom design rules (`.kicad_dru`) are not read.** Pins closer together
  than the clearance itself are usually routed in KiCad under a custom rule.
  WeaveEngine cannot reach them; it leaves their connections open and names
  the pads.
- **Copper pours are left to KiCad.** A net with a zone is not routed as
  traces. After importing the session, refill the zones (**B**): until then
  KiCad shows the pour it filled around the old routing, running straight over
  the new traces. Where the pour does not reach a pad (an inner plane needs a
  via down to it), that pad stays unconnected until you add the via in KiCad.
- **Old routing.** Remove existing traces, vias and teardrops before exporting
  if you want them redone: KiCad keeps its own teardrops as zones, and those
  are not taken out when the session is imported.
- **Drawings on the Margin layer** are not treated as board edge, though
  KiCad's check holds traces to the edge clearance from them.
- **Text on copper** is kept clear as a box around it, slightly larger than the
  letters.
- **On Windows** a routing pass runs on one core; several passes with different
  settings still race on all of them, as on macOS and Linux.
- **Always run KiCad's design rules check** on the imported result before you
  order a board.

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
`.github/workflows/build.yml` builds all three on every push, or when started
by hand from the Actions tab. For a version tag (`v1.0.0`) it also drafts a
GitHub Release with the three attached, to be looked over and published by
hand.

## License

WeaveEngine is free software under the GNU General Public License, version 3
or (at your option) any later version: see [LICENSE](LICENSE). You may use,
change and share it; if you share a changed version, it has to stay under the
same license, with its source.
