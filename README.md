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

Most of WeaveEngine's code, tests and documentation were written by an AI
(Anthropic's Claude, in Claude Code), directed, reviewed and tested by the
project's author against real boards and KiCad's design rules check. As with
any autorouter, run KiCad's DRC on the result before you order a board.

## Installing

Download the build for your system from the
[latest release](https://github.com/c28ba/WeaveEngine/releases/latest) and
unpack it.
The builds are not signed, so the first time:

- **macOS:** open `WeaveEngine.app` once (macOS says it cannot check it and
  will not open it), then go to **System Settings → Privacy & Security**,
  scroll down to the note about WeaveEngine and choose **Open Anyway**. Or, in
  Terminal: `xattr -dr com.apple.quarantine WeaveEngine.app`.
- **Windows:** run `WeaveEngine.exe` in the `WeaveEngine` folder; if
  SmartScreen warns, choose **More info**, then **Run anyway**.
- **Linux:** run `WeaveEngine/WeaveEngine` from the unpacked folder. The build
  is made on Ubuntu 22.04 and runs there and on later systems (Debian 12,
  Fedora 36, Mint 21 and newer). If it does not start, it is usually a
  library Qt needs that a desktop install leaves out, most often
  `libxcb-cursor0`; on Ubuntu or Debian:

  ```
  sudo apt install libxcb-cursor0 libxkbcommon-x11-0 libxcb-icccm4 libxcb-keysyms1 \
    libxcb-shape0 libxcb-randr0 libxcb-render-util0 libxcb-image0 libxcb-xinerama0 libegl1
  ```

Or build it yourself (see the end of this page).

## Using it with KiCad

1. In the PCB editor: **File → Export → Specctra DSN…** Save the `.dsn` next to
   the `.kicad_pcb` and `.kicad_pro`, with the same name (KiCad's default), so
   WeaveEngine can read what a DSN leaves out (see Known limitations).
2. In WeaveEngine: **Open DSN…**, then **Route**. You can watch the routing
   take shape; **Stop** at any time keeps the best result so far.
3. **Export SES…**
4. Back in KiCad: **File → Import → Specctra Session…**, then refill the zones
   (**B**) and run the design rules check.

To try it without a board of your own, open one of the open-source examples
in [boards](boards) (credited there).

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
| Margin drawings as board edge | Traces and vias keep the edge clearance from drawings on KiCad's Margin layer too; on by default |
| Safety margin | Extra clearance routed with, so nothing sits exactly on a limit |
| Use vias | Off: every connection stays on one layer |
| Join pads to planes with vias | A pad its net's pour does not reach gets a via beside it, down to the pour; on by default |
| Narrow traces to reach fine-pitch pins | A net whose pins its class's width cannot reach is routed narrower, down to the board's minimum; on by default |
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

`--layers F.Cu B.Cu` routes only the layers named: on a four-layer board with
its planes inside, the inner layers carry no traces, but their pours are still
reached by vias and their copper and keepouts still kept clear of.
`--max-rounds N` caps the rip-up rounds, for a run that must end sooner.
`--time-limit MINUTES` stops bettering the routing when the time is up and
writes the best found so far (finishing it takes a little longer: half a
minute past a one-minute limit on hackrf-one); the app has the same setting.
While it runs, the command line writes the best routing found so far to the
output file each time a pass or raced variant finishes with a better one, and
says so: a run that is stopped (Ctrl+C), crashes or is killed leaves that
behind. A session file is written whole or not at all.

## Known limitations

Found by routing 44 open-source KiCad boards and checking each result with
KiCad's own design rules check.

- **What it is for.** WeaveEngine finds a way to connect everything that keeps
  the clearances. It knows nothing of signal integrity: no impedance control,
  length matching, differential pairs, return paths or crosstalk, and it uses
  each net class's width as given, so give power nets a wide class in KiCad.
  It suits boards of slow signals and modest currents (hobby, logic, keyboard,
  prototype boards). Fast, RF and high-current boards still need routing by
  hand, or at least close checking of what it gives.

- **Keep the KiCad files beside the DSN.** A DSN leaves out much of what KiCad
  checks: the hole clearance and drill sizes, local clearances of parts and
  pads, text and drawings on copper layers, what a rule area that forbids neither tracks nor vias does forbid, the
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
- **Custom design rules (`.kicad_dru`) are read in part.** Beside the DSN,
  the rules that test `A.NetClass`, `A.NetName`, `A.Reference`, an area or a
  courtyard (joined by `&&`, `||`) are taken: a clearance below the board's
  is kept from the pads they match (not from tracks), and a track width
  minimum or maximum is given to the nets they match. Rules about
  differential pairs, lengths, `B.` items and the like are named in the
  import note as skipped, and pins they would allow closer than the
  clearance stay open and named.
- **Copper pours are left to KiCad.** A net with a zone is not routed as
  traces. After importing the session, refill the zones (**B**): until then
  KiCad shows the pour it filled around the old routing, running straight over
  the new traces. A pad its pour does not reach on its own layer (a ground pad
  on top of a board whose ground plane is inside or on the back) gets a via
  beside it down to the pour, placed once everything else is routed; where
  there is no room for one, the pad is named and left for you.
- **Old routing.** Remove existing traces, vias and teardrops before exporting
  if you want them redone: KiCad keeps its own teardrops as zones, and those
  are not taken out when the session is imported. Locked traces and vias are
  kept as they are, routed around, and count as joining a pad to its plane.
- **Drawings on the Margin layer** are board edge, as in KiCad's check, but a
  pad that lies within the edge clearance of one is still reached (straight
  away from the drawing where its part allows), so KiCad reports the pad and
  the track that touches it. Move such drawings to a User layer if they are
  only notes.
- **Closed solder jumpers and net ties.** Copper drawn in a footprint that joins
  pads of two nets is kept clear by other nets' traces, but KiCad gives that
  copper no net, so its check reports a clearance error or a short wherever a
  trace meets one of those pads, however it is routed. Check them by eye.
- **One via size.** Every via is the board's default via. A net class that
  asks for a larger via is not given it, and where its drill is larger, KiCad
  drills the class's hole in the default via on import, leaving a thin ring
  (one via in 44 boards). Check the annular width in the design rules check.
- **Text on copper** is kept clear as a box around it, slightly larger than the
  letters.
- **Large boards take a while.** A board of a few hundred connections routes
  in seconds to a few minutes; a dense one with thousands of pads (KiCad's
  video demo: 2,118 pads, 589 nets) can take over 20 minutes. The app shows
  the routing as it grows, and **Stop** keeps the best result so far. A time
  limit (Settings, or `--time-limit` on the command line) ends the run with
  the best routing found by then; fewer rip-up rounds (`--max-rounds`; 100 by
  default) finish sooner too, with more connections left open.
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
by hand from the Actions tab. For a version tag (`v1.0.0`) it also publishes a
GitHub Release with the three attached.

## License

WeaveEngine is free software under the GNU General Public License, version 3
or (at your option) any later version: see [LICENSE](LICENSE). You may use,
change and share it; if you share a changed version, it has to stay under the
same license, with its source.
