# Example boards

Four open-source KiCad boards to try WeaveEngine on, from small to large. Each
folder holds the board as a Specctra DSN and the `.kicad_pcb` and `.kicad_pro`
beside it, so WeaveEngine reads the rules a DSN leaves out. Open the `.dsn` in
WeaveEngine; import the session into the `.kicad_pcb`.

| Board | Layers | Pads | Connections | Routed in the 1.0 sweep |
|---|---|---|---|---|
| [pic-programmer](pic-programmer) | 2 | 247 | 86 | all 86, no vias, 3 s |
| [lily58-pro](lily58-pro) | 2 | 942 | 311 | all 311, 90 vias, 41 s |
| [coldfire](coldfire) | 4 | 825 | 313 | all 313, 217 vias, 3 min |
| [hackrf-one](hackrf-one) | 4 | 1,639 | 545 | all 545, 467 vias, 6 min |

All four routed every connection with no design-rule errors caused by the
routing (KiCad 10's DRC, zones refilled). Times are on a 10-core Apple M4
Mac.

## Where they come from

These files are **not** under WeaveEngine's GPL: each keeps the license of the
design it was taken from, given below with its full text in
[LICENSES](LICENSES).

What was changed, in every board: all tracks and vias were removed, the board
was saved again by KiCad 10.0.6 (which may update the file's format), and the
DSN was exported from it by KiCad (File → Export → Specctra DSN). The project
file (`.kicad_pro`) is the original, renamed. Nothing else was changed.

- **pic-programmer**: the *pic_programmer* demo from the
  [KiCad](https://gitlab.com/kicad/code/kicad) source tree
  (`demos/pic_programmer`, commit `085c618`), by the KiCad developers.
  Licensed under [CC BY-SA 4.0](LICENSES/CC-BY-SA-4.0.txt); these changed
  files are shared under the same license.
- **coldfire**: the *kit-dev-coldfire-xilinx_5213* demo from the
  [KiCad](https://gitlab.com/kicad/code/kicad) source tree
  (`demos/kit-dev-coldfire-xilinx_5213`, commit `085c618`), by the KiCad
  developers. Licensed under [CC BY-SA 4.0](LICENSES/CC-BY-SA-4.0.txt); these
  changed files are shared under the same license.
- **lily58-pro**: the *Lily58 Pro* keyboard by Naoki Katahira (kata0510),
  from [github.com/kata0510/Lily58](https://github.com/kata0510/Lily58)
  (`Pro/PCB`, commit `58c6497`). Copyright (c) 2018 Naoki Katahira, under the
  [MIT License](LICENSES/MIT-lily58.txt).
- **hackrf-one**: the *HackRF One* software-defined radio by Great Scott
  Gadgets, from
  [github.com/greatscottgadgets/hackrf](https://github.com/greatscottgadgets/hackrf)
  (`hardware/hackrf-one`, commit `3a63b29`). Copyright (c) 2012-2022, Great
  Scott Gadgets, under the
  [CERN Open Hardware Licence Version 2 - Permissive](LICENSES/CERN-OHL-P-2.0-hackrf-one.txt).
  "HackRF" is a trademark of Great Scott Gadgets, used here only to name the
  design.

Thanks to their authors for publishing them.
