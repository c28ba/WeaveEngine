"""prep.py SRC.kicad_pcb OUTDIR  (KiCad's python) -> OUTDIR/board.kicad_pcb without tracks or vias, board.dsn, the project's rules."""
import sys, os, json, shutil
if sys.platform == "win32":
    # Without a wx app, some of KiCad's checks on loading a board raise a wxWidgets assert, which on Windows is a
    # dialog that waits, unseen, for a click when run over SSH or from a service: with one made and asserts off, none.
    import wx
    _app = wx.App(False)
    wx.DisableAsserts()
import pcbnew
src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
b = pcbnew.LoadBoard(src)
tracks = list(b.GetTracks())
vias = sum(1 for t in tracks if t.Type() == pcbnew.PCB_VIA_T)
for t in tracks:
    b.Remove(t)
pcb = os.path.join(out, "board.kicad_pcb")
for ext in (".kicad_pro", ".kicad_dru"):
    p = src[:-len(".kicad_pcb")] + ext
    if os.path.exists(p):
        shutil.copy(p, os.path.join(out, "board" + ext))
b.Save(pcb)
b = pcbnew.LoadBoard(pcb)
ok = pcbnew.ExportSpecctraDSN(b, os.path.join(out, "board.dsn"))
info = {"src": src, "layers": b.GetCopperLayerCount(), "footprints": len(b.GetFootprints()), "pads": sum(len(f.Pads()) for f in b.GetFootprints()),
        "nets": b.GetNetCount(), "orig_tracks": len(tracks) - vias, "orig_vias": vias, "zones": len(b.Zones()), "dsn": bool(ok)}
json.dump(info, open(os.path.join(out, "prep.json"), "w"))
print(json.dumps(info))
sys.stdout.flush()
os._exit(0)  # (not a normal exit: on Windows the wx app made above can hang its teardown for ever)
