"""post.py OUTDIR  (KiCad's python) -> routed.kicad_pcb (board.ses imported) and view.kicad_pcb (the same, pours unfilled, for pictures)."""
import sys, os
import pcbnew
out = sys.argv[1]
b = pcbnew.LoadBoard(os.path.join(out, "board.kicad_pcb"))
ok = pcbnew.ImportSpecctraSES(b, os.path.join(out, "board.ses"))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())  # as KiCad's B: the pours round the new traces, not the old
b.Save(os.path.join(out, "routed.kicad_pcb"))
for z in b.Zones():
    z.UnFill()
b.Save(os.path.join(out, "view.kicad_pcb"))
print("imported" if ok else "IMPORT FAILED", len(b.GetTracks()), "tracks")
