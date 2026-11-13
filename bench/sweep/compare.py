"""Two sweep runs side by side, board by board:

    python bench/sweep/compare.py BASELINE.jsonl NEW.jsonl

Counted against each board as it was before routing (its own DRC with the
routing stripped), so only what the routing causes is counted. A board is
worse when it fails where it did not, routes fewer connections, leaves more
open in KiCad, has more DRC errors the routing causes, or more islands of pour
(isolated copper: pieces of a zone joined to nothing). Solder mask bridges
and starved thermals are shown apart: they depend on the board's own mask and
zone settings more than on the routing. Only the boards in NEW are compared.
Exit status 1 if any is worse. Also shown, not judged: how far the pour estimate's
open items and islands are from KiCad's ("estimate off by"), where NEW has them.
"""
import json, sys

ROUTING = {"clearance", "shorting_items", "hole_clearance", "copper_edge_clearance", "tracks_crossing", "track_dangling",
           "via_dangling", "annular_width", "hole_to_hole", "holes_co_located", "drill_out_of_range", "via_diameter",
           "connection_width", "track_width"}
APART = {"solder_mask_bridge", "starved_thermal"}
ISLANDS = {"isolated_copper"}


def load(path: str) -> dict[str, dict]:
    """The last finished result of each board (a later run of a board replaces an earlier one)."""
    out = {}
    for line in open(path, encoding="utf-8"):
        d = json.loads(line)
        if "drc" in d or d["id"] not in out:
            out[d["id"]] = d
    return out


def caused(d: dict, kinds: set[str]) -> tuple[int | None, dict[str, int]]:
    """DRC errors of these kinds the routing added to the board, in all and by kind."""
    if "kinds" not in d.get("drc", {}) or "kinds" not in d.get("drc_before", {}):
        return None, {}
    after, before = d["drc"]["kinds"], d["drc_before"]["kinds"]
    extra = {k: after.get(k, 0) - before.get(k, 0) for k in kinds if after.get(k, 0) > before.get(k, 0)}
    return sum(extra.values()), extra


def estimate_off(d: dict) -> tuple[int | None, int | None] | None:
    """How far the pour estimate's open items (connections left unrouted and pads cut off) and islands are from what
    KiCad's DRC finds (unconnected items, isolated copper); None where a run has no count."""
    if d.get("routed") is None or d.get("cut_off") is None or "kinds" not in d.get("drc", {}):
        return None
    opened = d.get("est_open", d["connections"] - d["routed"] + d["cut_off"]) - d["drc"]["unconnected"]
    islands = d["est_islands"] - d["drc"]["kinds"].get("isolated_copper", 0) if "est_islands" in d else None
    return opened, islands


def compare(base_path: str, new_path: str) -> bool:
    """Prints the comparison; True if any board is worse."""
    base, new = load(base_path), load(new_path)
    worse_any = False
    total = {"drc": [0, 0], "islands": [0, 0], "apart": [0, 0], "open": [0, 0], "cut": [0, 0], "time": [0.0, 0.0]}
    drift = [0, [0, 0], [0, 0]]  # boards; |estimated open - KiCad's open| and |estimated islands - KiCad's isolated copper|, each run
    for bid in sorted(new):
        a, b = base.get(bid), new[bid]
        if a is None:
            print(f"{bid:26s} (not in the baseline)")
            continue
        na, _ = caused(a, ROUTING)
        nb, kinds = caused(b, ROUTING)
        ma, _ = caused(a, APART)
        mb, _ = caused(b, APART)
        ia, _ = caused(a, ISLANDS)
        ib, _ = caused(b, ISLANDS)
        ua, ub = a.get("drc", {}).get("unconnected"), b.get("drc", {}).get("unconnected")
        why = []
        if a.get("stage") == "done" and b.get("stage") != "done":
            why.append(f"failed at {b.get('stage')}")
        if (b.get("routed") or 0) < (a.get("routed") or 0):
            why.append("fewer routed")
        if ua is not None and ub is not None and ub > ua:
            why.append("more open")
        if na is not None and nb is not None and nb > na:
            why.append("more DRC")
        if ia is not None and ib is not None and ib > ia:
            why.append("more islands")
        ta, tb = a.get("route_s") or 0, b.get("route_s") or 0
        slow = " (slower)" if tb > 1.3 * ta and tb - ta > 5 else ""
        worse_any |= bool(why)
        ca, cb = a.get("cut_off"), b.get("cut_off")  # (pads cut off from their pour, by the estimate: older runs have none)
        for key, x, y in (("drc", na, nb), ("islands", ia, ib), ("apart", ma, mb), ("open", ua, ub), ("cut", ca, cb), ("time", ta, tb)):
            if x is not None and y is not None:
                total[key][0] += x
                total[key][1] += y
        off = ""
        ea, eb = estimate_off(a), estimate_off(b)
        if ea is not None and eb is not None:
            drift[0] += 1
            for k in range(2):
                drift[1 + k][0] += abs(ea[k] if ea[k] is not None else 0)
                drift[1 + k][1] += abs(eb[k] if eb[k] is not None else 0)
            off = "  estimate off by open {} -> {:+d}{}".format("?" if ea[0] is None else f"{ea[0]:+d}", eb[0],
                                                                  "" if eb[1] is None else f" islands {eb[1]:+d}")
        # Traces of nets narrowed to reach fine-pitch pads: tapered / narrow all along (older runs have none).
        taper = f"tapered/narrow {a.get('tapered')}/{a.get('narrow')} -> {b.get('tapered')}/{b.get('narrow')}  " if "tapered" in a or "tapered" in b else ""
        # Meetings of one net's copper under the least angle (``realize.acute``; older runs have none): not judged.
        acute = f"acute {sum(a['acute'].values()) if 'acute' in a else '?'} -> {sum(b['acute'].values()) if 'acute' in b else '?'}  "             if "acute" in a or "acute" in b else ""
        print(f"{bid:26s} routed {a.get('routed')}/{a.get('connections')} -> {b.get('routed')}/{b.get('connections')}  "
              f"drc {na} -> {nb} {kinds or ''}  islands {ia} -> {ib}  mask/thermal {ma} -> {mb}  open {ua} -> {ub}  "
              f"{'' if ca is None and cb is None else f'cut off {ca} -> {cb}  '}{taper}{acute}{ta:.0f} -> {tb:.0f} s{slow}{off}" + (f"  WORSE: {', '.join(why)}" if why else ""))
    d, i, m, o, c, t = total["drc"], total["islands"], total["apart"], total["open"], total["cut"], total["time"]
    print(f"TOTAL drc {d[0]} -> {d[1]}, islands {i[0]} -> {i[1]}, mask/thermal {m[0]} -> {m[1]}, open {o[0]} -> {o[1]}, "
          f"cut off {c[0]} -> {c[1]} (where both have it), "
          f"routing time {t[0]:.0f} -> {t[1]:.0f} s" + ("  (some boards worse)" if worse_any else ""))
    if drift[0]:
        print(f"estimate against KiCad on {drift[0]} boards: open items off by {drift[1][0]} -> {drift[1][1]} in all, "
              f"islands off by {drift[2][1]} (where NEW has them)")
    return worse_any


if __name__ == "__main__":
    sys.exit(1 if compare(sys.argv[1], sys.argv[2]) else 0)
