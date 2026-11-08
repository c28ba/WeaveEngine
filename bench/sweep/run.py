"""The board sweep: open-source KiCad boards, each stripped of its routing,
routed by WeaveEngine, imported back into KiCad and checked with KiCad's DRC
(design 4.1).

    python bench/sweep/run.py --tier 1                   # the quick boards
    python bench/sweep/run.py --upto 2 --jobs 4          # tiers 1 and 2, four boards at a time
    python bench/sweep/run.py --ramp --baseline main.jsonl
                                    # tier 1, then 2, then 3, stopping at the first tier worse than the baseline
    python bench/sweep/run.py --tier 4                   # the big boards (half an hour each, at most): last
    python bench/sweep/run.py lily58-pro hackrf-one      # these boards

Run it with the Python that has WeaveEngine installed (the repository's
.venv). KiCad 9 or later is needed for the stripping, import and DRC (its
kicad-cli and its Python, which has pcbnew); it is looked for where KiCad
installs itself, or set KICAD_CLI and KICAD_PYTHON. The boards' repositories
(sources.txt, pinned to a commit) are fetched once into WE_SWEEP_DIR (default
~/.cache/weaveengine-sweep), and each run's files go to WE_SWEEP_DIR/work/LABEL.
One JSON line per board goes to --out; compare two runs with compare.py.

On GitHub (.github/workflows/sweep.yml) each board runs on a runner of its
own, so a sweep takes about as long as its slowest board. Push a branch named
sweep/SOMETHING to sweep tiers 1 to 3 of its code, or run the workflow "sweep"
by hand from the Actions tab (the tiers: "1", "1-3", "4"; and a label). The
results go to the branch sweep-results: results/LABEL.jsonl, and LABEL.txt,
compared with the latest sweep of main (git fetch origin sweep-results). The
workflow uses --list for its matrix and --fetch to fill its cache.
"""
import argparse, glob, json, os, re, shutil, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SWEEP = os.path.expanduser(os.environ.get("WE_SWEEP_DIR", "~/.cache/weaveengine-sweep"))
DL = os.path.join(SWEEP, "dl")
PATTERNS = ["*.kicad_pcb", "*.kicad_pro", "*.kicad_dru"]


def kicad() -> tuple[str, str]:
    """(kicad-cli, KiCad's Python): from KICAD_CLI and KICAD_PYTHON, else where KiCad installs itself."""
    cli, py = os.environ.get("KICAD_CLI"), os.environ.get("KICAD_PYTHON")
    if not (cli and py):
        if sys.platform == "darwin":
            for app in ("/Applications/KiCad/KiCad.app", os.path.expanduser("~/Applications/KiCad/KiCad.app"),
                        os.path.expanduser("~/Desktop/KiCad/KiCad.app")):
                if os.path.exists(app):
                    cli = cli or os.path.join(app, "Contents/MacOS/kicad-cli")
                    py = py or os.path.join(app, "Contents/Frameworks/Python.framework/Versions/Current/bin/python3")
                    break
        elif sys.platform == "win32":
            bins = sorted(glob.glob(r"C:\Program Files\KiCad\*\bin"), key=lambda p: [int(x) for x in re.findall(r"\d+", p)])
            if bins:
                cli = cli or os.path.join(bins[-1], "kicad-cli.exe")
                py = py or os.path.join(bins[-1], "python.exe")
        else:
            cli = cli or shutil.which("kicad-cli")
            py = py or "/usr/bin/python3"  # (KiCad's packages put pcbnew in the system Python)
    if not (cli and os.path.exists(cli) and py and os.path.exists(py)):
        sys.exit("KiCad not found: install KiCad 9 or later, or set KICAD_CLI and KICAD_PYTHON")
    return cli, py


def sources() -> dict[str, tuple[str, str, str]]:
    out = {}
    for line in open(os.path.join(HERE, "sources.txt"), encoding="utf-8"):
        if line.strip() and not line.startswith("#"):
            name, url, commit, part = (line.rstrip("\n").split("|") + [""])[:4]
            out[name] = (url, commit, part)
    return out


def boards() -> list[tuple[str, int, str, str]]:
    out = []
    for line in open(os.path.join(HERE, "boards.txt"), encoding="utf-8"):
        if line.strip() and not line.startswith("#"):
            bid, tier, src, path = line.rstrip("\n").split("|")
            out.append((bid, int(tier), src, path))
    return out


def fetch(name: str) -> str:
    """The source's KiCad files, at its pinned commit (fetched once)."""
    url, commit, part = sources()[name]
    dest = os.path.join(DL, name)
    stamp = os.path.join(dest, ".weaveengine-commit")
    if os.path.exists(stamp) and open(stamp).read().strip() == commit:
        return dest
    print(f"fetching {name} ({url} at {commit[:7]})", flush=True)
    shutil.rmtree(dest, ignore_errors=True)
    git = ["git", "-C", dest]
    subprocess.run(["git", "clone", "-q", "--filter=blob:none", "--no-checkout", url, dest], check=True)
    prefix = f"/{part}/**/" if part else "/**/"
    subprocess.run(git + ["sparse-checkout", "set", "--no-cone"] + [prefix + p for p in PATTERNS], check=True)
    subprocess.run(git + ["-c", "advice.detachedHead=false", "checkout", "-q", commit], check=True)
    open(stamp, "w").write(commit)
    return dest


def sh(cmd, timeout=600, log=None, cwd=None):
    t = time.time()
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd, errors="replace")
        out, code = p.stdout + p.stderr, p.returncode
    except subprocess.TimeoutExpired as e:
        out = e.stdout.decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or "")
        code = "timeout"
    if log:
        open(log, "w", encoding="utf-8").write(out)
    return code, out, time.time() - t


def drc(cli: str, pcb: str, out: str) -> dict:
    code, text, _ = sh([cli, "pcb", "drc", "--format", "json", "--severity-error", "--severity-warning", "--refill-zones",
                        "-o", out, pcb], timeout=900)
    try:
        d = json.load(open(out, encoding="utf-8"))
    except Exception:
        return {"error": text[-300:]}
    kinds = {}
    for v in d.get("violations", []):
        kinds[v["type"]] = kinds.get(v["type"], 0) + 1
    return {"kinds": kinds, "unconnected": len(d.get("unconnected_items", []))}


def one(bid: str, src: str, path: str, work: str, args) -> dict:
    """Strip, route, import, check: one board."""
    cli, py = kicad()
    out = os.path.join(work, bid)
    os.makedirs(out, exist_ok=True)
    rec = {"id": bid, "src": f"{src}/{path}"}
    pcb = os.path.join(DL, src, path)
    code, text, _ = sh([py, os.path.join(HERE, "prep.py"), pcb, out], log=os.path.join(out, "prep.log"))
    if code != 0 or not os.path.exists(os.path.join(out, "board.dsn")):
        rec.update(stage="prep", error=text[-500:])
        return rec
    rec.update(json.load(open(os.path.join(out, "prep.json"))))
    route = [args.python, "-m", "weaveengine.cli", "board.dsn", "--quiet"]
    if args.workers:
        route += ["--workers", str(args.workers)]
    code, text, s = sh(route, timeout=args.limit, log=os.path.join(out, "route.log"), cwd=out)
    rec["route_s"], rec["route_code"] = round(s, 1), code
    m = re.search(r"(\d+)/(\d+) connections routed .*?; (\d+) vias\)", text)
    if m:
        rec.update(routed=int(m[1]), connections=int(m[2]), vias=int(m[3]))
    m = re.search(r"-> (OK|VIOLATIONS)\s*$", text.strip(), re.M)
    rec["measured"] = m[1] if m else None
    if "Traceback" in text:
        rec.update(stage="route", error=text[text.index("Traceback"):][-1500:])
        return rec
    if not os.path.exists(os.path.join(out, "board.ses")):
        rec.update(stage="route", error=text[-800:])
        return rec
    code, text, _ = sh([py, os.path.join(HERE, "post.py"), out], log=os.path.join(out, "post.log"))
    if "imported" not in text:
        rec.update(stage="import", error=text[-800:])
        return rec
    rec["drc"] = drc(cli, os.path.join(out, "routed.kicad_pcb"), os.path.join(out, "drc.json"))
    rec["drc_before"] = drc(cli, os.path.join(out, "board.kicad_pcb"), os.path.join(out, "drc_before.json"))
    n = rec.get("layers", 2)
    layers = ",".join(["F.Cu"] + [f"In{i}.Cu" for i in range(1, n - 1)] + ["B.Cu", "Edge.Cuts"])
    sh([cli, "pcb", "export", "svg", "--mode-single", "--page-size-mode", "2", "--exclude-drawing-sheet", "-l", layers,
        "-o", os.path.join(out, "view.svg"), os.path.join(out, "view.kicad_pcb")])
    rec["stage"] = "done"
    return rec


def run(chosen: list[tuple[str, int, str, str]], work: str, args) -> list[dict]:
    for src in sorted({b[2] for b in chosen}):
        fetch(src)

    def task(b):
        bid, tier, src, path = b
        t = time.time()
        rec = one(bid, src, path, work, args)
        rec["tier"], rec["total_s"] = tier, round(time.time() - t, 1)
        with open(args.out, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
        drc_ = rec.get("drc", {})
        print(f"{bid:26s} {rec.get('stage', '?'):6s} {rec.get('routed')}/{rec.get('connections')} routed, "
              f"{rec.get('vias')} vias, {rec.get('route_s')} s, unconnected {drc_.get('unconnected')}", flush=True)
        return rec

    with ThreadPoolExecutor(max(1, args.jobs)) as pool:
        return list(pool.map(task, chosen))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("ids", nargs="*", help="boards to run (default: by tier)")
    ap.add_argument("--tier", type=int, help="only this tier")
    ap.add_argument("--upto", type=int, help="tiers 1 to this (default 3: tier 4, the big boards, only when asked for)")
    ap.add_argument("--ramp", action="store_true", help="tier by tier, stopping at the first one worse than --baseline")
    ap.add_argument("--baseline", help="results of an earlier run, for --ramp")
    ap.add_argument("--out", help="results file (default: results-LABEL.jsonl in WE_SWEEP_DIR)")
    ap.add_argument("--label", help="name of this run (default: the git branch and commit)")
    ap.add_argument("--jobs", type=int, default=1, help="boards at a time (default 1; give each fewer --workers)")
    ap.add_argument("--workers", type=int, help="processes per routing (default: all cores)")
    ap.add_argument("--limit", type=int, default=2400, help="seconds a routing may take (default 2400)")
    ap.add_argument("--python", default=sys.executable, help="the Python with WeaveEngine (default: this one)")
    ap.add_argument("--list", metavar="TIERS", help='print the boards of these tiers ("1", "1-3", "2,4") as JSON and stop')
    ap.add_argument("--fetch", action="store_true", help="fetch every board's source and stop")
    args = ap.parse_args()
    if args.ramp and not args.baseline:
        ap.error("--ramp needs --baseline")
    if args.list:
        tiers = set()
        for part in args.list.replace(" ", "").split(","):
            lo, _, hi = part.partition("-")
            if not (lo.isdigit() and (hi or lo).isdigit()):
                ap.error(f"--list: not tiers: {args.list!r}")
            tiers.update(range(int(lo), int(hi or lo) + 1))
        print(json.dumps([{"id": b[0], "tier": b[1]} for b in boards() if b[1] in tiers]))
        return 0
    if args.fetch:
        for name in sources():
            fetch(name)
        return 0
    label = args.label
    if not label:
        git = lambda *a: subprocess.run(["git", "-C", ROOT, *a], capture_output=True, text=True).stdout.strip()
        label = f"{git('rev-parse', '--abbrev-ref', 'HEAD')}-{git('rev-parse', '--short', 'HEAD')}".replace("/", "-")
    work = os.path.join(SWEEP, "work", label)
    args.out = args.out or os.path.join(SWEEP, f"results-{label}.jsonl")
    os.makedirs(work, exist_ok=True)
    print(f"run {label}: results to {args.out}", flush=True)

    every = boards()
    if args.ids:
        unknown = set(args.ids) - {b[0] for b in every}
        if unknown:
            ap.error(f"unknown boards: {sorted(unknown)}")
        run([b for b in every if b[0] in args.ids], work, args)
        return 0
    tiers = [args.tier] if args.tier else list(range(1, (args.upto or 3) + 1))
    for tier in tiers:
        print(f"-- tier {tier}", flush=True)
        run([b for b in every if b[1] == tier], work, args)
        if args.ramp:
            sys.path.insert(0, HERE)
            from compare import compare
            if compare(args.baseline, args.out):
                print(f"tier {tier} is worse than the baseline: stopping here", flush=True)
                return 1
            print(f"tier {tier}: nothing worse than the baseline", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
