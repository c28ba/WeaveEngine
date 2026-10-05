"""User settings, kept in ``settings.json`` (design section 23).

Every knob a front end exposes lives here, with its default and a one-line
description. The file is plain JSON, so it can be edited by hand; unknown keys
are ignored and missing ones take their defaults.
"""
import json
import os
import sys
from dataclasses import asdict, dataclass, fields

from weaveengine.board import Rules
from weaveengine.plan.context import Options
from weaveengine.topo.costs import CostParams

# name -> (label, description, group)
DESCRIPTIONS = {
    "workers": ("Worker processes", "How many processor cores routing may use", "Speed"),
    "portfolio": ("Variants raced per pass", "Each pass is tried several ways at once and the best is kept", "Speed"),
    "seed": ("Random seed", "Same seed and settings give the same result", "Speed"),
    "max_via_rounds": ("Via passes", "How many times vias may be added and the board routed again", "Routing"),
    "drc_rounds": ("Design-rule repair rounds", "Reroute attempts for traces that fail the final check", "Routing"),
    "use_vias": ("Insert vias", "Allow vias for connections no single layer can complete", "Routing"),
    "live_vias": ("Vias during routing", "Place vias while the board is being routed; off = only between passes, routing everything again", "Routing"),
    "global_selection": ("Global candidate selection", "Phase 1: choose routes for all nets together", "Routing"),
    "regret_order": ("Commit in regret order", "Phase 2: most-constrained nets first", "Routing"),
    "lookahead": ("Look-ahead costs", "Airwire-crossing and barrier costs", "Routing"),
    "demand": ("Demand map", "Reserve scarce channels for nets that need them", "Routing"),
    "ripup": ("Rip-up and reroute", "Phase 3: negotiate congestion", "Routing"),
    "refine": ("Refinement", "Phase 4: shorten each wire once routing is done", "Routing"),
    "candidates": ("Candidates per connection (K)", "Alternatives considered in Phase 1", "Routing"),
    "reroute_candidates": ("Candidates per reroute", "Alternatives considered in rip-up", "Routing"),
    "max_ripup_rounds": ("Rip-up round limit", "Upper bound on Phase 3 rounds", "Routing"),
    "heuristic_weight": ("Search heuristic weight", "1 = exact; above 1 is faster and less exact", "Routing"),
    "trace_width": ("Trace width (mm)", "Width of the default net class", "Rules"),
    "clearance": ("Clearance (mm)", "Copper to copper", "Rules"),
    "via_diameter": ("Via diameter (mm)", "Outer diameter of a via", "Rules"),
    "via_drill": ("Via drill (mm)", "Hole diameter of a via", "Rules"),
    "edge_clearance": ("Copper to board edge (mm)", "A DSN does not carry this rule", "Rules"),
    "margin": ("Safety margin (mm)", "Extra clearance routed with, so nothing sits exactly on a limit", "Rules"),
    "ignore_keepouts": ("Ignore keepout areas", "Drop the DSN's keepouts", "Rules"),
    "show_outlines": ("Component outlines", "Draw the outline of every part", "View"),
    "show_names": ("Component names", "Draw every part's reference", "View"),
    "smooth_corners": ("Round sharp corners", "Replace each sharp corner with the largest arc the design rules allow", "Routing"),
    "teardrops": ("Teardrops", "Add teardrops where traces meet pads and vias", "Teardrops"),
    "teardrops_in_ses": ("Write teardrops to the SES", "As fans of ordinary traces (SES has no filled shapes)", "Teardrops"),
    "teardrop_max_length": ("Longest teardrop (mm)", "Beyond the pad edge", "Teardrops"),
    "teardrop_max_width": ("Widest teardrop (mm)", "Across, at the pad", "Teardrops"),
    "teardrop_breathing": ("Teardrop breathing room", "Clearances kept from foreign copper; cramped teardrops are left out", "Teardrops"),
}


# Settings stored as a number where 0 means "decide for me": name -> (what the
# automatic choice is, the value to start from when it is switched off). An
# editor shows these as a switch plus a number, not as a number with a magic 0.
AUTOMATIC = {
    "workers": ("Use every core", 4),
    "portfolio": ("One per worker, up to 8", 4),
    "trace_width": ("Use the value in the DSN", Rules.trace_width),
    "clearance": ("Use the value in the DSN", Rules.clearance),
    "via_diameter": ("Use the value in the DSN", Rules.via_diameter),
    "via_drill": ("Use the value in the DSN", Rules.via_drill),
    "edge_clearance": ("From the KiCad project beside the DSN, else the clearance", 0.5),
}


@dataclass
class Settings:
    workers: int = 0
    portfolio: int = 0
    seed: int = 0
    max_via_rounds: int = 4
    drc_rounds: int = 4
    use_vias: bool = True
    live_vias: bool = True
    global_selection: bool = True
    regret_order: bool = True
    lookahead: bool = True
    demand: bool = True
    ripup: bool = True
    refine: bool = True
    candidates: int = 6
    reroute_candidates: int = 2
    max_ripup_rounds: int = 100
    heuristic_weight: float = 1.0
    trace_width: float = 0.0
    clearance: float = 0.0
    via_diameter: float = 0.0
    via_drill: float = 0.0
    edge_clearance: float = 0.0
    margin: float = 0.01
    ignore_keepouts: bool = False
    smooth_corners: bool = True
    show_outlines: bool = True
    show_names: bool = True
    teardrops: bool = True
    teardrops_in_ses: bool = True
    teardrop_max_length: float = 1.0
    teardrop_max_width: float = 2.0
    teardrop_breathing: float = 1.5

    # -- file ---------------------------------------------------------------
    @classmethod
    def load(cls, path: str | None = None) -> "Settings":
        """Settings from the file, or the defaults if it is missing or unreadable."""
        path = path or default_path()
        settings = cls()
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return settings
        for field in fields(cls):
            if field.name in data:
                try:
                    setattr(settings, field.name, type(getattr(settings, field.name))(data[field.name]))
                except (TypeError, ValueError):
                    pass  # a bad value in the file: keep the default
        return settings

    def save(self, path: str | None = None) -> str:
        path = path or default_path()
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)
            f.write("\n")
        return path

    # -- what the router takes ----------------------------------------------
    def options(self) -> Options:
        return Options(global_selection=self.global_selection, regret_order=self.regret_order, lookahead=self.lookahead,
                       demand=self.demand, ripup=self.ripup, refine=self.refine, vias=self.use_vias, live_vias=self.live_vias,
                       smooth=self.smooth_corners, teardrops=self.teardrops, teardrop_max_length=self.teardrop_max_length,
                       teardrop_max_width=self.teardrop_max_width, teardrop_breathing=self.teardrop_breathing,
                       portfolio=self.portfolio)

    def cost_overrides(self) -> dict:
        return {"K": self.candidates, "K_reroute": self.reroute_candidates, "max_rounds": self.max_ripup_rounds,
                "h_weight": self.heuristic_weight}

    def apply_rules(self, design, dsn_path: str | None = None) -> tuple[float, float, list[str]]:
        """Applies the rule overrides to ``design.board`` (in place) and adds the
        routing margin. Returns the clearance and edge clearance to *check*
        against (without the margin), and notes for the user."""
        from weaveengine.io.check import kicad_project_rules
        board, rules, notes = design.board, design.board.rules, []
        if self.ignore_keepouts:
            kept = [o for o in board.obstacles if o.net_id >= 0]
            notes.append(f"ignoring {len(board.obstacles) - len(kept)} keepout areas")
            board.obstacles = kept
        if self.trace_width > 0:
            rules.trace_width = self.trace_width
        if self.clearance > 0:
            rules.clearance = self.clearance
        if self.via_diameter > 0:
            rules.via_diameter = self.via_diameter
        if self.via_drill > 0:
            rules.via_drill = self.via_drill
        # Rules the DSN left out and the settings do not set either: say what is used.
        missing = [text for name, text in design.defaulted.items() if getattr(self, name) <= 0]
        if missing:
            notes.append("not in the DSN, defaults used: " + ", ".join(missing))
        if design.dropped_wiring:
            notes.append(design.dropped_wiring)
        if self.edge_clearance > 0:
            rules.edge_clearance = self.edge_clearance
        else:
            project = (dsn_path.rsplit(".", 1)[0] + ".kicad_pro") if dsn_path else ""
            if project and os.path.exists(project):
                rules.edge_clearance = max(kicad_project_rules(project)["edge_clearance"], rules.clearance)
                notes.append(f"edge clearance {rules.edge_clearance:g} mm from {os.path.basename(project)}")
            else:
                notes.append(f"edge clearance {rules.clearance:g} mm (the DSN does not carry one)")
        if board.plane_nets:
            names = design.net_names
            notes.append(f"{len(board.plane_nets)} nets have a copper plane and are not routed: "
                         + ", ".join(sorted(names[n] for n in board.plane_nets)))
        want_clearance = rules.clearance
        want_edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
        rules.clearance = want_clearance + self.margin
        rules.edge_clearance = want_edge + self.margin
        return want_clearance, want_edge, notes


def default_path() -> str:
    """``settings.json`` beside the program: next to ``main.py`` when run from
    source, next to the application (in the folder that holds WeaveEngine.app
    or the executable) when packaged, so the app and its settings travel
    together. Not inside the .app bundle itself: that would break its signature
    and be thrown away by the next build. If that folder cannot be written to
    (an app in /Applications, say), the user's configuration folder is used."""
    if not getattr(sys, "frozen", False):
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "settings.json")
    here = os.path.dirname(os.path.abspath(sys.executable))
    if sys.platform == "darwin" and ".app/Contents/MacOS" in here + "/":
        here = os.path.dirname(here.split(".app/Contents/MacOS")[0])  # the folder containing the .app
    beside = os.path.join(here, "settings.json")
    if os.path.exists(beside) or os.access(here, os.W_OK):
        return beside
    base = (os.path.expanduser("~/Library/Application Support") if sys.platform == "darwin"
            else os.environ.get("APPDATA") or os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config"))
    return os.path.join(base, "WeaveEngine", "settings.json")


def cost_params(settings: Settings, pmap) -> CostParams:
    return CostParams.for_map(pmap, **settings.cost_overrides())
