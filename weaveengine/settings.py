"""User settings, kept in ``settings.json`` (design section 23).

Every knob a front end exposes lives here, with its default and a one-line
description. The file is plain JSON, so it can be edited by hand; unknown keys
are ignored and missing ones take their defaults.
"""
import json
import os
import re
import sys
from fnmatch import fnmatchcase
from dataclasses import asdict, dataclass, fields

from weaveengine.board import Rules
from weaveengine.plan.context import Options
from weaveengine.topo.costs import CostParams

# The settings come in two parts. "App" is the program itself: how it draws and what it uses of this computer; none
# of it changes the routed result. "Routing" is everything that does. Each part is split by topic (the tabs of the
# editor and the argument groups of ``--help``). topic -> what that tab is about.
SECTIONS = {
    "App": {
        "Display": "How the board is drawn on screen. Nothing here changes the routing or the files written.",
        "Computer": "What the program uses of this machine. It can change how fast routing runs, not what it routes.",
    },
    "Routing": {
        "Rules & nets": "The design rules routing keeps, and which nets it routes. Each rule left on automatic is read from the board.",
        "Layers": "Which layers carry traces, and which are dearer to use.",
        "Margin & edges": "How close copper may come to the board edge, and what counts as the edge.",
        "Effort & search": "How hard and how long the router tries, and how it searches. More effort finds more routes but takes longer; "
                           "the defaults suit most boards.",
        "Traces & shape": "How the traces are drawn once routed: corners, widths, angles and detours.",
        "Vias": "Whether and how vias are used to change layer.",
        "Pours & planes": "Copper pours (zones) and planes: joining pads and pieces they do not reach, and keeping them whole under the traces.",
        "Teardrops": "The tapered fillets where a trace meets a pad or a via.",
        "Export": "What is written to the output files.",
    },
}
TOPICS = [topic for topics in SECTIONS.values() for topic in topics]


def section_of(topic: str) -> str:
    """The part ("App" or "Routing") that holds ``topic``."""
    return next(part for part, topics in SECTIONS.items() if topic in topics)


# name -> (label, description, topic). In the order the editor lists them.
DESCRIPTIONS = {
    # -- App ------------------------------------------------------------------
    "show_outlines": ("Component outlines", "Draw the outline of every part. Turn off for a clearer view of the copper on a crowded board. "
                      "Display only.", "Display"),
    "show_names": ("Component names", "Draw every part's reference (U1, R12). Turn off to declutter a dense board. Display only.", "Display"),
    "show_pours": ("Show pours", "Draw the estimated fill of each copper pour once the board is routed, so you can see where the "
                   "copper lies. It is WeaveEngine's estimate, not KiCad's own fill. Display only.", "Display"),
    "show_pour_islands": ("Show islands", "Draw the islands of a pour: pieces joined to nothing, left in by their zone (KiCad reports "
                          "them as isolated copper; a zone set to remove islands has none). Useful for seeing what 'Join pour islands "
                          "that have no pads' would act on. Display only.", "Display"),
    "gpu_drawing": ("Draw with the graphics card", "Much faster panning and zooming on large boards. If this machine cannot do it, or it "
                    "once failed, the ordinary drawing is used instead. Off by default because some graphics drivers misbehave.", "Display"),
    "workers": ("Worker processes", "How many processor cores routing may use. Every core is fastest; use fewer to keep the machine "
                "responsive while you route. Racing variants (Effort & search) by default races one per worker.", "Computer"),
    # -- Routing: Rules & nets ------------------------------------------------
    "trace_width": ("Trace width (mm)", "Width of the default net class. Leave on automatic to take it from the board; set it only "
                    "to route with something other than the board's own rule.", "Rules & nets"),
    "clearance": ("Clearance (mm)", "The gap kept between copper of different nets. Automatic takes the board's rule. A larger value "
                  "routes with more room and so leaves more connections open; a smaller one than the board's will fail KiCad's check.", "Rules & nets"),
    "via_diameter": ("Via diameter (mm)", "Outer diameter of a via. Automatic takes the board's own. Larger vias need more room and "
                     "route less easily.", "Rules & nets"),
    "via_drill": ("Via drill (mm)", "Hole diameter of a via. Automatic takes the board's own. The ring of copper left around the hole "
                  "is the via diameter less this.", "Rules & nets"),
    "margin": ("Safety margin (mm)", "Extra clearance routed with, so that nothing sits exactly on a limit and the board passes KiCad's "
              "check despite rounding. Raise it for more slack at the cost of room; the checks themselves still use the real rules.", "Rules & nets"),
    "ignore_keepouts": ("Ignore keepout areas", "Drop the keepout areas the board file carries, so traces and vias may cross them. Leave "
                        "off unless the keepouts are wrong or you mean to fix them afterwards: crossing them breaks the design's own rules.", "Rules & nets"),
    "skip_nets": ("Nets not to route", "Names separated by commas; * and ? match any text (GND*, say). Treated as a net with a pour "
                  "already: not routed, and not counted as open. Use for nets you will draw by hand.", "Rules & nets"),
    # -- Routing: Layers ------------------------------------------------------
    "layers": ("Layers to route", "Names of the signal layers that may carry traces, separated by commas (F.Cu, B.Cu, say). Empty: every "
               "layer the board file has. The layers left out still keep vias clear of their copper and reach their planes. Applies "
               "the next time the board is routed.", "Layers"),
    "layer_cost": ("Layers that cost more", "A layer and how many times its length a trace on it counts for, as LAYER:FACTOR, separated "
                   "by commas (In1.Cu:5, say). Traces then use that layer only where they must: for a layer that also carries a "
                   "plane, say. The factor is 1 or more. Empty: every layer costs the same.", "Layers"),
    # -- Routing: Margin & edges ----------------------------------------------
    "edge_clearance": ("Copper to board edge (mm)", "How far copper keeps from the board outline. A DSN file does not carry this rule, so "
                       "automatic reads it from the KiCad project beside the file, else uses the clearance. Set it if your fabricator "
                       "needs more.", "Margin & edges"),
    "margin_drawings": ("Drawings on the Margin layer", "Technical. KiCad's check holds copper an edge clearance away from anything "
                        "drawn on the Margin layer, as if it were the board edge, and the DSN does not tell WeaveEngine about it. "
                        "'Added' (the default) keeps clear of those drawings the copper WeaveEngine adds after routing (teardrops, rounded "
                        "corners, tapers, corner blunting, arcs, plane vias and trace ends) and changes no routing. 'Edge' treats the "
                        "drawings as a real board edge for traces and vias too, and a pad inside the band is left only straight away from "
                        "the drawing: the fewest KiCad edge errors, but on a board whose Margin drawings are shield outlines crossed by "
                        "the design's own traces it can leave many connections unrouted. 'Off' ignores them.", "Margin & edges"),
    # -- Routing: Effort & search ---------------------------------------------
    "portfolio": ("Variants raced per pass", "Each pass is tried several ways at once and the best result is kept. More variants find "
                  "better routings but use more cores and memory; off gives one try per pass.", "Effort & search"),
    "seed": ("Random seed", "Same seed and settings give the same result. Change it to get a different routing of the same board, "
             "which sometimes completes where another does not.", "Effort & search"),
    "time_limit": ("Time limit (minutes)", "When it is up, the routing is not bettered further: the best found so far is finished and "
                   "shown (that takes a little longer still). Leave unlimited for the best result.", "Effort & search"),
    "drc_rounds": ("Design-rule repair rounds", "Reroute attempts for traces that fail the final design-rule check. More rounds fix more "
                   "violations but take longer; none leaves them for you.", "Effort & search"),
    "global_selection": ("Global candidate selection", "Phase 1: choose routes for all nets together rather than one at a time, so the "
                         "nets share the room well. Turning it off is faster and usually completes less.", "Effort & search"),
    "regret_order": ("Commit in regret order", "Phase 2: route the most constrained nets first, since they have the fewest choices. "
                     "Off routes shortest first.", "Effort & search"),
    "lookahead": ("Look-ahead costs", "Make the search aware of where other nets' airwires will have to cross, and avoid blocking them. "
                  "Better completion, slightly slower.", "Effort & search"),
    "demand": ("Demand map", "Reserve narrow channels for the nets that need them, so earlier nets do not use them up. Better completion "
               "on dense boards.", "Effort & search"),
    "ripup": ("Rip-up and reroute", "Phase 3: where nets are in each other's way, remove some and route them again until the congestion "
              "is settled. The step that completes hard boards; off is much faster and leaves more open.", "Effort & search"),
    "refine": ("Refinement", "Phase 4: once routing is done, reroute each wire once more to shorten it. Shorter copper, a little more "
               "time.", "Effort & search"),
    "candidates": ("Candidates per connection (K)", "Alternative routes considered for each connection in phase 1. More gives the "
                   "global choice more to pick from at the cost of time and memory.", "Effort & search"),
    "reroute_candidates": ("Candidates per reroute", "Alternative routes considered when a connection is rerouted in rip-up. More "
                           "finds ways around congestion, more slowly.", "Effort & search"),
    "max_ripup_rounds": ("Rip-up round limit", "Upper bound on the rounds of phase 3. Fewer finish sooner with more left open; a board "
                         "that is going to complete usually does so well within the default.", "Effort & search"),
    "heuristic_weight": ("Search heuristic weight", "1 is exact. Above 1 the search is faster but less exact, so traces may be a little "
                         "longer. Only worth raising for very large boards.", "Effort & search"),
    # -- Routing: Traces & shape ----------------------------------------------
    "untangle": ("Undo long detours", "Move the traces that force a connection the long way round, where that leaves less copper. "
                 "Shorter, tidier routing; turn off to keep the first routing found.", "Traces & shape"),
    "narrow_to_reach": ("Narrow traces to reach fine-pitch pins", "Where a net's pads cannot be reached at its class's width, route it "
                        "narrower, down to the board's minimum track width (from the KiCad project). Off keeps every net at its class "
                        "width, and leaves the pins it cannot reach open.", "Traces & shape"),
    "taper_to_pins": ("Taper traces to fine-pitch pins", "A net narrowed to reach fine-pitch pins is drawn at its own width again, "
                      "narrowing (with a wedge) only over the last stretch to each such pin, where there is room. Off: narrow all the "
                      "way. Tapering keeps the resistance down on the stretch that does not need to be thin.", "Traces & shape"),
    "smooth_corners": ("Round sharp corners", "Replace each sharp corner with the largest arc the design rules allow. Smoother copper "
                       "and nicer to etch; the arcs are written as arcs or as short segments (see Export).", "Traces & shape"),
    "least_angle": ("Least angle where a net meets itself (degrees)", "Where a trace meets its pad, or traces of one net meet at a via, "
                    "a pad or a branch, at a sharper angle than this, the copper makes an acid trap (a wedge that holds etchant). It "
                    "is redrawn where there is room: square into the pad, or along the other trace before it branches off. At most "
                    "90, the default, redraws any meeting sharper than square; lower leaves meetings above that angle as drawn; "
                    "zero turns it off.", "Traces & shape"),
    # -- Routing: Vias --------------------------------------------------------
    "use_vias": ("Use vias", "Let a connection change layer through vias where that is the cheaper way. Off keeps every connection on "
                 "one layer (for a single-sided board, say); many will then stay open.", "Vias"),
    "slide_vias": ("Straighten vias", "Move each via to where its traces run through it with less of a bend. Neater routing; off "
                   "leaves vias where the search put them.", "Vias"),
    # -- Routing: Pours & planes ----------------------------------------------
    "fanout_to_planes": ("Join pads to planes", "A pad of a poured net (GND, say) that its pour does not reach, or a piece of the pour "
                         "the traces cut off, is joined to the rest of the net once the board is routed, by the cheapest way that the "
                         "estimated fill finds joins it: a via in or beside the pad or in the piece, a trace into the fill, or a "
                         "jumper (a via pair and a trace on another layer). Off leaves those pads and pieces for the pour or for "
                         "you, and the settings below then do nothing.", "Pours & planes"),
    "pour_reliefs": ("Change thermal reliefs to join pads", "A pad whose thermal spokes do not reach its pour may be joined solid, or "
                     "its spokes turned, where that joins it and costs least. Off by default because it changes how the pad is "
                     "connected, which can affect soldering. Not the zone's rule: written as the pad's own in the routed copy of the "
                     "board (the SES cannot carry it).", "Pours & planes"),
    "pour_narrow_reliefs": ("Narrow a pad's relief to join it", "A pad cut off from its pour may have its relief gap narrowed to the "
                            "zone's clearance and its spokes to the zone's minimum width, where that joins it and costs least. Off by "
                            "default because it changes the pad's own thermal rule. Written as the pad's own in the routed copy of "
                            "the board.", "Pours & planes"),
    "join_bare_islands": ("Join pour islands that have no pads", "A piece of a pour that holds no pad and that its zone does not "
                          "remove (KiCad reports it as isolated copper) is joined to the rest of its net, by a via in it or a trace "
                          "from it, so that the pour is whole under the traces. Off by default because it adds copper that nothing "
                          "needs; the piece is harmless, but KiCad flags it. Off: such pieces are left as they are.", "Pours & planes"),
    "pour_spokes": ("Add spokes to starved thermal pads", "A pad joined to its pour by fewer thermal spokes than the board asks (KiCad's "
                    "starved thermal) gets traces from it into the fill, each a spoke KiCad counts. Clears those KiCad warnings; off "
                    "leaves the pads as they are.", "Pours & planes"),
    "pour_skeletons": ("Keep pours whole", "Once the traces are routed, a thin path through each pour joining its pads, which traces "
                       "move off where they can, so that no trace cuts the pour into pieces. A trace is never given up for it. Off "
                       "lets traces cut a pour freely, which leaves more pieces to join.", "Pours & planes"),
    "pour_min_width": ("Pour minimum width (mm)", "The narrowest copper a pour's fill may have, which the estimate of the fill and the "
                       "pours' skeletons are drawn to. Automatic takes each zone's own minimum thickness from the board (0.25 mm where "
                       "there is none); set it to "
                       "estimate with something else.", "Pours & planes"),
    # -- Routing: Teardrops ---------------------------------------------------
    "teardrop_style": ("Teardrops", "Tapered fillets where traces meet pads and vias, which strengthen the joint. 'Tracks' writes them "
                       "into the SES as short tracks, 'zones' as KiCad teardrop zones in a routed copy of the board, 'none' leaves "
                       "them out. Can be changed after routing: only the teardrops are made again.", "Teardrops"),
    "teardrop_max_length": ("Longest teardrop (mm)", "How far a teardrop reaches along the trace beyond the pad's edge. Longer is "
                            "gentler but needs more room.", "Teardrops"),
    "teardrop_max_width": ("Widest teardrop (mm)", "How wide a teardrop may be across at the pad.", "Teardrops"),
    "teardrop_breathing": ("Teardrop breathing room", "Clearances kept from foreign copper; a teardrop that would be closer is left "
                           "out, so a cramped one never makes a clearance fault. Raise it for fewer, safer teardrops.", "Teardrops"),
    # -- Routing: Export ------------------------------------------------------
    "routed_board": ("Routed copy of the board", "Also write the routing straight into a copy of the KiCad board (.kicad_pcb) beside "
                     "the SES, at this path. Empty: NAME-routed.kicad_pcb, written only when teardrop zones or changed reliefs need "
                     "it. The original board is never changed.", "Export"),
    "arcs_in_ses": ("Write arcs to the SES", "Write curves as arcs (qarc) instead of many short segments. Arcs are smaller and smoother; "
                    "turn off if another tool reading the SES does not take arcs.", "Export"),
}

# A setting that is one of a few choices: name -> {value: label}
CHOICES = {
    "margin_drawings": {
        "added": "Added copper keeps clear (default)",
        "edge": "A board edge for everything",
        "off": "Ignored",
    },
    "teardrop_style": {
        "tracks": "Tracks in the SES",
        "zones": "KiCad zones in a routed copy",
        "none": "None (KiCad can add its own)",
    },
}

# name -> words and synonyms a designer might type (the search also reads the label, description and name)
KEYWORDS: dict[str, tuple[str, ...]] = {
    "workers": ("cpu", "cores", "threads", "parallel", "processes", "multiprocessing", "performance"),
    "portfolio": ("race", "variants", "parallel", "strategies", "attempts", "best of", "speed"),
    "seed": ("random", "repeatable", "deterministic", "reproducible", "rng"),
    "time_limit": ("timeout", "deadline", "duration", "stop", "minutes", "maximum time", "cutoff"),
    "drc_rounds": ("drc", "repair", "fix", "violations", "errors", "check", "retry", "design rule check"),
    "use_vias": ("via", "layer change", "through hole", "layers", "plated"),
    "global_selection": ("phase 1", "global", "candidates", "route choice", "planning"),
    "regret_order": ("phase 2", "ordering", "priority", "constrained", "sequence", "commit"),
    "lookahead": ("cost", "airwire", "ratsnest", "crossing", "barrier", "predict"),
    "demand": ("congestion", "channel", "reserve", "capacity", "scarce", "map"),
    "ripup": ("rip up", "ripup", "reroute", "negotiate", "congestion", "phase 3", "rip-up"),
    "refine": ("phase 4", "shorten", "optimise", "optimize", "cleanup", "polish", "tidy", "length"),
    "candidates": ("k", "alternatives", "options", "routes", "phase 1", "paths"),
    "reroute_candidates": ("alternatives", "options", "ripup", "rip up", "retry", "paths"),
    "max_ripup_rounds": ("iterations", "limit", "maximum", "rip up", "phase 3", "loops", "passes"),
    "heuristic_weight": ("astar", "a star", "search", "speed", "quality", "greedy", "exact", "weighted"),
    "trace_width": ("track", "track width", "wire", "net class", "netclass", "thickness", "line width", "default"),
    "clearance": ("spacing", "gap", "isolation", "distance", "copper", "net class", "design rule"),
    "via_diameter": ("via size", "via pad", "annular ring", "size", "outer", "pad size"),
    "via_drill": ("via size", "hole", "drill size", "hole size", "bore", "plated"),
    "layers": ("layer", "layers", "copper layers", "signal", "single sided", "top", "bottom", "inner", "f.cu", "b.cu", "restrict", "only"),
    "layer_cost": ("layer", "prefer", "avoid", "penalty", "weight", "plane layer", "inner", "cost", "factor", "dearer"),
    "edge_clearance": ("board edge", "outline", "border", "keepout", "edge cut", "edge cuts", "margin", "boundary"),
    "margin": ("slack", "tolerance", "safety", "buffer", "extra", "clearance", "headroom"),
    "margin_drawings": ("margin", "edge", "board edge", "edge clearance", "outline", "border", "kicad", "copper_edge_clearance",
                       "drawing", "shield", "keepout"),
    "ignore_keepouts": ("keep out", "keepout", "restricted", "forbidden", "zone", "no route", "dsn", "area"),
    "show_outlines": ("courtyard", "footprint", "body", "silkscreen", "display", "draw", "parts", "components"),
    "show_names": ("reference", "designator", "refdes", "labels", "text", "display", "parts", "components"),
    "gpu_drawing": ("opengl", "graphics", "acceleration", "hardware", "render", "performance", "fast", "video card"),
    "skip_nets": ("ignore", "exclude", "gnd", "ground", "power", "pattern", "wildcard", "glob", "net names", "unrouted"),
    "untangle": ("detour", "wiggle", "shorten", "straighten", "loop", "long way", "tidy", "clean", "length"),
    "narrow_to_reach": ("taper", "neck", "neckdown", "narrow", "thin", "width", "fine pitch", "pin", "pad", "bga", "qfn", "minimum track"),
    "fanout_to_planes": ("fan out", "fanout", "plane", "ground", "gnd", "power", "pour", "zone", "via", "stitch", "decoupling"),
    "pour_reliefs": ("thermal", "relief", "spoke", "solid", "zone", "connection", "starved", "pad", "override", "kicad"),
    "pour_narrow_reliefs": ("thermal", "relief", "gap", "spoke", "width", "clearance", "narrow", "starved", "pad", "override"),
    "join_bare_islands": ("island", "isolated", "copper", "pour", "zone", "plane", "stitch", "via", "whole", "return path", "fill"),
    "pour_spokes": ("thermal", "spoke", "starved", "relief", "pad", "pour", "zone", "drc", "connection", "trace"),
    "pour_skeletons": ("zone", "plane", "copper fill", "fill", "connectivity", "split", "cut", "thermal", "spoke", "whole"),
    "pour_min_width": ("zone", "fill", "minimum thickness", "min width", "sliver", "copper fill", "thin", "narrowest"),
    "show_pours": ("zone", "plane", "copper fill", "fill", "display", "draw", "ground plane", "polygon"),
    "show_pour_islands": ("zone", "orphan", "dead copper", "isolated", "unconnected", "fill", "display", "remnant"),
    "slide_vias": ("move", "straighten", "bend", "shift", "adjust", "via", "optimise", "optimize", "kink"),
    "taper_to_pins": ("taper", "neck", "neckdown", "narrow", "thin", "width", "fine pitch", "pin", "pad", "wedge"),
    "least_angle": ("acute", "acid trap", "angle", "sharp", "junction", "branch", "tee", "pad entry", "etch", "dfm", "wedge"),
    "smooth_corners": ("round", "arc", "curve", "bend", "angle", "45", "90", "chamfer", "radius", "fillet", "acute"),
    "teardrop_style": ("teardrop", "fillet", "pad entry", "tear drop", "reinforce", "annular ring", "junction", "via", "kicad",
                       "zone", "session", "export", "fan", "tracks", "none", "off"),
    "routed_board": ("kicad", "kicad_pcb", "board", "copy", "export", "output", "save", "pcb", "file", "teardrop", "zone"),
    "arcs_in_ses": ("curve", "arc", "qarc", "session", "export", "output", "freerouting", "segments", "round"),
    "teardrop_max_length": ("teardrop", "fillet", "size", "longest", "extent", "limit", "pad entry"),
    "teardrop_max_width": ("teardrop", "fillet", "size", "widest", "limit", "pad entry"),
    "teardrop_breathing": ("teardrop", "fillet", "clearance", "spacing", "room", "space", "cramped", "foreign copper", "pad entry"),
}


def edit_distance_at_most_one(a: str, b: str) -> bool:
    """True when ``a`` becomes ``b`` by one insert, delete, substitution or swap of neighbours."""
    if a == b:
        return True
    if abs(len(a) - len(b)) > 1:
        return False
    i = 0
    while i < min(len(a), len(b)) and a[i] == b[i]:
        i += 1
    if len(a) == len(b):
        return a[i + 1:] == b[i + 1:] or (a[i:i + 2] == b[i:i + 2][::-1] and a[i + 2:] == b[i + 2:])
    short, long = sorted((a, b), key=len)
    return short[i:] == long[i + 1:]


def word_matches(query_word: str, word: str) -> bool:
    """A word starts with the query word; for 4+ letters one typo is allowed (in the start of the word too)."""
    if word.startswith(query_word):
        return True
    n = len(query_word)
    return n >= 4 and any(edit_distance_at_most_one(query_word, word[:k]) for k in (n - 1, n, n + 1))


def matches_search(query: str, name: str) -> bool:
    """Whether setting ``name`` fits every word of ``query``, in any order, by label, description, keywords or field name."""
    label, description, _ = DESCRIPTIONS[name]
    text = " ".join((label, description, *KEYWORDS[name], name.replace("_", " "))).lower()
    words = re.findall(r"[^\W_]+", text)
    return all(any(word_matches(q, w) for w in words) for q in query.lower().split())


# Settings stored as a number where 0 means "decide for me": name -> (what the
# automatic choice is, the value to start from when it is switched off). An
# editor shows these as a switch plus a number, not as a number with a magic 0.
AUTOMATIC = {
    "workers": ("Use every core", 4),
    "portfolio": ("One per worker, up to 8", 4),
    "time_limit": ("No limit", 30.0),
    "trace_width": ("Use the value in the DSN", Rules.trace_width),
    "clearance": ("Use the value in the DSN", Rules.clearance),
    "via_diameter": ("Use the value in the DSN", Rules.via_diameter),
    "via_drill": ("Use the value in the DSN", Rules.via_drill),
    "edge_clearance": ("From the KiCad project", 0.5),   # (else the clearance: the description says so)
    "pour_min_width": ("From the board's zones", 0.25),  # (else 0.25 mm: the description says so)
}


@dataclass
class Settings:
    workers: int = 0
    portfolio: int = 0
    seed: int = 0
    time_limit: float = 0.0  # minutes; 0 = none
    drc_rounds: int = 4
    use_vias: bool = True
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
    margin_drawings: str = "added"  # one of CHOICES["margin_drawings"]
    slide_vias: bool = True
    untangle: bool = True
    narrow_to_reach: bool = True
    taper_to_pins: bool = True
    fanout_to_planes: bool = True
    pour_reliefs: bool = False
    pour_narrow_reliefs: bool = False
    join_bare_islands: bool = False
    pour_spokes: bool = True
    pour_skeletons: bool = True
    pour_min_width: float = 0.0  # mm; 0 = from the board's zones
    show_pours: bool = True
    show_pour_islands: bool = False
    skip_nets: str = ""  # comma-separated net names or patterns
    layers: str = ""  # comma-separated signal layers to route; "" = all
    layer_cost: str = ""  # comma-separated LAYER:FACTOR; "" = every layer costs the same
    smooth_corners: bool = True
    least_angle: float = 90.0  # degrees; 0 = traces are not redrawn for acute meetings
    show_outlines: bool = True
    show_names: bool = True
    gpu_drawing: bool = False
    teardrop_style: str = "tracks"  # one of CHOICES["teardrop_style"]
    routed_board: str = ""          # where the routed copy of the board goes ("": NAME-routed.kicad_pcb, for teardrop zones only)
    arcs_in_ses: bool = True
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
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return settings
        for field in fields(cls):
            if field.name in data:
                try:
                    setattr(settings, field.name, type(getattr(settings, field.name))(data[field.name]))
                except (TypeError, ValueError):
                    pass  # a bad value in the file: keep the default
        if "teardrop_style" not in data and False in (data.get("teardrops"), data.get("teardrops_in_ses")):
            settings.teardrop_style = "none"  # (as a settings file from before 1.1 says it)
        for name, choices in CHOICES.items():
            if getattr(settings, name) not in choices:
                setattr(settings, name, getattr(cls(), name))
        return settings

    def save(self, path: str | None = None) -> str:
        path = path or default_path()
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2)
            f.write("\n")
        return path

    # -- what the router takes ----------------------------------------------
    def options(self) -> Options:
        return Options(global_selection=self.global_selection, regret_order=self.regret_order, lookahead=self.lookahead,
                       demand=self.demand, ripup=self.ripup, refine=self.refine, vias=self.use_vias,
                       slide=self.slide_vias, smooth=self.smooth_corners, untangle=self.untangle, narrow=self.narrow_to_reach, taper=self.taper_to_pins, fanout=self.fanout_to_planes, reliefs=self.pour_reliefs, narrow_reliefs=self.pour_narrow_reliefs, bare_islands=self.join_bare_islands, spokes=self.pour_spokes, skeleton=self.pour_skeletons, teardrops=self.teardrop_style != "none", teardrop_max_length=self.teardrop_max_length,
                       teardrop_max_width=self.teardrop_max_width, teardrop_breathing=self.teardrop_breathing,
                       least_angle=min(max(self.least_angle, 0.0), 90.0), portfolio=self.portfolio)

    def cost_overrides(self) -> dict:
        return {"K": self.candidates, "K_reroute": self.reroute_candidates, "max_rounds": self.max_ripup_rounds,
                "h_weight": self.heuristic_weight}

    def apply_rules(self, design, dsn_path: str | None = None) -> tuple[float, float, list[str]]:
        """Applies the rule overrides to ``design.board`` (in place) and adds the
        routing margin. Returns the clearance and edge clearance to *check*
        against (without the margin), and notes for the user."""
        board, rules, notes = design.board, design.board.rules, []
        found = detected_rules(design, dsn_path)
        if self.ignore_keepouts:
            kept = [o for o in board.obstacles if o.net_id >= 0]
            notes.append(f"ignoring {len(board.obstacles) - len(kept) + len(board.no_via)} keepout areas")
            board.obstacles, board.no_via = kept, []
        if self.trace_width > 0:
            rules.trace_width = self.trace_width
        if self.clearance > 0:
            rules.clearance = self.clearance
        if self.via_diameter > 0:
            rules.via_diameter = self.via_diameter
        if self.via_drill > 0:
            rules.via_drill = self.via_drill
        # Rules the board does not give and the settings do not set either: say what is used instead.
        notes += [f"WARNING: {text}" for text in undetected(found, self)]
        if design.dropped_wiring:
            notes.append(design.dropped_wiring)
        notes += [f"WARNING: {text}" for text in design.warnings]
        if design.kicad:
            notes.append(design.kicad)
        if design.walled:
            from weaveengine.io.dsn import walled_note
            notes.append(walled_note(design.walled))
        board.pour_override = self.pour_min_width
        board.margin_edge = self.margin_drawings
        if self.pour_min_width > 0:
            notes.append(f"pour minimum width {self.pour_min_width:g} mm, set here (the zones' own are not used)")
        if self.edge_clearance > 0:
            rules.edge_clearance = self.edge_clearance
        elif found["edge_clearance"][0] is not None:
            rules.edge_clearance = max(found["edge_clearance"][0], rules.clearance, rules.edge_clearance or 0.0)
            notes.append(f"edge clearance {rules.edge_clearance:g} mm, {found['edge_clearance'][1]}")
        if board.plane_nets:
            names = design.net_names
            notes.append(f"{len(board.plane_nets)} nets have a copper plane and are not routed: "
                         + ", ".join(sorted(names[n] for n in board.plane_nets)))
        note = skip_nets(design, self.skip_nets.split(","))
        if note:
            notes.append(note)
        want_clearance = rules.clearance
        want_edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
        rules.clearance = want_clearance + self.margin
        rules.edge_clearance = want_edge + self.margin
        rules.margin = self.margin
        return want_clearance, want_edge, notes


def layer_list(text: str) -> list[str] | None:
    """The layers named in the ``layers`` setting, or None for all of them."""
    return [name.strip() for name in text.split(",") if name.strip()] or None


def layer_costs(design, items) -> dict[int, float]:
    """Layer index -> factor for the ``LAYER:FACTOR`` items (the ``layer_cost`` setting split at commas, or ``--layer-cost``);
    only factors above 1. Raises ValueError, with a message to show, for a layer not routed or a factor below 1."""
    out = {}
    for item in items:
        if not item.strip():
            continue
        name, _, factor = item.strip().rpartition(":")
        try:
            factor = float(factor)
        except ValueError:
            factor = 0.0
        if name not in design.board.layers or not 1.0 <= factor < float("inf"):
            raise ValueError(f"layer cost {item.strip()}: a layer routed ({', '.join(design.board.layers)}) and a factor of 1 or more")
        if factor > 1.0:
            out[design.board.layers.index(name)] = factor
    return out


def skip_nets(design, patterns) -> str:
    """Marks the nets whose names match any of ``patterns`` (globs) as not to be routed.
    Returns the line to tell the user, or "" if none matched."""
    patterns = [p.strip() for p in patterns if p.strip()]
    hit = {n for n, name in design.net_names.items() if any(fnmatchcase(name, p) for p in patterns)}
    design.board.skip_nets = hit
    if not hit:
        return ""
    return (f"{len(hit)} nets are left unrouted by choice and not counted as open: "
            + ", ".join(sorted(design.net_names[n] for n in hit)))


RULES = ("trace_width", "clearance", "via_diameter", "via_drill", "edge_clearance")  # the rules settings can override


def kicad_project(dsn_path: str | None) -> str | None:
    """The KiCad project file of the same name beside the DSN, if there is one."""
    project = (dsn_path.rsplit(".", 1)[0] + ".kicad_pro") if dsn_path else ""
    return project if project and os.path.exists(project) else None


def detected_rules(design, dsn_path: str | None) -> dict[str, tuple[float | None, str]]:
    """What the board itself says for each rule in ``RULES``, before any setting
    overrides it: name -> (value in mm, where it came from), or (None, what is
    used instead) when the board does not give it. Read ``design`` as
    ``read_dsn`` returned it. The edge clearance is in no DSN; it comes from
    the KiCad project beside the file, else the clearance stands in for it."""
    rules, out = design.board.rules, {}
    for name in RULES[:-1]:
        value = getattr(rules, name)
        out[name] = (None, f"the built-in {value:g} mm is used") if name in design.defaulted else (value, "from the DSN")
    project = kicad_project(dsn_path)
    edge = 0.0
    if project:
        from weaveengine.io.check import kicad_project_rules
        try:
            edge = kicad_project_rules(project)["edge_clearance"]
        except (OSError, ValueError, KeyError, TypeError):
            edge = 0.0
    if edge > 0:
        out["edge_clearance"] = (edge, f"from {os.path.basename(project)}")
    else:
        stem = os.path.basename((dsn_path or "board.dsn").rsplit(".", 1)[0])
        where = f"{os.path.basename(project)} sets none" if project else f"there is no {stem}.kicad_pro beside the DSN"
        out["edge_clearance"] = (None, f"a DSN does not carry it and {where}; the clearance is used ({rules.clearance:g} mm unless set)")
    return out


def undetected(found: dict, settings: "Settings | None" = None) -> list[str]:
    """One line per rule the board does not give and ``settings`` (if given) do not set either."""
    labels = {"trace_width": "trace width", "clearance": "clearance", "via_diameter": "via diameter",
              "via_drill": "via drill", "edge_clearance": "copper to board edge clearance"}
    return [f"{labels[name]} not detected: {text}" for name, (value, text) in found.items()
            if value is None and (settings is None or getattr(settings, name) <= 0)]


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


def cost_params(settings: Settings, pmap, layer_cost: dict | None = None) -> CostParams:
    """The cost parameters for ``pmap``; ``layer_cost``: layer index -> factor (``layer_costs``)."""
    overrides = settings.cost_overrides()
    if layer_cost:
        overrides["layer_cost"] = layer_cost
    return CostParams.for_map(pmap, **overrides)
