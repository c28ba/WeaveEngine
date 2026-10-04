"""Cost model parameters (design sections 9 and 20). All costs are millimetre-equivalents."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from weaveengine.topo.planar_map import PlanarMap


@dataclass
class CostParams:
    pres_fac: float = 0.5        # present-congestion weight
    pres_growth: float = 1.5     # multiplied in per rip-up round
    hist_inc: float = 1.0        # history growth per overflow unit
    w_d: float = 0.5             # demand weight
    lambda_x: float = 2.0        # per airwire crossing
    lambda_sever: float = 50.0   # per severed net
    lambda_conf: float = 5.0     # per candidate-pair crossing
    cross_penalty: float = 10.0  # relaxed search, per violated crossing
    K: int = 6                   # candidates per connection (Phase 1)
    K_reroute: int = 2           # candidates per reroute (Phase 2 fallback and Phase 3)
    alpha: float = 0.5           # candidate cost slack over the shortest
    max_rounds: int = 100        # Phase 3 iteration limit
    restarts: int = 3            # ICM random restarts
    h_weight: float = 1.0        # A* heuristic weight; above 1 trades optimality of the estimate for speed
    batch: int = 1               # connections rerouted against one snapshot of the state, in parallel (section 22)

    @classmethod
    def for_map(cls, pmap: PlanarMap, **overrides) -> "CostParams":
        """Section 20 defaults, scaled by the median pad pitch."""
        pitch = median_pad_pitch(pmap)
        params = cls(lambda_x=2 * pitch, lambda_sever=50 * pitch, lambda_conf=5 * pitch, cross_penalty=10 * pitch)
        for k, v in overrides.items():
            if not hasattr(params, k):
                raise TypeError(f"unknown cost parameter {k!r}")
            setattr(params, k, v)
        return params


def median_pad_pitch(pmap: PlanarMap) -> float:
    pts = np.array(list(pmap.pad_centre.values()), dtype=np.float64)
    if len(pts) < 2:
        return 1.0
    dist, _ = cKDTree(pts).query(pts, k=2)
    return float(np.median(dist[:, 1])) or 1.0
