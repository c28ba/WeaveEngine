import numpy as np
from toporouter.geom.mesh_builder import build_planar_map_from_points
from toporouter.topo.state import TopoState
from toporouter.topo.router import TopoRouter, Net
from toporouter.viz.svg import export_svg

def create_complex_board():
    # 12-point obstacle array simulating component pads and via clusters
    points = [
        # Net 1 Start & Target (Left to Right bottom)
        (0.5, 0.5),   # v0  - Pad 101 (Net 1 Start)
        (4.5, 0.5),   # v1  - Pad 102 (Net 1 Target)
        
        # Net 2 Start & Target (Left to Right top)
        (0.5, 3.5),   # v2  - Pad 201 (Net 2 Start)
        (4.5, 3.5),   # v3  - Pad 202 (Net 2 Target)
        
        # Net 3 Start & Target (Bottom-Center to Top-Center)
        (2.5, 0.2),   # v4  - Pad 301 (Net 3 Start)
        (2.5, 3.8),   # v5  - Pad 302 (Net 3 Target)
        
        # Obstacle Vertices (Middle channel obstacles)
        (1.5, 1.5),   # v6  - Obstacle pin
        (3.5, 1.5),   # v7  - Obstacle pin
        (1.5, 2.5),   # v8  - Obstacle pin
        (3.5, 2.5),   # v9  - Obstacle pin
        (2.5, 2.0),   # v10 - Central obstacle pin
        (2.5, 1.0),   # v11 - Lower channel pin
    ]

    pad_owners = {
        0: 101, 1: 102,
        2: 201, 3: 202,
        4: 301, 5: 302,
    }

    return points, pad_owners

def main():
    points, pad_owners = create_complex_board()

    print("Building 12-vertex Delaunay PlanarMap...")
    pmap = build_planar_map_from_points(
        points=points,
        pad_owners=pad_owners,
        trace_width=0.15,
        clearance=0.15
    )

    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    # Define 3 nets traversing intersecting horizontal and vertical channels
    nets = [
        Net(net_id=1, start_pad_id=101, target_pad_id=102, target_pos=(4.5, 0.5)),
        Net(net_id=2, start_pad_id=201, target_pad_id=202, target_pos=(4.5, 3.5)),
        Net(net_id=3, start_pad_id=301, target_pad_id=302, target_pos=(2.5, 3.8)),
    ]

    print("Executing Rip-Up and Reroute (RRR) on complex board...")
    if router.route_all(nets, max_iterations=15):
        print("Routing succeeded! Exporting complex_mesh.svg...")
        # Export visual mesh using Net 1 start coordinate as baseline origin
        export_svg(pmap, state, "complex_mesh.svg", start_pos=(0.5, 0.5), target_pos=(4.5, 0.5))
        print("Exported to complex_mesh.svg")
    else:
        print("Routing failed due to topology constraints or capacity limits.")

if __name__ == "__main__":
    main()