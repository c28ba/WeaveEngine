from toporouter.geom.mesh_builder import build_planar_map_from_points
from toporouter.topo.state import TopoState
from toporouter.topo.router import TopoRouter, Net
from toporouter.viz.svg import export_svg

def main():
    points = [(0.0, 0.0), (2.0, 0.0), (0.5, 2.0), (2.5, 2.0), (1.2, 0.8), (1.8, 1.2)]
    pad_owners = {0: 100, 3: 200}

    print("Building Delaunay PlanarMap mesh...")
    pmap = build_planar_map_from_points(
        points=points,
        pad_owners=pad_owners,
        trace_width=0.15,
        clearance=0.15
    )

    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    start_pos = (0.0, 0.0)
    target_pos = (2.5, 2.0)

    nets = [
        Net(net_id=1, start_pad_id=100, target_pad_id=200, target_pos=target_pos),
        Net(net_id=2, start_pad_id=100, target_pad_id=200, target_pos=target_pos),
    ]

    print("Routing nets on Delaunay topology...")
    if router.route_all(nets):
        print("Routing succeeded! Generating SVG...")
        export_svg(pmap, state, "routed_mesh.svg", start_pos=start_pos, target_pos=target_pos)
        print("Saved generated mesh to routed_mesh.svg")
    else:
        print("Routing failed.")

if __name__ == "__main__":
    main()