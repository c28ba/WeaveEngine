from toporouter.geom.board import SyntheticBoard, Pad, KeepoutZone
from toporouter.topo.state import TopoState
from toporouter.topo.router import TopoRouter, Net
from toporouter.viz.svg import export_svg
from toporouter.topo.analysis import analyze_and_export_routes

def main():
    board = SyntheticBoard(
        width=100.0,
        height=60.0,
        trace_width=1.0,
        clearance=0.8,
        pads=[
            Pad(pad_id=101, x=10.0, y=50.0, net_id=1),
            Pad(pad_id=102, x=90.0, y=10.0, net_id=1),
            Pad(pad_id=201, x=10.0, y=10.0, net_id=2),
            Pad(pad_id=202, x=90.0, y=50.0, net_id=2),
            Pad(pad_id=301, x=10.0, y=30.0, net_id=3),
            Pad(pad_id=302, x=90.0, y=30.0, net_id=3),
        ],
        keepouts=[
            KeepoutZone(vertices=[
                (40.0, 20.0),
                (60.0, 20.0),
                (60.0, 40.0),
                (40.0, 40.0)
            ])
        ]
    )

    pmap, pad_positions = board.build_planar_map()
    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    nets = [
        Net(net_id=1, start_pad_id=101, target_pad_id=102, target_pos=pad_positions[102]),
        Net(net_id=2, start_pad_id=201, target_pad_id=202, target_pos=pad_positions[202]),
        Net(net_id=3, start_pad_id=301, target_pad_id=302, target_pos=pad_positions[302]),
    ]

    success = router.route_all(nets, max_iterations=20)

    nets_info = [
        {
            "net_id": n.net_id,
            "start_pos": pad_positions[n.start_pad_id],
            "target_pos": pad_positions[n.target_pad_id],
        }
        for n in nets
    ]

    analyze_and_export_routes(pmap, state, nets_info, "route_analysis.json")

    keepout_polys = [k.vertices for k in board.keepouts]

    export_svg(
        pmap,
        state,
        "synthetic_board.svg",
        start_pos=pad_positions[101],
        target_pos=pad_positions[102],
        scale=10.0,
        padding=50.0,
        trace_width=board.trace_width,
        clearance=board.clearance,
        keepout_polys=keepout_polys,
    )
    print("Rendered physical board layout with offset clearances to 'synthetic_board.svg'.")

if __name__ == "__main__":
    main()