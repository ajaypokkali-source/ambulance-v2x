# Multi-hop V2V relay - standalone corridor demo (SUMO + TraCI)

    J1 -- J2 -- J3 -- J4 -- J5      1 ambulance, J2/J3/J4 as real traffic lights
    (same 300 m-spaced corridor shape as v2x_integrated)

This is the SIMPLE, isolated version of the multi-hop relay idea: one
ambulance, one corridor, two dedicated "relay_1"/"relay_2" cars placed
ahead of it so a 2-hop relay chain to J2 is reliably demonstrable even
without heavy background traffic. Good for a clean explainer diagram or
a quick console-log demo.

For the BETTER-LOOKING live demo (purple rings + a moving purple line
through the actual relay chain, 10-signal grid, ~530 background cars),
use `ambulance_v2x_city`'s `v2x_city.py --relay` instead - see
`..\ambulance_v2x_city\README.md`. This folder is still useful on its
own as the minimal, easy-to-reason-about version for the report.

## WHAT'S DIFFERENT FROM v2x_integrated

v2x_integrated (and v2x_multi) treat V2I as unlimited-range: "if the
ambulance is within PREEMPTION_DISTANCE, the message arrives." I2I
approximates "infrastructure already knows the ambulance is coming" with
a bigger fixed detection radius once the upstream signal activates - not
a real network mechanism.

This scenario replaces that shortcut with the real thing: V2I_DIRECT_RANGE
is tightened to 70 m (deliberately too short for 300 m junction spacing),
and every step the controller runs a bounded BFS flood (max 3 hops) over
the live vehicle positions, looking for a chain of real cars that can
carry the request to a junction currently out of direct range. If a path
exists, the request is delivered - with the exact relay chain and a
modelled per-hop forwarding delay logged to the console. If no path
exists, the junction just doesn't hear it yet, which is the honest,
demonstrable coverage-gap result this scenario exists to show.

## Setup
    cd C:\ambulance_v2x\v2x_relay
    python build_network.py          # once, builds network.net.xml

## Run
    python test_relay_logic.py       # offline logic check, no SUMO needed
    python v2x_relay_test.py         # live GUI demo - watch the [RELAY] console lines
    python v2x_relay_test.py --no-relay          # direct-range-only baseline, for comparison
    python compare_relay.py          # headless: no-relay vs relay, writes results\comparison.png

## Options (v2x_relay_test.py)
    --nogui            headless
    --no-relay          disable multi-hop relay (direct V2I_DIRECT_RANGE only)
    --seed N            SUMO random seed (for repeatable comparison runs)

## Status
Written and logic-tested offline (`test_relay_logic.py`, 7/7 checks pass:
direct range, 2-hop and 3-hop chains, shortest-path selection, the
--no-relay toggle, and the max-hop cutoff). NOT yet run against real SUMO -
I don't have SUMO available where I built this, so this needs a live run
on your machine to confirm the relay_1/relay_2 spacing actually produces
a clean relay chain to J2 in practice. If it doesn't trigger, retune
their `departPos` in routes.rou.xml (the comments there explain the
spacing math) and re-run.
