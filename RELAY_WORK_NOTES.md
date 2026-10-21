# Multi-hop V2V relay - status notes (2026-10-21)

Added for the "needs to look like enough work for a 4-person team" ask:
a new networking mechanism (real multi-hop store-and-forward relay)
layered on top of the existing V2I/I2I preemption, in TWO places.

## 1. `ambulance_v2x_city\v2x_city.py --relay`  <- use this one for the live demo

Opt-in flag on the existing 10-signal grid scenario. OFF by default, so
every existing command (`--baseline`, `--no-v2v`, `./run.sh`,
`compare.py`, `results\comparison.png`) is **completely unchanged** -
confirmed by re-reading the whole diff before sending it over.

What it does: this grid has 300 m signal spacing but only a 100 m direct
V2I radius, so every signal with no upstream I2I neighbour (every FIRST
signal on a route) has a real ~200 m blind spot. `--relay` closes it: a
bounded BFS flood (max 3 hops) over the live vehicle positions looks for
a chain of nearby cars that can carry an ambulance's beacon to an
out-of-range signal. With ~530 background cars already in this scenario,
relay chains should form organically, no hand-placed vehicles needed.

Visuals (GUI only): a signal's ring turns **purple** while a request is
waiting on a relay chain (new ring state, alongside the existing
orange/yellow/green), and a live **purple line** is drawn through the
actual relaying vehicles' positions from the ambulance to the signal,
updating every step. Console/board events are tagged `[RELAY]`.

New files in that folder: `compare_relay.py` (full V2X vs full V2X +
relay, writes `results\comparison_relay.png`/`.csv` - separate from
`compare.py`, never touches the existing `comparison.png`),
`test_relay_logic.py` (offline unit test of the relay BFS, no SUMO
needed - 5/5 pass as shipped).

Try it:
    cd C:\ambulance_v2x\ambulance_v2x_city
    python test_relay_logic.py
    python v2x_city.py --relay
    python compare_relay.py

## 2. `v2x_relay\`  <- the simple, isolated version

A standalone 5-junction corridor (same shape as `v2x_integrated`), with
the same relay mechanism but no city-grid visuals - two dedicated
"relay_1"/"relay_2" cars guarantee a demonstrable 2-hop chain to J2.
Good for a clean explainer diagram in the report, or if the city grid is
too busy to read on a projector. See `v2x_relay\README.md`.

Try it:
    cd C:\ambulance_v2x\v2x_relay
    python build_network.py
    python test_relay_logic.py
    python v2x_relay_test.py
    python compare_relay.py

## Status / what's NOT done yet

I don't have SUMO available in the environment I built this in, so
**neither has been run against real SUMO yet** - only logic-tested
offline (the BFS pathfinding itself, in isolation). Everything else
(network files, traffic, GUI code) is either copied unchanged from your
proven scenarios or reuses `ambulance_v2x_city`'s existing, already-
working plumbing, so it should be solid - but it needs one real run on
your machine to confirm:

1. `ambulance_v2x_city --relay`: does a purple ring/line actually appear
   during a live run? (Should, given the background traffic density -
   if not, check the `[RELAY]`/`relay_no_path` counts in the console
   summary at the end.)
2. `v2x_relay`: does the hand-placed relay_1/relay_2 chain actually
   reach J2 before the ambulance gets there on its own? If not, retune
   their `departPos` in `v2x_relay\routes.rou.xml` (comments explain the
   spacing math).

Report back the console output (or a screenshot) from either run and
we'll fix anything that doesn't match.
