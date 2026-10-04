# 10-signal emergency corridor (SUMO + TraCI)

    J1  - J2  - J3  - J4  - J5      8 ambulances in 3 waves (~9 min), ~530 background cars
    |     |     |     |     |      crossing routes, incl. long diagonals through many signals
    J6  - J7  - J8  - J9  - J10

## Setup (Ubuntu)
    sudo apt install sumo sumo-tools python3-venv
    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
    # network, ambulances and traffic are already included; to regenerate:
    SUMO_HOME=/usr/share/sumo .venv/bin/python build_network.py

`traci`/`sumolib` must match the installed SUMO (1.18.0 on Ubuntu 24.04).

## Demo (for presenting)
    ./run.sh --pause               # GUI, full V2X, starts paused (press Play)
    ./run.sh --baseline            # same scenario WITHOUT V2X (show this first)
    ./compare.sh                   # runs 3 modes, prints a table, writes results/comparison.png

Suggested flow: run `--baseline` (ambulances stop at reds), then `./run.sh --pause`
(signals turn green ahead of them), then show `results/comparison.png`.

In the GUI (free camera: mouse wheel zooms at the cursor, drag to pan; the
magnifier icon in the toolbar lets you type an exact zoom/position):
- every junction is tagged J1..J10
- ring at a junction: orange = warned by I2I, yellow = clearing, green = priority
- coloured line between two signals = the I2I warning from one to the next
  (in that ambulance's colour); it disappears once the ambulance passes
- each ambulance: highlight circle, label (speed + next signal + distance),
  coloured ribbon along its remaining route
- board above the map: legend + the latest I2I / GREEN / arrival events

## Options
    --baseline            no V2X
    --no-v2v              V2I + I2I only (no lane clearing)
    --nogui               headless
    --pause               start paused
    --focus J3 --zoom 450 start zoomed in on a junction (100 = whole map)
    --track auto|amb3     LOCK the camera on an ambulance (no free pan/zoom)
    --delay 100           playback speed in ms per step (default 100; higher = slower)
    --latency 0.1 --loss 0.2     imperfect V2X channel

## Rebuild network / traffic / ambulances
    SUMO_HOME=/usr/share/sumo .venv/bin/python build_network.py
Edit the AMBULANCES list at the top of build_network.py to change routes/times.

## Results (this scenario)
| | travel time (sum) | time stopped | stops |
|---|---|---|---|
| Baseline | 1315 s | 373 s | 24 |
| V2I + I2I | 836 s | 7 s | 9 |
| + V2V | 836 s | 7 s | 9 |

## Notes
- SUMO's blue-light device is OFF: with it on, SUMO ambulances run red lights,
  which makes the baseline meaningless.
- V2V adds nothing measurable here: the ambulances already overtake on 2-lane
  roads, so the gain comes from the signals.
- When two ambulances reach one signal together, the nearer is served first
  and the other waits (see amb8).
- All V2I/I2I messages pass through `Channel` in v2x_city.py. Replace its
  latency/loss with real ns-3 NR-V2X results to couple the two simulators.

## Tips
- Zoom: mouse wheel zooms at the cursor; the magnifier icon in the toolbar
  opens a dialog to type an exact zoom/position.
- `./run.sh --focus J3 --zoom 450` starts zoomed in on one junction.
- `--track` locks the camera to an ambulance, so free pan/zoom is disabled
  while it is on.
