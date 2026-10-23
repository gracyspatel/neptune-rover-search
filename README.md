# Neptune Rover Search

Rover pathfinding agent (BFS, UCS, A*) for CSCI-561 at USC.

A search agent that plans a rover's route across a graph of named waypoints, from
`start` to `goal`, subject to elevation-dependent **energy** and **momentum**
constraints. Implements three classic search strategies — Breadth-First Search,
Uniform-Cost Search, and A* — selected at runtime via the input file.

## Problem setup

- The world is a graph: each node has 3D coordinates `(x, y, z)`, and edges are
  undirected, unweighted connections between nodes.
- Moving from one node to another costs or gains **momentum** depending on the
  change in elevation (`z`):
  - Moving **uphill** (`z` increases) costs energy, drawn first from momentum,
    then from the rover's fixed energy budget.
  - Moving **downhill** (`z` decreases) builds momentum, capped at the momentum
    limit.
  - A move is only legal if the energy required does not exceed
    `current_momentum + energy_limit`.
- The agent must find a path from `start` to `goal` that respects these
  constraints, using the algorithm specified in the input.

## Algorithms

| Algorithm | Edge cost used | Goal |
|---|---|---|
| **BFS** | none (unweighted) | shortest path by number of hops |
| **UCS** | 2D Euclidean distance (`x`, `y` only) | lowest total travel cost |
| **A\*** | 3D Euclidean distance (`x`, `y`, `z`), with a 3D-distance-to-goal heuristic | lowest total travel cost, found faster than UCS |

All three algorithms track `(node, momentum)` as search state, since the same
node can be reached with different momentum values, which affects which moves
are legal from there.

## Files

- `homework.py` — the agent. Reads `input.txt`, writes `output.txt`, both in the
  current directory.
- `training/` — 50 provided test cases, each as a trio:
  - `inputN.txt` — algorithm name, energy limit, momentum limit, node list, edge list
  - `outputN.txt` — a reference solution path
  - `pathlenN.txt` — the reference path's cost (hop count for BFS, distance for UCS/A*)
  - `000-runtimes.txt` — reference timing notes for the test cases
- `output.txt` — the most recent output produced by running the agent.

## Input format (`input.txt`)

```
<ALGORITHM>              # BFS, UCS, or A*
<energy_limit>            # integer
<momentum_limit>          # integer
<number_of_locations>
<name> <x> <y> <z>        # one line per location, including "start" and "goal"
...
<number_of_edges>
<name1> <name2>           # one line per undirected edge
...
```

## Output format (`output.txt`)

- On success: the path from `start` to `goal` as space-separated node names, e.g.
  `start abcd efgh goal`.
- On failure (no valid path exists): the single word `FAIL`.

## Running it

```bash
# Place an input.txt (see format above) in the same directory as homework.py, then:
python homework.py
```

The result is written to `output.txt` in the same directory.

## Verifying against the training set

Each training case can be checked by copying `inputN.txt` to `input.txt`, running
`homework.py`, and comparing:
1. the produced path is a sequence of valid, constraint-respecting edges from
   `start` to `goal`, and
2. its cost (hop count for BFS, summed edge distance for UCS/A*) matches
   `pathlenN.txt`.

See `RESULTS.txt` for the full per-testcase breakdown, or the summary below.

## Results

Verified against all 50 provided training cases (2026-10-23).

**50/50 passed.**

| Algorithm | Passed |
|---|---|
| BFS | 11/11 |
| UCS | 20/20 |
| A* | 19/19 |

A pass means the produced path is legal (every step is a real edge and respects
the energy/momentum rule) and its cost matches the reference value in
`training/pathlenN.txt`, within a 1% tolerance for floating-point differences.

Timing: slowest case was Test 21 (UCS) at 5.999s, fastest was Test 20 (A*) at
0.130s. All 50 cases completed well within typical grading time limits.

Full per-testcase results (cost produced vs. expected, timing) are in
[`RESULTS.txt`](RESULTS.txt).
