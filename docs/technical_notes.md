# Technical Notes

## System Overview

This project models a delivery agent over a 22-node, 35-edge road graph. The
agent chooses routes by minimizing travel time rather than distance. Travel time
is determined by a speed model, which lets the same A* implementation solve
three different planning modes:

1. Constant-speed baseline at 100 km/h.
2. Fuzzy safe-speed planning from cargo fragility and road bumpiness.
3. Reactive replanning after a school-zone speed cap becomes active mid-journey.

## Why A*

A* is a good fit because the road graph is weighted and the target is known. The
heuristic uses straight-line distance divided by an upper bound on speed. That
keeps the estimate optimistic, so A* preserves optimality while expanding fewer
nodes than uniform-cost search.

## Fuzzy Controller

The fuzzy controller maps two inputs to a safe segment speed:

- Cargo fragility: robust, moderate, delicate.
- Road bumpiness: smooth, moderate, rough.
- Output speed: slow, medium, fast.

The controller is implemented from scratch with NumPy. It uses min activation and
a centre-of-sets weighted average. A monotonicity test checks that safe speed never
increases when cargo fragility or road bumpiness increases.

## Reactive Replanning

The agent first plans under normal fuzzy speeds. After 20 percent of the route, a
school zone becomes active and caps selected edges at 40 km/h. The already-driven
prefix keeps its original cost, while the remaining tail is replanned from the
current node using the new speed model.

## Verification

The test suite checks:

- Map shape: 22 nodes, 35 edges, connected graph.
- Baseline A* route and cost.
- A* optimality against uniform-cost search.
- Fuzzy controller bounds and monotonicity.
- Reactive replanning path and phase-aware travel time.
- Good heuristic admissibility and a deliberate bad-heuristic counterexample.

## Known Limitations

- The road graph is a compact simulation, not a live GIS network.
- School-zone activation is deterministic rather than event-driven from live data.
- Bumpiness scores are synthetic but structured to support explainable fuzzy rules.
- The current implementation optimizes time only; a production router could add
  emissions, delivery windows, turn costs, or risk preferences.
