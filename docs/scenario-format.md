# Scenario format

Scenarios are local JSON files in `src/hybrid_delivery_router/data/`. They describe deterministic,
synthetic road networks and do not use live GIS or mapping data.

```json
{
  "id": "example",
  "name": "Readable scenario name",
  "description": "Optional context for users.",
  "default_start": "A",
  "default_goal": "B",
  "nodes": [{"id": "A", "coordinates": [0, 0], "label": "Start"}],
  "roads": [{"start": "A", "end": "B", "distance_km": 0.5, "bumpiness": 3}]
}
```

Each road is an undirected segment and appears once. Coordinates are synthetic Cartesian values
used only for the admissible straight-line heuristic and SVG visualisation. Distances are kilometres;
bumpiness is a score from 0 (smooth) to 10 (rough). The loader preserves deterministic ordering and
the next validation layer rejects malformed, disconnected, or inconsistent definitions.
