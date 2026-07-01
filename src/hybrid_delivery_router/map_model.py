"""Road-network model used by the hybrid delivery router."""

from __future__ import annotations

import math
import random
from typing import Dict, FrozenSet, List, Set, Tuple

RNG_SEED = 215
DEFAULT_START = "N1"
DEFAULT_GOAL = "N18"


class BoxHillDeliveryMap:
    """A compact weighted road graph with per-edge road quality metadata."""

    V_MAX = 100.0
    KM_PER_UNIT = 0.1

    def __init__(self) -> None:
        self.coords: Dict[str, Tuple[int, int]] = {
            "N1": (0, 0), "N2": (1, 0), "N3": (2, 0), "N4": (3, 0),
            "N5": (0, 1), "N6": (1, 1), "N7": (3, 1),
            "N8": (0, -1), "N9": (1, -1), "N10": (2, -1), "N11": (3, -1),
            "N12": (0, -2), "N13": (1, -2), "N14": (2, -2), "N15": (3, -2),
            "N16": (1, -3), "N17": (2, -3), "N18": (3, -3),
            "N19": (0, -4), "N20": (1, -4), "N21": (2, -4), "N22": (3, -4),
        }
        roads_units: Dict[str, Dict[str, int]] = {
            "N1": {"N2": 9, "N5": 5, "N8": 3},
            "N2": {"N1": 9, "N3": 5, "N5": 5, "N6": 4, "N9": 3},
            "N3": {"N2": 5, "N4": 8, "N10": 3},
            "N4": {"N3": 8, "N7": 4, "N11": 3},
            "N5": {"N1": 5, "N2": 5, "N6": 5},
            "N6": {"N5": 5, "N2": 4, "N7": 8},
            "N7": {"N6": 8, "N4": 4},
            "N8": {"N1": 3, "N12": 4},
            "N9": {"N2": 3, "N10": 5, "N13": 4, "N14": 5},
            "N10": {"N3": 3, "N9": 5, "N11": 8, "N14": 4},
            "N11": {"N4": 3, "N10": 8, "N15": 4},
            "N12": {"N8": 4, "N13": 9, "N19": 11},
            "N13": {"N9": 4, "N12": 9, "N14": 5, "N16": 4},
            "N14": {"N9": 5, "N10": 4, "N13": 5, "N15": 8, "N17": 4},
            "N15": {"N11": 4, "N14": 8, "N18": 4},
            "N16": {"N13": 4, "N17": 5, "N20": 7},
            "N17": {"N14": 4, "N16": 5, "N18": 8, "N21": 7},
            "N18": {"N15": 4, "N17": 8, "N22": 7},
            "N19": {"N12": 11, "N20": 9},
            "N20": {"N19": 9, "N16": 7, "N21": 5},
            "N21": {"N20": 5, "N17": 7, "N22": 8},
            "N22": {"N18": 7, "N21": 8},
        }
        self.roads_km: Dict[str, Dict[str, float]] = {
            node: {neighbor: round(cost * self.KM_PER_UNIT, 2) for neighbor, cost in edges.items()}
            for node, edges in roads_units.items()
        }
        bumpiness_pairs = [
            ("N1", "N2", 2), ("N2", "N3", 2), ("N3", "N4", 3), ("N2", "N9", 2),
            ("N3", "N10", 2), ("N4", "N11", 3), ("N9", "N10", 3), ("N10", "N14", 3),
            ("N14", "N17", 3), ("N15", "N18", 3),
            ("N1", "N5", 5), ("N2", "N5", 5), ("N2", "N6", 4), ("N5", "N6", 5),
            ("N6", "N7", 6), ("N4", "N7", 4), ("N9", "N13", 5), ("N9", "N14", 4),
            ("N13", "N14", 5), ("N10", "N11", 6), ("N11", "N15", 4), ("N14", "N15", 6),
            ("N16", "N17", 5), ("N17", "N18", 6), ("N1", "N8", 4),
            ("N8", "N12", 7), ("N12", "N13", 7), ("N12", "N19", 9), ("N13", "N16", 6),
            ("N16", "N20", 8), ("N17", "N21", 8), ("N18", "N22", 8), ("N19", "N20", 9),
            ("N20", "N21", 7), ("N21", "N22", 8),
        ]
        self.bumpiness: Dict[FrozenSet[str], int] = {
            frozenset((u, v)): score for u, v, score in bumpiness_pairs
        }
        self.landmarks: Dict[str, str] = {
            "N1": "Elgar x Whitehorse", "N2": "Station x Whitehorse",
            "N3": "Rose x Whitehorse", "N4": "Middleborough x Whitehorse",
            "N5": "Arnold x Nelson", "N6": "Thames x Station",
            "N7": "Simpsons x Whitehorse", "N8": "Elgar x Hopetoun",
            "N9": "Station x Rutland", "N10": "Rose x Rutland",
            "N11": "Middleborough x Rutland", "N12": "Elgar x Oxford",
            "N13": "Station x Oxford", "N14": "Rose x Harrow",
            "N15": "Middleborough x Sweetland", "N16": "Bass x Albion",
            "N17": "Wavell x Albion", "N18": "Middleborough x Albion",
            "N19": "Elgar x Canterbury", "N20": "Bass x Canterbury",
            "N21": "Wavell x Canterbury", "N22": "Middleborough x Canterbury",
        }
        self.km_per_grid = min(
            distance / math.hypot(self.coords[u][0] - self.coords[v][0], self.coords[u][1] - self.coords[v][1])
            for u, neighbors in self.roads_km.items()
            for v, distance in neighbors.items()
        )
        self.school_zone_active = False
        self.capped_edges: Set[FrozenSet[str]] = set()
        self._validate()

    def _validate(self) -> None:
        if set(self.roads_km) != set(self.coords) or set(self.coords) != set(self.landmarks):
            raise ValueError("Map nodes, coordinates, and landmarks are out of sync.")
        if len(self.roads_km) != 22:
            raise ValueError("Expected 22 intersections.")
        if self.num_edges() != 35:
            raise ValueError("Expected 35 road segments.")
        missing = [
            (u, v)
            for u, neighbors in self.roads_km.items()
            for v in neighbors
            if frozenset((u, v)) not in self.bumpiness
        ]
        if missing:
            raise ValueError(f"Missing bumpiness scores for edges: {missing}")
        if len(self.bumpiness) != self.num_edges():
            raise ValueError("Bumpiness metadata does not match the road graph.")

    def num_edges(self) -> int:
        return sum(len(neighbors) for neighbors in self.roads_km.values()) // 2

    def edge_keys(self) -> List[FrozenSet[str]]:
        return sorted(self.bumpiness.keys(), key=lambda edge: tuple(sorted(edge)))

    def edge_list(self) -> List[Tuple[str, str]]:
        return [tuple(sorted(edge)) for edge in self.edge_keys()]

    def neighbours(self, node: str) -> List[str]:
        if node not in self.roads_km:
            raise KeyError(f"Unknown node: {node!r}")
        return list(self.roads_km[node].keys())

    def edge_km(self, u: str, v: str) -> float:
        return self.roads_km[u][v]

    def edge_bumpiness(self, u: str, v: str) -> int:
        return self.bumpiness[frozenset((u, v))]

    def straight_line_km(self, u: str, v: str) -> float:
        (x1, y1), (x2, y2) = self.coords[u], self.coords[v]
        return self.km_per_grid * math.hypot(x1 - x2, y1 - y2)

    def apply_school_zone(self, fraction: float = 0.60, seed: int = RNG_SEED) -> None:
        edges = self.edge_keys()
        count = round(fraction * len(edges))
        self.capped_edges = set(random.Random(seed).sample(edges, count))

    def set_school_zone(self, active: bool) -> None:
        self.school_zone_active = active

    def is_capped(self, u: str, v: str) -> bool:
        return self.school_zone_active and frozenset((u, v)) in self.capped_edges

    def is_connected(self) -> bool:
        seen, stack = {next(iter(self.roads_km))}, [next(iter(self.roads_km))]
        while stack:
            current = stack.pop()
            for neighbor in self.neighbours(current):
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        return len(seen) == len(self.roads_km)

    def summary(self) -> dict[str, object]:
        distances = [distance for neighbors in self.roads_km.values() for distance in neighbors.values()]
        return {
            "nodes": len(self.roads_km),
            "edges": self.num_edges(),
            "distance_min_km": min(distances),
            "distance_max_km": max(distances),
            "km_per_grid": round(self.km_per_grid, 3),
            "connected": self.is_connected(),
        }
