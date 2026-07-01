"""A* and uniform-cost planning over the delivery graph."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set, Tuple

from .map_model import BoxHillDeliveryMap

SpeedFunction = Callable[[str, str], float]
HeuristicFunction = Callable[[str, str], float]


@dataclass(frozen=True)
class RouteResult:
    path: Optional[List[str]]
    time_min: float
    dist_km: float
    nodes_expanded: int
    max_frontier: int
    trace: List[dict] = field(default_factory=list)


class HybridPlanner:
    """A* planner where each edge cost is travel time."""

    def __init__(self, env: BoxHillDeliveryMap) -> None:
        self.env = env

    def heuristic(self, node: str, goal: str) -> float:
        return self.env.straight_line_km(node, goal) / self.env.V_MAX * 60.0

    @staticmethod
    def edge_time_min(km: float, speed_kmh: float) -> float:
        if speed_kmh <= 0:
            raise ValueError("speed_kmh must be positive")
        return km / speed_kmh * 60.0

    def astar(
        self,
        start: str,
        goal: str,
        speed_fn: SpeedFunction,
        h_fn: Optional[HeuristicFunction] = None,
        record_trace: bool = False,
    ) -> RouteResult:
        h = h_fn if h_fn is not None else self.heuristic
        tie_break = itertools.count()
        frontier: List[Tuple[float, int, str]] = [(h(start, goal), next(tie_break), start)]
        g_score: Dict[str, float] = {start: 0.0}
        parent: Dict[str, str] = {}
        closed: Set[str] = set()
        popped = 0
        peak_frontier = 1
        trace: List[dict] = []

        while frontier:
            peak_frontier = max(peak_frontier, len(frontier))
            _, _, node = heapq.heappop(frontier)
            if node in closed:
                continue
            closed.add(node)
            popped += 1

            if record_trace:
                h_value = h(node, goal)
                trace.append({
                    "order": popped,
                    "node": node,
                    "g_min": round(g_score[node], 4),
                    "h_min": round(h_value, 4),
                    "f_min": round(g_score[node] + h_value, 4),
                })

            if node == goal:
                path = self._rebuild(parent, start, goal)
                return RouteResult(path, g_score[node], self.path_km(path), popped, peak_frontier, trace)

            for neighbor in self.env.neighbours(node):
                step_cost = self.edge_time_min(self.env.edge_km(node, neighbor), speed_fn(node, neighbor))
                candidate_g = g_score[node] + step_cost
                if neighbor not in g_score or candidate_g < g_score[neighbor]:
                    g_score[neighbor] = candidate_g
                    parent[neighbor] = node
                    heapq.heappush(frontier, (candidate_g + h(neighbor, goal), next(tie_break), neighbor))

        return RouteResult(None, math.inf, math.inf, popped, peak_frontier, trace)

    def uniform_cost(self, start: str, goal: str, speed_fn: SpeedFunction) -> RouteResult:
        return self.astar(start, goal, speed_fn, h_fn=lambda _node, _goal: 0.0)

    @staticmethod
    def _rebuild(parent: Dict[str, str], start: str, goal: str) -> List[str]:
        if start == goal:
            return [start]
        path = [goal]
        current = goal
        while current != start:
            current = parent[current]
            path.append(current)
        return path[::-1]

    def path_km(self, path: List[str]) -> float:
        return sum(self.env.edge_km(path[i], path[i + 1]) for i in range(len(path) - 1))

    def path_time_min(self, path: List[str], speed_fn: SpeedFunction) -> float:
        return sum(
            self.edge_time_min(self.env.edge_km(path[i], path[i + 1]), speed_fn(path[i], path[i + 1]))
            for i in range(len(path) - 1)
        )


def constant_speed(speed_kmh: float = 100.0) -> SpeedFunction:
    return lambda _u, _v: speed_kmh
