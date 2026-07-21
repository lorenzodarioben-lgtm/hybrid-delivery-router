"""A* and uniform-cost planning over a typed delivery graph contract."""

from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Callable
from typing import Protocol

from .domain import RoutePlan, ScoreBreakdown, SearchEvent, SearchEventKind, SearchStatistics

SpeedFunction = Callable[[str, str], float]
HeuristicFunction = Callable[[str, str], float]


class RoutingGraph(Protocol):
    """The small graph surface the planner needs from any network implementation."""

    V_MAX: float

    def neighbours(self, node: str) -> list[str]: ...

    def edge_km(self, start: str, end: str) -> float: ...

    def straight_line_km(self, start: str, end: str) -> float: ...


RouteResult = RoutePlan


class HybridPlanner:
    """Plan minimum travel-time paths over any graph satisfying :class:`RoutingGraph`."""

    def __init__(self, network: RoutingGraph) -> None:
        self.network = network
        self.env = network

    def heuristic(self, node: str, goal: str) -> float:
        return self.network.straight_line_km(node, goal) / self.network.V_MAX * 60.0

    @staticmethod
    def edge_time_min(km: float, speed_kmh: float) -> float:
        if not math.isfinite(speed_kmh) or speed_kmh <= 0:
            raise ValueError("speed_kmh must be a positive finite value")
        return km / speed_kmh * 60.0

    def astar(
        self,
        start: str,
        goal: str,
        speed_fn: SpeedFunction,
        h_fn: HeuristicFunction | None = None,
        record_trace: bool = False,
    ) -> RoutePlan:
        self.network.neighbours(start)
        self.network.neighbours(goal)
        h = h_fn if h_fn is not None else self.heuristic
        tie_break = itertools.count()
        frontier: list[tuple[float, int, str, float]] = [
            (h(start, goal), next(tie_break), start, 0.0)
        ]
        g_score: dict[str, float] = {start: 0.0}
        parent: dict[str, str] = {}
        closed: set[str] = set()
        selected = 0
        peak_frontier = 1
        reopened = 0
        stale_entries = 0
        trace: list[SearchEvent] = []

        def record(
            kind: SearchEventKind,
            node: str,
            g_min: float,
            h_min: float,
            *,
            edge: tuple[str, str] | None = None,
            related_node: str | None = None,
            accepted: bool | None = None,
        ) -> None:
            if record_trace:
                trace.append(
                    SearchEvent(
                        kind=kind,
                        order=len(trace) + 1,
                        node=node,
                        g_min=g_min,
                        h_min=h_min,
                        f_min=g_min + h_min,
                        edge=edge,
                        related_node=related_node,
                        frontier_size=len(frontier),
                        accepted=accepted,
                    )
                )

        while frontier:
            peak_frontier = max(peak_frontier, len(frontier))
            _priority, _tie, node, queued_g = heapq.heappop(frontier)
            if queued_g > g_score.get(node, math.inf):
                stale_entries += 1
                continue
            if node in closed:
                stale_entries += 1
                continue
            closed.add(node)
            selected += 1
            h_value = h(node, goal)

            record("select", node, g_score[node], h_value)

            if node == goal:
                path = self._rebuild(parent, start, goal)
                record("goal", node, g_score[node], h_value, accepted=True)
                record("reconstruct", node, g_score[node], h_value, accepted=True)
                return RoutePlan(
                    path=path,
                    time_min=g_score[node],
                    distance_km=self.path_km(path),
                    statistics=SearchStatistics(
                        selected,
                        peak_frontier,
                        nodes_reopened=reopened,
                        stale_entries_skipped=stale_entries,
                    ),
                    trace=tuple(trace),
                    score=ScoreBreakdown(travel_time=g_score[node], total=g_score[node]),
                )

            for neighbor in self.network.neighbours(node):
                step_cost = self.edge_time_min(
                    self.network.edge_km(node, neighbor), speed_fn(node, neighbor)
                )
                candidate_g = g_score[node] + step_cost
                neighbor_h = h(neighbor, goal)
                record(
                    "consider",
                    node,
                    candidate_g,
                    neighbor_h,
                    edge=(node, neighbor),
                    related_node=neighbor,
                )
                if candidate_g < g_score.get(neighbor, math.inf):
                    if neighbor in closed:
                        closed.remove(neighbor)
                        reopened += 1
                        record(
                            "reopen",
                            neighbor,
                            candidate_g,
                            neighbor_h,
                            edge=(node, neighbor),
                            related_node=node,
                            accepted=True,
                        )
                    g_score[neighbor] = candidate_g
                    parent[neighbor] = node
                    heapq.heappush(
                        frontier,
                        (candidate_g + neighbor_h, next(tie_break), neighbor, candidate_g),
                    )
                    record(
                        "relax",
                        neighbor,
                        candidate_g,
                        neighbor_h,
                        edge=(node, neighbor),
                        related_node=node,
                        accepted=True,
                    )
                else:
                    record(
                        "reject",
                        neighbor,
                        candidate_g,
                        neighbor_h,
                        edge=(node, neighbor),
                        related_node=node,
                        accepted=False,
                    )

        return RoutePlan(
            path=None,
            time_min=math.inf,
            distance_km=math.inf,
            statistics=SearchStatistics(
                selected,
                peak_frontier,
                nodes_reopened=reopened,
                stale_entries_skipped=stale_entries,
            ),
            trace=tuple(trace),
        )

    def uniform_cost(self, start: str, goal: str, speed_fn: SpeedFunction) -> RoutePlan:
        return self.astar(start, goal, speed_fn, h_fn=lambda _node, _goal: 0.0)

    @staticmethod
    def _rebuild(parent: dict[str, str], start: str, goal: str) -> tuple[str, ...]:
        if start == goal:
            return (start,)
        path = [goal]
        current = goal
        while current != start:
            current = parent[current]
            path.append(current)
        return tuple(reversed(path))

    def path_km(self, path: tuple[str, ...] | list[str]) -> float:
        validate_path = getattr(self.network, "validate_path", None)
        if callable(validate_path):
            validate_path(path)
        return sum(
            self.network.edge_km(path[index], path[index + 1]) for index in range(len(path) - 1)
        )

    def path_time_min(self, path: tuple[str, ...] | list[str], speed_fn: SpeedFunction) -> float:
        validate_path = getattr(self.network, "validate_path", None)
        if callable(validate_path):
            validate_path(path)
        return sum(
            self.edge_time_min(
                self.network.edge_km(path[index], path[index + 1]),
                speed_fn(path[index], path[index + 1]),
            )
            for index in range(len(path) - 1)
        )


def constant_speed(speed_kmh: float = 100.0) -> SpeedFunction:
    """Return a speed function that applies a constant legal speed to every edge."""

    return lambda _start, _end: speed_kmh
