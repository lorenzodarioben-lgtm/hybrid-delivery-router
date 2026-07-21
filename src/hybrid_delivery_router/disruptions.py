"""Immutable road-disruption overlays that never mutate a base scenario."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .domain import Edge

DisruptionKind = Literal["speed_cap", "congestion", "closure"]


@dataclass(frozen=True, slots=True)
class RoadDisruption:
    identifier: str
    kind: DisruptionKind
    affected_edges: tuple[Edge, ...]
    reason: str
    activation_distance_km: float = 0.0
    end_distance_km: float | None = None
    value: float = 1.0

    def affects(self, start: str, end: str, distance_km: float) -> bool:
        edge = tuple(sorted((start, end)))
        active = distance_km >= self.activation_distance_km
        if self.end_distance_km is not None:
            active = active and distance_km < self.end_distance_km
        return active and edge in self.affected_edges


@dataclass(frozen=True, slots=True)
class DisruptionOverlay:
    disruptions: tuple[RoadDisruption, ...] = ()

    def speed(
        self, start: str, end: str, base_speed_kmh: float, distance_km: float
    ) -> float | None:
        speed = base_speed_kmh
        for disruption in self.disruptions:
            if not disruption.affects(start, end, distance_km):
                continue
            if disruption.kind == "closure":
                return None
            if disruption.kind == "speed_cap":
                speed = min(speed, disruption.value)
            if disruption.kind == "congestion":
                speed /= disruption.value
        return speed


def school_zone_preset(
    edges: tuple[Edge, ...], activation_distance_km: float = 0.0
) -> RoadDisruption:
    """Preserve the original deterministic school-zone demonstration as a preset."""

    return RoadDisruption(
        "school-zone",
        "speed_cap",
        edges,
        "School-zone speed cap",
        activation_distance_km,
        value=40.0,
    )
