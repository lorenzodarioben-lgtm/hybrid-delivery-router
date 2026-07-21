"""Distance-aware journey simulation with immutable segment accounting."""

from __future__ import annotations

from .disruptions import DisruptionOverlay
from .domain import JourneySegment
from .fuzzy import FuzzySpeedController
from .map_model import ScenarioMap
from .planner import HybridPlanner


def simulate_segments(
    road_map: ScenarioMap,
    planner: HybridPlanner,
    controller: FuzzySpeedController,
    path: tuple[str, ...],
    fragility: float,
    overlay: DisruptionOverlay = DisruptionOverlay(),
) -> tuple[JourneySegment, ...]:
    """Replay a path using actual distance progress and immutable disruption overlays."""
    road_map.validate_path(path)
    travelled = 0.0
    segments: list[JourneySegment] = []
    for start, end in zip(path, path[1:]):
        distance = road_map.edge_km(start, end)
        base_speed = controller.safe_speed(fragility, road_map.edge_bumpiness(start, end))
        speed = overlay.speed(start, end, base_speed, travelled)
        if speed is None:
            raise RuntimeError(f"Road closure prevents travel from {start} to {end}")
        active = next(
            (
                item.identifier
                for item in overlay.disruptions
                if item.affects(start, end, travelled)
            ),
            None,
        )
        segments.append(
            JourneySegment(
                start,
                end,
                distance,
                road_map.edge_bumpiness(start, end),
                speed,
                planner.edge_time_min(distance, speed),
                active,
                active is None,
            )
        )
        travelled += distance
    return tuple(segments)
