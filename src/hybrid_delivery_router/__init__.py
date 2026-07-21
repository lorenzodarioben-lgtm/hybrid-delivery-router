"""Hybrid delivery routing package."""

from .agent import (
    FRAGILITY_LEVELS,
    JourneyRecord,
    ReactiveAgent,
    cargo_top_speed,
    fuzzy_informed_heuristic,
    fuzzy_speed,
    schoolzone_speed,
)
from .disruptions import DisruptionOverlay, RoadDisruption, school_zone_preset
from .domain import (
    CargoProfile,
    InvalidRouteError,
    JourneyResult,
    JourneySegment,
    NetworkValidationError,
    Node,
    RoadNetwork,
    RoadSegment,
    RouteObjective,
    RoutePlan,
    RoutingError,
    ScoreBreakdown,
    SearchEvent,
    SearchStatistics,
    UnknownNodeError,
    UnreachableRouteError,
)
from .fuzzy import FuzzyExplanation, FuzzySpeedController, FuzzyVariable, trapmf, trimf
from .journeys import simulate_segments
from .map_model import DEFAULT_GOAL, DEFAULT_START, RNG_SEED, BoxHillDeliveryMap, ScenarioMap
from .planner import HybridPlanner, RouteResult, constant_speed
from .scenarios import DEFAULT_SCENARIO_ID, ScenarioLoadError, available_scenarios, load_scenario
from .tracing import summarize_trace

__all__ = [
    "BoxHillDeliveryMap",
    "CargoProfile",
    "DEFAULT_GOAL",
    "DisruptionOverlay",
    "DEFAULT_SCENARIO_ID",
    "DEFAULT_START",
    "FRAGILITY_LEVELS",
    "FuzzySpeedController",
    "FuzzyExplanation",
    "FuzzyVariable",
    "HybridPlanner",
    "InvalidRouteError",
    "JourneyRecord",
    "JourneyResult",
    "JourneySegment",
    "NetworkValidationError",
    "Node",
    "RoadNetwork",
    "RoadDisruption",
    "RoadSegment",
    "RouteObjective",
    "RoutePlan",
    "RNG_SEED",
    "ReactiveAgent",
    "RouteResult",
    "RoutingError",
    "ScoreBreakdown",
    "SearchEvent",
    "SearchStatistics",
    "ScenarioLoadError",
    "ScenarioMap",
    "UnknownNodeError",
    "UnreachableRouteError",
    "cargo_top_speed",
    "available_scenarios",
    "constant_speed",
    "fuzzy_informed_heuristic",
    "fuzzy_speed",
    "load_scenario",
    "schoolzone_speed",
    "school_zone_preset",
    "simulate_segments",
    "summarize_trace",
    "trapmf",
    "trimf",
]

__version__ = "0.1.0"
