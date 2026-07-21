"""Transparent weighted route scoring."""

from .domain import RouteObjective, ScoreBreakdown


def score_route(
    time_min: float, distance_km: float, risk: float, exposure: float, objective: RouteObjective
) -> ScoreBreakdown:
    total = (
        time_min * objective.travel_time
        + distance_km * objective.distance
        + risk * objective.cargo_risk
        + exposure * objective.disruption_exposure
    )
    return ScoreBreakdown(time_min, distance_km, risk, exposure, total)
