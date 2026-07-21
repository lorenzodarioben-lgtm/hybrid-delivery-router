"""Concise, presentation-safe summaries of typed search traces."""

from __future__ import annotations

from .domain import SearchEvent


def summarize_trace(events: tuple[SearchEvent, ...]) -> str:
    """Summarise an optional trace without exposing internal graph objects."""

    if not events:
        return "Trace recording was disabled."
    selected = sum(event.kind == "select" for event in events)
    relaxations = sum(event.kind == "relax" for event in events)
    reopened = sum(event.kind == "reopen" for event in events)
    goal = next((event for event in events if event.kind == "goal"), None)
    destination = goal.node if goal is not None else "no reachable goal"
    return (
        f"Selected {selected} nodes, accepted {relaxations} edge relaxations, "
        f"reopened {reopened} nodes, and finished at {destination}."
    )
