"""Command-line entry point for deterministic scenario inspection and planning."""

from __future__ import annotations

import argparse
import json

from .map_model import ScenarioMap
from .planner import HybridPlanner, constant_speed
from .scenarios import available_scenarios, load_scenario


def main() -> None:
    parser = argparse.ArgumentParser(prog="hybrid-router")
    parser.add_argument("command", choices=("list", "inspect", "plan"))
    parser.add_argument("--scenario", default="box-hill-synthetic")
    parser.add_argument("--start")
    parser.add_argument("--goal")
    args = parser.parse_args()
    if args.command == "list":
        print("\n".join(available_scenarios()))
        return
    network = load_scenario(args.scenario)
    if args.command == "inspect":
        print(
            json.dumps(
                {
                    "id": network.identifier,
                    "nodes": len(network.nodes),
                    "roads": len(network.roads),
                },
                sort_keys=True,
            )
        )
        return
    road_map = ScenarioMap(network)
    route = HybridPlanner(road_map).astar(
        args.start or network.default_start or "",
        args.goal or network.default_goal or "",
        constant_speed(),
    )
    print(
        json.dumps(
            {"path": route.path, "time_min": route.time_min, "distance_km": route.distance_km},
            default=list,
            sort_keys=True,
        )
    )
