import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hybrid_delivery_router import (
    DEFAULT_GOAL,
    DEFAULT_START,
    BoxHillDeliveryMap,
    FuzzySpeedController,
    HybridPlanner,
    ReactiveAgent,
    cargo_top_speed,
    constant_speed,
    fuzzy_informed_heuristic,
    fuzzy_speed,
    schoolzone_speed,
)
from hybrid_delivery_router.evaluation import (
    admissibility_audit,
    bad_school_zone_heuristic,
    phase_aware_breakdown,
)


class HybridDeliveryRouterTest(unittest.TestCase):
    def setUp(self):
        self.env = BoxHillDeliveryMap()
        self.env.apply_school_zone()
        self.controller = FuzzySpeedController()
        self.planner = HybridPlanner(self.env)

    def test_map_integrity_and_static_missing_link(self):
        self.assertEqual(len(self.env.roads_km), 22)
        self.assertEqual(self.env.num_edges(), 35)
        self.assertTrue(self.env.is_connected())
        self.assertNotIn("N9", self.env.neighbours("N8"))
        self.assertNotIn("N8", self.env.neighbours("N9"))
        summary = self.env.summary()
        self.assertEqual(summary["distance_min_km"], 0.3)
        self.assertEqual(summary["distance_max_km"], 1.1)
        self.assertEqual(len(self.env.capped_edges), 21)

    def test_baseline_astar_matches_uniform_cost(self):
        speed = constant_speed(100.0)
        astar = self.planner.astar(DEFAULT_START, DEFAULT_GOAL, speed)
        ucs = self.planner.uniform_cost(DEFAULT_START, DEFAULT_GOAL, speed)
        self.assertEqual(astar.path, ["N1", "N2", "N9", "N14", "N17", "N18"])
        self.assertAlmostEqual(astar.dist_km, 2.9)
        self.assertAlmostEqual(astar.time_min, 1.74)
        self.assertEqual(astar.nodes_expanded, 16)
        self.assertEqual(ucs.nodes_expanded, 21)
        self.assertAlmostEqual(astar.time_min, ucs.time_min)

    def test_start_equals_goal_and_unreachable_route_handling(self):
        same = self.planner.astar("N1", "N1", constant_speed(100.0))
        self.assertEqual(same.path, ["N1"])
        self.assertEqual(same.dist_km, 0)
        self.assertEqual(same.time_min, 0)

        isolated = BoxHillDeliveryMap()
        for neighbours in isolated.roads_km.values():
            neighbours.pop("N18", None)
        isolated.roads_km["N18"] = {}
        isolated_planner = HybridPlanner(isolated)
        missing = isolated_planner.astar(DEFAULT_START, DEFAULT_GOAL, constant_speed(100.0))
        self.assertIsNone(missing.path)
        self.assertTrue(math.isinf(missing.time_min))
        self.assertTrue(math.isinf(missing.dist_km))

        agent = ReactiveAgent(isolated, self.controller, isolated_planner)
        with self.assertRaises(RuntimeError):
            agent.run(DEFAULT_START, DEFAULT_GOAL, 2.0)

    def test_fuzzy_controller_is_bounded_and_monotone(self):
        self.assertAlmostEqual(self.controller.safe_speed(1, 1), 88.36, places=2)
        self.assertAlmostEqual(self.controller.safe_speed(5, 5), 70.0, places=2)
        self.assertAlmostEqual(self.controller.safe_speed(9, 9), 51.64, places=2)
        self.assertTrue(40 <= self.controller.safe_speed(float("nan"), 3) <= 100)
        self.assertEqual(
            self.controller.monotonicity_violations(), {"fragility": 0, "bumpiness": 0}
        )

    def test_school_zone_caps_only_when_active(self):
        capped = tuple(sorted(next(iter(self.env.capped_edges))))
        uncapped = next(
            edge for edge in self.env.edge_list() if frozenset(edge) not in self.env.capped_edges
        )
        speed = schoolzone_speed(self.env, self.controller, 2.0)

        self.env.set_school_zone(False)
        self.assertAlmostEqual(
            speed(*capped), self.controller.safe_speed(2.0, self.env.edge_bumpiness(*capped))
        )

        self.env.set_school_zone(True)
        self.assertEqual(speed(*capped), 40.0)
        self.assertAlmostEqual(
            speed(*uncapped), self.controller.safe_speed(2.0, self.env.edge_bumpiness(*uncapped))
        )

    def test_fuzzy_routes_and_replanning(self):
        robust_h = fuzzy_informed_heuristic(self.env, cargo_top_speed(self.controller, 2.0))
        robust = self.planner.astar(
            DEFAULT_START, DEFAULT_GOAL, fuzzy_speed(self.env, self.controller, 2.0), h_fn=robust_h
        )
        self.assertEqual(robust.path, ["N1", "N2", "N9", "N14", "N15", "N18"])
        self.assertAlmostEqual(robust.time_min, 2.0322, places=4)
        self.assertEqual(robust.nodes_expanded, 15)

        agent = ReactiveAgent(self.env, self.controller, self.planner)
        journey = agent.run(DEFAULT_START, DEFAULT_GOAL, 2.0)
        self.assertEqual(journey.actual_path, ["N1", "N2", "N3", "N10", "N14", "N15", "N18"])
        self.assertTrue(journey.rerouted)
        self.assertAlmostEqual(journey.actual_min, 2.8785, places=4)
        self.assertEqual(journey.total_planning_nodes, 31)

        phase_total = sum(
            row["time_min"] for row in phase_aware_breakdown(journey, self.env, self.controller)
        )
        self.assertAlmostEqual(phase_total, journey.actual_min, places=9)

    def test_heuristic_audits(self):
        good = admissibility_audit(
            self.env, self.planner, DEFAULT_GOAL, constant_speed(100.0), self.planner.heuristic
        )
        bad = admissibility_audit(
            self.env,
            self.planner,
            DEFAULT_GOAL,
            constant_speed(100.0),
            bad_school_zone_heuristic(self.env),
        )
        self.assertEqual(sum(not row["admissible"] for row in good), 0)
        self.assertEqual(sum(not row["admissible"] for row in bad), 13)

    def test_fuzzy_heuristic_is_admissible_and_matches_ucs(self):
        for fragility in (2.0, 5.0, 8.0):
            speed = fuzzy_speed(self.env, self.controller, fragility)
            heuristic = fuzzy_informed_heuristic(
                self.env, cargo_top_speed(self.controller, fragility)
            )
            audit = admissibility_audit(self.env, self.planner, DEFAULT_GOAL, speed, heuristic)
            self.assertEqual(sum(not row["admissible"] for row in audit), 0)
            astar = self.planner.astar(DEFAULT_START, DEFAULT_GOAL, speed, h_fn=heuristic)
            ucs = self.planner.uniform_cost(DEFAULT_START, DEFAULT_GOAL, speed)
            self.assertAlmostEqual(astar.time_min, ucs.time_min)


if __name__ == "__main__":
    unittest.main()
