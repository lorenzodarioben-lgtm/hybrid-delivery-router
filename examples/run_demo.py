import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hybrid_delivery_router import DEFAULT_GOAL, DEFAULT_START, FuzzySpeedController
from hybrid_delivery_router.evaluation import master_comparison, sensitivity_analysis


def main() -> None:
    controller = FuzzySpeedController()
    print("Hybrid Delivery Router")
    print(f"Route target: {DEFAULT_START} -> {DEFAULT_GOAL}")
    print("Consequent centroids:", {k: round(v, 1) for k, v in controller.cons_centroid.items()})
    print()
    print("Master comparison")
    for row in master_comparison():
        print(
            f"- {row['case']}: {row['time_min']:.2f} min, "
            f"{row['distance_km']:.2f} km, nodes={row['planning_nodes']}, "
            f"rerouted={row['rerouted']}"
        )
    print()
    print("Sensitivity")
    for row in sensitivity_analysis():
        print(
            f"- fragility {row['fragility']:.0f}: planned {row['planned_time_min']:.2f} min, "
            f"actual {row['actual_time_min']:.2f} min, "
            f"avg speed {row['avg_planned_speed_kmh']:.1f} km/h"
        )


if __name__ == "__main__":
    main()
