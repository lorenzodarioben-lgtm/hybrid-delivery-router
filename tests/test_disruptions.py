from hybrid_delivery_router.disruptions import DisruptionOverlay, RoadDisruption, school_zone_preset


def test_disruption_overlay_supports_caps_congestion_and_closures() -> None:
    edge = (("A", "B"),)
    cap = school_zone_preset(edge)
    congestion = RoadDisruption("traffic", "congestion", edge, "Traffic", value=2.0)
    closure = RoadDisruption("closed", "closure", edge, "Works")

    assert DisruptionOverlay((cap,)).speed("A", "B", 70.0, 0.0) == 40.0
    assert DisruptionOverlay((congestion,)).speed("A", "B", 70.0, 0.0) == 35.0
    assert DisruptionOverlay((closure,)).speed("A", "B", 70.0, 0.0) is None
