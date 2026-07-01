"""Hand-built fuzzy speed controller."""

from __future__ import annotations

import math
from typing import Callable, Dict, Tuple

import numpy as np


def trimf(x, abc):
    """Triangular membership function."""
    a, b, c = abc
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)
    if a != b:
        y = np.where((x > a) & (x < b), (x - a) / (b - a), y)
    if b != c:
        y = np.where((x >= b) & (x < c), (c - x) / (c - b), y)
    y = np.where(x == b, 1.0, y)
    return y


def trapmf(x, abcd):
    """Trapezoidal membership function with optional shoulder sides."""
    a, b, c, d = abcd
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)
    if a != b:
        y = np.where((x > a) & (x < b), (x - a) / (b - a), y)
    y = np.where((x >= b) & (x <= c), 1.0, y)
    if c != d:
        y = np.where((x > c) & (x < d), (d - x) / (d - c), y)
    return y


class FuzzyVariable:
    """A linguistic variable with named membership functions."""

    def __init__(self, name: str, universe, unit: str = "") -> None:
        self.name = name
        self.universe = np.asarray(universe, dtype=float)
        self.unit = unit
        self.mfs: Dict[str, Callable] = {}
        self._curve: Dict[str, np.ndarray] = {}

    def add(self, label: str, fn: Callable):
        self.mfs[label] = fn
        self._curve[label] = fn(self.universe)
        return self

    def mu(self, label: str, value: float) -> float:
        value = _finite_or_default(value, float(self.universe[0]))
        value = float(np.clip(value, self.universe[0], self.universe[-1]))
        return float(self.mfs[label](np.array([value]))[0])

    def fuzzify(self, value: float) -> Dict[str, float]:
        return {label: self.mu(label, value) for label in self.mfs}

    def curve(self, label: str) -> np.ndarray:
        return self._curve[label]


def _finite_or_default(value: float, default: float) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


class FuzzySpeedController:
    """Maps cargo fragility and road bumpiness to a safe speed in km/h."""

    def __init__(self) -> None:
        u_in = np.linspace(0, 10, 1001)
        self.fragility = (
            FuzzyVariable("Cargo Fragility", u_in)
            .add("Robust", lambda x: trapmf(x, (0, 0, 3, 5)))
            .add("Moderate", lambda x: trimf(x, (3, 5, 7)))
            .add("Delicate", lambda x: trapmf(x, (5, 7, 10, 10)))
        )
        self.bumpiness = (
            FuzzyVariable("Road Bumpiness", u_in)
            .add("Smooth", lambda x: trapmf(x, (0, 0, 3, 5)))
            .add("Moderate", lambda x: trimf(x, (3, 5, 7)))
            .add("Rough", lambda x: trapmf(x, (5, 7, 10, 10)))
        )
        self.speed_universe = np.linspace(40, 100, 601)
        self.speed = (
            FuzzyVariable("Max Safe Speed", self.speed_universe, "km/h")
            .add("Slow", lambda x: trapmf(x, (40, 40, 55, 70)))
            .add("Medium", lambda x: trimf(x, (55, 70, 85)))
            .add("Fast", lambda x: trapmf(x, (70, 85, 100, 100)))
        )
        self.cons_centroid = {
            label: float((self.speed_universe * self.speed.curve(label)).sum() / self.speed.curve(label).sum())
            for label in self.speed.mfs
        }
        self.rules: Dict[Tuple[str, str], str] = {
            ("Robust", "Smooth"): "Fast",
            ("Robust", "Moderate"): "Fast",
            ("Robust", "Rough"): "Medium",
            ("Moderate", "Smooth"): "Fast",
            ("Moderate", "Moderate"): "Medium",
            ("Moderate", "Rough"): "Slow",
            ("Delicate", "Smooth"): "Medium",
            ("Delicate", "Moderate"): "Slow",
            ("Delicate", "Rough"): "Slow",
        }
        self._cache: Dict[Tuple[float, float], float] = {}

    def infer(self, fragility: float, bumpiness: float) -> dict:
        fragility = _finite_or_default(fragility, 0.0)
        bumpiness = _finite_or_default(bumpiness, 0.0)
        f_membership = self.fragility.fuzzify(fragility)
        b_membership = self.bumpiness.fuzzify(bumpiness)
        activations = []
        numerator = 0.0
        denominator = 0.0

        for (frag_label, bump_label), speed_label in self.rules.items():
            alpha = min(f_membership[frag_label], b_membership[bump_label])
            if alpha > 0:
                activations.append({
                    "rule": f"{frag_label} & {bump_label}",
                    "consequent": speed_label,
                    "alpha": round(alpha, 4),
                })
                numerator += alpha * self.cons_centroid[speed_label]
                denominator += alpha

        crisp = numerator / denominator if denominator else float(np.mean(list(self.cons_centroid.values())))
        crisp = float(np.clip(crisp, 40.0, 100.0))
        return {
            "fragility_mu": f_membership,
            "bumpiness_mu": b_membership,
            "activations": activations,
            "crisp": crisp,
            "cons_centroid": self.cons_centroid,
        }

    def safe_speed(self, fragility: float, bumpiness: float) -> float:
        fragility = _finite_or_default(fragility, 0.0)
        bumpiness = _finite_or_default(bumpiness, 0.0)
        key = (round(fragility, 4), round(bumpiness, 4))
        if key not in self._cache:
            self._cache[key] = self.infer(*key)["crisp"]
        return self._cache[key]

    def monotonicity_violations(self, step: float = 0.5, tolerance: float = 1e-9) -> dict[str, int]:
        values = np.round(np.arange(0, 10 + step / 2, step), 4)
        surface = np.array([[self.safe_speed(fragility, bumpiness) for bumpiness in values] for fragility in values])
        fragility_violations = int((np.diff(surface, axis=0) > tolerance).sum())
        bumpiness_violations = int((np.diff(surface, axis=1) > tolerance).sum())
        return {"fragility": fragility_violations, "bumpiness": bumpiness_violations}
