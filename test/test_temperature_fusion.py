"""Unit tests for temperature_fusion strategies and data structures.

These tests exercise the pure-function parts of the module (strategies,
data structures, confidence formula) without booting a full Klipper runtime.
"""

from __future__ import annotations

import pytest

from klippy.extras.temperature_fusion import (
    CONFIDENCE_ALPHA,
    FusionResult,
    FusionStrategy,
    KalmanFusionStrategy,
    RobustMedianStrategy,
    SensorSample,
    WeightedMeanStrategy,
    _compute_confidence,
    _median,
)

######################################################################
# Helpers
######################################################################


def make_samples(temps, weights=None, channels=None):
    """Build a list of SensorSample from temperature values."""
    n = len(temps)
    if weights is None:
        weights = [1.0] * n
    if channels is None:
        channels = list(range(n))
    return [
        SensorSample(
            channel=channels[i],
            raw_value=int(temps[i] * 10),
            temperature=temps[i],
            weight=weights[i],
            is_valid=True,
            timestamp=0.0,
        )
        for i in range(n)
    ]


######################################################################
# Data structure tests
######################################################################


class TestSensorSample:
    def test_construction_defaults(self):
        s = SensorSample(channel=0, raw_value=250, temperature=25.0, weight=1.0)
        assert s.zone is None
        assert s.position is None
        assert s.is_valid is True
        assert s.timestamp == 0.0

    def test_full_construction(self):
        s = SensorSample(
            channel=3,
            raw_value=420,
            temperature=42.0,
            weight=2.5,
            zone="top",
            position=(1.0, 2.0, 3.0),
            is_valid=True,
            timestamp=123.45,
        )
        assert s.channel == 3
        assert s.raw_value == 420
        assert s.temperature == 42.0
        assert s.weight == 2.5
        assert s.zone == "top"
        assert s.position == (1.0, 2.0, 3.0)
        assert s.timestamp == 123.45


class TestFusionResult:
    def test_default_excluded(self):
        r = FusionResult(temperature=42.0, confidence=0.9, valid_samples=12)
        assert r.excluded_samples == []

    def test_full(self):
        r = FusionResult(
            temperature=42.0,
            confidence=0.9,
            valid_samples=10,
            excluded_samples=[{"channel": 5, "reason": "outlier"}],
        )
        assert len(r.excluded_samples) == 1


######################################################################
# Helper function tests
######################################################################


class TestMedian:
    def test_odd(self):
        assert _median([3, 1, 2]) == 2.0

    def test_even(self):
        assert _median([1, 2, 3, 4]) == 2.5

    def test_single(self):
        assert _median([42.0]) == 42.0

    def test_empty(self):
        assert _median([]) == 0.0


class TestComputeConfidence:
    def test_full_valid_high_consistency(self):
        c = _compute_confidence(12, 12, 1.0)
        assert c == pytest.approx(1.0)

    def test_zero_valid(self):
        c = _compute_confidence(0, 12, 1.0)
        # 0.7*0 + 0.3*1.0 = 0.3
        assert c == pytest.approx(0.3)

    def test_half_valid(self):
        c = _compute_confidence(6, 12, 1.0)
        # 0.7*0.5 + 0.3*1.0 = 0.65
        assert c == pytest.approx(0.65)

    def test_total_zero(self):
        assert _compute_confidence(0, 0, 1.0) == 0.0

    def test_negative_consistency_clamped(self):
        c = _compute_confidence(6, 12, -0.5)
        # consistency clamped to 0: 0.7*0.5 + 0.3*0 = 0.35
        assert c == pytest.approx(0.35)

    def test_alpha_weight(self):
        assert CONFIDENCE_ALPHA == 0.7


######################################################################
# WeightedMeanStrategy tests
######################################################################


class TestWeightedMeanStrategy:
    def setup_method(self):
        self.strategy = WeightedMeanStrategy({}, 12)

    def test_uniform(self):
        samples = make_samples([42.0] * 12)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert result.temperature == pytest.approx(42.0)
        assert result.valid_samples == 12
        assert result.excluded_samples == []
        assert result.confidence == pytest.approx(1.0)

    def test_single_outlier(self):
        temps = [42.0] * 11 + [50.0]
        samples = make_samples(temps)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        # The outlier should be excluded
        assert result.valid_samples < 12
        assert len(result.excluded_samples) >= 1
        # Mean should be closer to 42 than 50
        assert result.temperature == pytest.approx(42.0, abs=0.1)

    def test_multiple_outliers(self):
        temps = [42.0] * 10 + [55.0, 60.0]
        samples = make_samples(temps)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert result.valid_samples < 12
        assert len(result.excluded_samples) >= 1

    def test_all_same(self):
        samples = make_samples([42.0] * 12)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert result.temperature == pytest.approx(42.0)
        assert result.excluded_samples == []
        diag = self.strategy.get_diagnostics()
        assert diag["mad"] == 0.0

    def test_empty_samples(self):
        self.strategy.update([], 0.0)
        result = self.strategy.fuse()
        assert result.temperature == 0.0
        assert result.valid_samples == 0
        assert result.confidence == 0.0

    def test_weighted(self):
        temps = [40.0, 40.0, 50.0, 50.0]
        weights = [3.0, 3.0, 1.0, 1.0]
        samples = make_samples(temps, weights)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        # Weighted mean = (3*40 + 3*40 + 1*50 + 1*50) / 8 = 340/8 = 42.5
        assert result.temperature == pytest.approx(42.5, abs=0.5)

    def test_custom_zscore_threshold(self):
        strategy = WeightedMeanStrategy({"outlier_zscore": 100.0}, 12)
        temps = [42.0] * 11 + [50.0]
        samples = make_samples(temps)
        strategy.update(samples, 0.0)
        result = strategy.fuse()
        # With huge threshold, nothing should be excluded
        assert result.valid_samples == 12
        assert result.excluded_samples == []

    def test_reset(self):
        samples = make_samples([42.0] * 12)
        self.strategy.update(samples, 0.0)
        self.strategy.reset()
        diag = self.strategy.get_diagnostics()
        assert diag["mad"] == 0.0
        assert diag["weighted_mean"] == 0.0
        assert diag["z_scores"] == []

    def test_diagnostics_structure(self):
        samples = make_samples([42.0, 43.0, 41.0])
        self.strategy.update(samples, 0.0)
        diag = self.strategy.get_diagnostics()
        assert "excluded" in diag
        assert "mad" in diag
        assert "weighted_mean" in diag
        assert "z_scores" in diag
        assert len(diag["z_scores"]) == 3


######################################################################
# RobustMedianStrategy tests
######################################################################


class TestRobustMedianStrategy:
    def setup_method(self):
        self.strategy = RobustMedianStrategy({}, 12)

    def test_uniform(self):
        samples = make_samples([42.0] * 12)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert result.temperature == pytest.approx(42.0)
        assert result.valid_samples == 12
        assert result.confidence == pytest.approx(1.0)

    def test_single_outlier(self):
        temps = [
            41.0,
            41.5,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.5,
            43.0,
            55.0,
        ]
        samples = make_samples(temps)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert result.valid_samples < 12
        assert len(result.excluded_samples) >= 1
        assert result.temperature == pytest.approx(42.0, abs=1.0)

    def test_weighted_median(self):
        temps = [40.0, 41.0, 42.0, 43.0, 44.0]
        weights = [1.0, 1.0, 3.0, 1.0, 1.0]
        samples = make_samples(temps, weights)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        # Total weight = 7, half = 3.5. Cumulative:
        # 40(1) -> 41(2) -> 42(5 >= 3.5) -> median = 42
        assert result.temperature == pytest.approx(42.0)

    def test_empty_samples(self):
        self.strategy.update([], 0.0)
        result = self.strategy.fuse()
        assert result.temperature == 0.0
        assert result.valid_samples == 0

    def test_all_same(self):
        samples = make_samples([42.0] * 12)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert result.temperature == pytest.approx(42.0)
        diag = self.strategy.get_diagnostics()
        assert diag["iqr"] == 0.0

    def test_custom_iqr_multiplier(self):
        strategy = RobustMedianStrategy({"iqr_multiplier": 100.0}, 12)
        temps = [42.0] * 11 + [55.0]
        samples = make_samples(temps)
        strategy.update(samples, 0.0)
        result = strategy.fuse()
        # With huge multiplier, nothing excluded
        assert result.valid_samples == 12

    def test_reset(self):
        samples = make_samples([42.0] * 12)
        self.strategy.update(samples, 0.0)
        self.strategy.reset()
        diag = self.strategy.get_diagnostics()
        assert diag["q1"] == 0.0
        assert diag["q3"] == 0.0
        assert diag["weighted_median"] == 0.0

    def test_diagnostics_structure(self):
        samples = make_samples([40.0, 42.0, 44.0, 46.0])
        self.strategy.update(samples, 0.0)
        diag = self.strategy.get_diagnostics()
        assert "q1" in diag
        assert "q3" in diag
        assert "iqr" in diag
        assert "weighted_median" in diag


######################################################################
# KalmanFusionStrategy tests
######################################################################


class TestKalmanFusionStrategy:
    def setup_method(self):
        self.strategy = KalmanFusionStrategy({}, 4)

    def test_initialization_from_first_observation(self):
        samples = make_samples([42.0, 42.0, 42.0, 42.0])
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        # Should converge near 42 with uniform input
        assert result.temperature == pytest.approx(42.0, abs=0.5)

    def test_covariance_decrease(self):
        samples = make_samples([42.0, 42.0, 42.0, 42.0])
        self.strategy.update(samples, 0.0)
        diag1 = self.strategy.get_diagnostics()
        cov1 = diag1["covariance"]

        # Another update should reduce covariance further
        self.strategy.update(samples, 1.0)
        diag2 = self.strategy.get_diagnostics()
        cov2 = diag2["covariance"]
        assert (
            cov2 <= cov1 + self.strategy.q
        )  # P grows by Q in predict, then shrinks

    def test_noise_tracking(self):
        # Feed noisy samples; Kalman should track the mean
        temps = [42.0 + 0.5, 42.0 - 0.5, 42.0 + 0.3, 42.0 - 0.3]
        samples = make_samples(temps)
        self.strategy.update(samples, 0.0)
        result = self.strategy.fuse()
        assert abs(result.temperature - 42.0) < 1.0

    def test_empty_samples(self):
        self.strategy.update([], 0.0)
        result = self.strategy.fuse()
        assert result.temperature == 0.0
        assert result.valid_samples == 0

    def test_custom_params(self):
        strategy = KalmanFusionStrategy(
            {"q": 0.1, "r_default": 0.5, "init_p": 100.0}, 4
        )
        assert strategy.q == 0.1
        assert strategy.r_default == 0.5
        assert strategy.init_p == 100.0

    def test_reset(self):
        samples = make_samples([42.0, 42.0, 42.0, 42.0])
        self.strategy.update(samples, 0.0)
        self.strategy.reset()
        diag = self.strategy.get_diagnostics()
        assert diag["state_estimate"] is None
        assert diag["covariance"] == pytest.approx(self.strategy.init_p)

    def test_diagnostics_structure(self):
        samples = make_samples([42.0, 43.0, 41.0, 42.0])
        self.strategy.update(samples, 0.0)
        diag = self.strategy.get_diagnostics()
        assert "state_estimate" in diag
        assert "covariance" in diag
        assert "innovation" in diag
        assert "kalman_gains" in diag
        assert len(diag["innovation"]) == 4
        assert len(diag["kalman_gains"]) == 4

    def test_per_channel_noise(self):
        strategy = KalmanFusionStrategy({}, 4)
        strategy.set_noise_variances([0.1, 0.1, 0.1, 0.1])
        samples = make_samples([42.0, 42.0, 42.0, 42.0])
        strategy.update(samples, 0.0)
        result = strategy.fuse()
        assert result.temperature == pytest.approx(42.0, abs=0.5)

    def test_convergence_over_multiple_cycles(self):
        """Kalman should converge to the true value over multiple cycles."""
        strategy = KalmanFusionStrategy({"q": 0.01, "r_default": 0.1}, 4)
        for _ in range(20):
            samples = make_samples([42.0, 42.0, 42.0, 42.0])
            strategy.update(samples, 0.0)
            strategy.fuse()
        result = strategy.fuse()
        assert result.temperature == pytest.approx(42.0, abs=0.01)
        diag = strategy.get_diagnostics()
        # Covariance should have decreased significantly
        assert diag["covariance"] < strategy.init_p


######################################################################
# Strategy base class tests
######################################################################


class TestFusionStrategyBase:
    def test_not_implemented(self):
        strategy = FusionStrategy({}, 4)
        with pytest.raises(NotImplementedError):
            strategy.update([], 0.0)
        with pytest.raises(NotImplementedError):
            strategy.fuse()

    def test_get_diagnostics_default(self):
        strategy = FusionStrategy({}, 4)
        assert strategy.get_diagnostics() == {}

    def test_reset_default(self):
        strategy = FusionStrategy({}, 4)
        strategy.reset()  # should not raise
