from openflex_thesis_experiments.statistics import spearman_correlation, wilson_interval


def test_wilson_interval_contains_observed_rate():
    low, high = wilson_interval(8, 10)
    assert low < 0.8 < high


def test_spearman_detects_monotonic_order():
    assert spearman_correlation([0.1, 0.2, 0.3], [0.0, 0.5, 1.0]) == 1.0
