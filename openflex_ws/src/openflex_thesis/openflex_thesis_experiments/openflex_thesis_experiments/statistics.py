"""Dependency-light statistics used by the frozen experiment report."""

import math
from typing import Iterable, List, Sequence, Tuple


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> Tuple[float, float]:
    if trials <= 0:
        return 0.0, 0.0
    successes = max(0, min(trials, successes))
    proportion = successes / trials
    denominator = 1.0 + z * z / trials
    center = (proportion + z * z / (2.0 * trials)) / denominator
    radius = z * math.sqrt(
        proportion * (1.0 - proportion) / trials + z * z / (4.0 * trials * trials)
    ) / denominator
    return max(0.0, center - radius), min(1.0, center + radius)


def _average_ranks(values: Sequence[float]) -> List[float]:
    indexed = sorted(enumerate(values), key=lambda item: item[1])
    ranks = [0.0] * len(values)
    index = 0
    while index < len(indexed):
        end = index + 1
        while end < len(indexed) and indexed[end][1] == indexed[index][1]:
            end += 1
        rank = 0.5 * ((index + 1) + end)
        for cursor in range(index, end):
            ranks[indexed[cursor][0]] = rank
        index = end
    return ranks


def spearman_correlation(first: Sequence[float], second: Sequence[float]) -> float:
    if len(first) != len(second) or len(first) < 2:
        raise ValueError("equal sequences with at least two values are required")
    rank_first = _average_ranks(first)
    rank_second = _average_ranks(second)
    mean_first = sum(rank_first) / len(rank_first)
    mean_second = sum(rank_second) / len(rank_second)
    numerator = sum(
        (a - mean_first) * (b - mean_second)
        for a, b in zip(rank_first, rank_second)
    )
    denominator = math.sqrt(
        sum((a - mean_first) ** 2 for a in rank_first)
        * sum((b - mean_second) ** 2 for b in rank_second)
    )
    return numerator / denominator if denominator > 0.0 else 0.0
