"""Distance-field statistics and masked Laplacian used by the local GUI."""
from __future__ import annotations

from collections import deque
from typing import Any
import numpy as np

SENTINEL_LIMIT = 1.0e300


def finite_free_mask(distance: np.ndarray, occupancy: np.ndarray) -> np.ndarray:
    if distance.shape != occupancy.shape:
        raise ValueError("distance and occupancy shapes differ")
    return (
        (occupancy >= 0.5)
        & np.isfinite(distance)
        & (np.abs(distance) < SENTINEL_LIMIT)
    )


def five_point_laplacian(
    distance: np.ndarray, occupancy: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    valid = finite_free_mask(distance, occupancy)
    stencil = np.zeros_like(valid)
    stencil[1:-1, 1:-1] = (
        valid[1:-1, 1:-1]
        & valid[:-2, 1:-1]
        & valid[2:, 1:-1]
        & valid[1:-1, :-2]
        & valid[1:-1, 2:]
    )
    laplacian = np.full(distance.shape, np.nan, dtype=np.float64)
    with np.errstate(invalid="ignore", over="ignore"):
        values = (
            distance[:-2, 1:-1]
            + distance[2:, 1:-1]
            + distance[1:-1, :-2]
            + distance[1:-1, 2:]
            - 4.0 * distance[1:-1, 1:-1]
        )
    inner = laplacian[1:-1, 1:-1]
    inner_mask = stencil[1:-1, 1:-1]
    inner[inner_mask] = values[inner_mask]
    return laplacian, stencil


def connected_component_sizes(mask: np.ndarray) -> list[int]:
    if mask.ndim != 2:
        raise ValueError("component mask must be two-dimensional")
    visited = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    sizes: list[int] = []
    for y, x in np.argwhere(mask):
        if visited[y, x]:
            continue
        visited[y, x] = True
        queue: deque[tuple[int, int]] = deque([(int(y), int(x))])
        size = 0
        while queue:
            cy, cx = queue.popleft()
            size += 1
            for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                if (
                    0 <= ny < height
                    and 0 <= nx < width
                    and mask[ny, nx]
                    and not visited[ny, nx]
                ):
                    visited[ny, nx] = True
                    queue.append((ny, nx))
        sizes.append(size)
    return sorted(sizes, reverse=True)


def percentile(values: np.ndarray, level: float) -> float | None:
    if values.size == 0:
        return None
    return float(np.percentile(values, level))


def field_metrics(
    distance: np.ndarray,
    occupancy: np.ndarray,
    *,
    ridge_threshold: float,
) -> dict[str, Any]:
    free = occupancy >= 0.5
    occupied = ~free
    finite_free = finite_free_mask(distance, occupancy)
    missing = free & ~finite_free
    missing_components = connected_component_sizes(missing)

    laplacian, stencil = five_point_laplacian(distance, occupancy)
    absolute_laplacian = np.abs(laplacian[stencil])
    ridge = stencil & (np.abs(laplacian) >= ridge_threshold)
    ridge_components = connected_component_sizes(ridge)

    valid_values = distance[finite_free]
    horizontal_mask = finite_free[:, 1:] & finite_free[:, :-1]
    vertical_mask = finite_free[1:, :] & finite_free[:-1, :]
    adjacent_jumps = np.concatenate(
        (
            np.abs(distance[:, 1:] - distance[:, :-1])[horizontal_mask],
            np.abs(distance[1:, :] - distance[:-1, :])[vertical_mask],
        )
    )

    if absolute_laplacian.size:
        max_flat = int(np.nanargmax(np.abs(laplacian)))
        max_y, max_x = np.unravel_index(max_flat, laplacian.shape)
        laplacian_max = float(np.abs(laplacian[max_y, max_x]))
    else:
        max_x = max_y = -1
        laplacian_max = None

    return {
        "width": int(distance.shape[1]),
        "height": int(distance.shape[0]),
        "total_cells": int(distance.size),
        "free_cells": int(np.count_nonzero(free)),
        "occupied_cells": int(np.count_nonzero(occupied)),
        "finite_free_cells": int(np.count_nonzero(finite_free)),
        "missing_free_cells": int(np.count_nonzero(missing)),
        "missing_component_count": len(missing_components),
        "largest_missing_component": missing_components[0] if missing_components else 0,
        "largest_missing_components": missing_components[:20],
        "free_nan_cells": int(np.count_nonzero(free & np.isnan(distance))),
        "free_infinite_or_sentinel_cells": int(
            np.count_nonzero(free & (~np.isfinite(distance) | (np.abs(distance) >= SENTINEL_LIMIT)))
        ),
        "finite_distance_min": float(np.min(valid_values)) if valid_values.size else None,
        "finite_distance_max": float(np.max(valid_values)) if valid_values.size else None,
        "laplacian_stencil_cells": int(absolute_laplacian.size),
        "mean_abs_laplacian": (
            float(np.mean(absolute_laplacian)) if absolute_laplacian.size else None
        ),
        "p95_abs_laplacian": percentile(absolute_laplacian, 95.0),
        "p99_abs_laplacian": percentile(absolute_laplacian, 99.0),
        "max_abs_laplacian": laplacian_max,
        "max_abs_laplacian_x": int(max_x),
        "max_abs_laplacian_y": int(max_y),
        "ridge_threshold": float(ridge_threshold),
        "ridge_cells": int(np.count_nonzero(ridge)),
        "ridge_component_count": len(ridge_components),
        "largest_ridge_component": ridge_components[0] if ridge_components else 0,
        "mean_adjacent_jump": (
            float(np.mean(adjacent_jumps)) if adjacent_jumps.size else None
        ),
        "p95_adjacent_jump": percentile(adjacent_jumps, 95.0),
        "p99_adjacent_jump": percentile(adjacent_jumps, 99.0),
        "max_adjacent_jump": (
            float(np.max(adjacent_jumps)) if adjacent_jumps.size else None
        ),
    }
