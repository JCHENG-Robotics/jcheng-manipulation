"""Markerless floor-cube localization for G8 task takeover.

A pure-NumPy port of ``tools/phase2/object_segment_rgbd.py``. The registered
point cloud is in the Aurora's OpenCV **optical** layout (even though the
driver labels it ``depth_camera_link``); the validated hand-eye
``T_link4_camera`` maps it to ``link4``, and the measured joint pulses give
``base_link -> link4``. Nothing here moves the robot.

The algorithm is unchanged from the validated 2026-11-19/21 pipeline:

1. RANSAC the support plane on the raw cloud and refine it with SVD;
2. remove the floor and fit the plane in ``base_link``;
3. voxel-grid Euclidean components on the points above the floor;
4. select the component in the expected region/height/size band;
5. take the highest slab (the top face) and require a full enough extent.

The differences are only mechanical: the cloud arrives from the gateway as
float32 XYZ triples, the FK is the backend's audited URDF FK, and the report is
plain JSON.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

DEFAULT_MIN_TOP_EXTENT_M = 0.035


@dataclass(frozen=True)
class LocateParams:
    min_range_m: float = 0.15
    max_range_m: float = 0.60
    floor_threshold_m: float = 0.004
    floor_margin_m: float = 0.006
    voxel_m: float = 0.005
    min_cluster_points: int = 30
    slab_m: float = 0.005
    min_top_extent_m: float = DEFAULT_MIN_TOP_EXTENT_M
    ransac_iterations: int = 800
    ransac_points: int = 40000
    x_range_m: tuple[float, float] = (0.15, 0.32)
    y_range_m: float = 0.15
    max_xy_m: float = 0.09
    min_height_m: float = 0.020
    max_height_m: float = 0.070
    seed: int = 0
    # Min-range clip recovery: the Aurora reports no valid depth nearer than
    # ~0.25 m at the view pose, so a cube inside that sphere loses the near
    # part of its top face. When the far edge and the two side edges are
    # intact, the centre can be rebuilt from the far edge with the measured
    # cube size. ``cube_size_m <= 0`` disables the recovery.
    cube_size_m: float = 0.0
    recovery_min_visible_m: float = 0.012
    recovery_intact_extent_m: float = 0.030
    recovery_edge_tol_m: float = 0.002
    recovery_range_band_m: float = 0.0025
    recovery_range_std_m: float = 0.0015
    recovery_radial_min: float = 0.85

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "min_range_m": self.min_range_m,
            "max_range_m": self.max_range_m,
            "floor_threshold_m": self.floor_threshold_m,
            "floor_margin_m": self.floor_margin_m,
            "voxel_m": self.voxel_m,
            "min_cluster_points": self.min_cluster_points,
            "slab_m": self.slab_m,
            "min_top_extent_m": self.min_top_extent_m,
            "ransac_iterations": self.ransac_iterations,
            "ransac_points": self.ransac_points,
            "x_range_m": list(self.x_range_m),
            "y_range_m": self.y_range_m,
            "max_xy_m": self.max_xy_m,
            "min_height_m": self.min_height_m,
            "max_height_m": self.max_height_m,
            "seed": self.seed,
            "cube_size_m": self.cube_size_m,
            "recovery_min_visible_m": self.recovery_min_visible_m,
            "recovery_intact_extent_m": self.recovery_intact_extent_m,
            "recovery_edge_tol_m": self.recovery_edge_tol_m,
            "recovery_range_band_m": self.recovery_range_band_m,
            "recovery_range_std_m": self.recovery_range_std_m,
            "recovery_radial_min": self.recovery_radial_min,
        }


def _voxel_labels(points: np.ndarray, pitch: float) -> np.ndarray:
    return np.floor(points / pitch).astype(np.int64)


def connected_components(
    points: np.ndarray,
    pitch: float,
    min_points: int = 20,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Euclidean components on a voxel grid with 26-neighbour connectivity."""
    if len(points) == 0:
        return np.full(0, -1, dtype=np.int64), []
    voxels = _voxel_labels(points, pitch)
    keyed: dict[tuple[int, int, int], list[int]] = {}
    for index, voxel in enumerate(map(tuple, voxels)):
        keyed.setdefault(voxel, []).append(index)

    parent = {voxel: voxel for voxel in keyed}

    def find(item: tuple[int, int, int]) -> tuple[int, int, int]:
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    def union(a: tuple[int, int, int], b: tuple[int, int, int]) -> None:
        root_a, root_b = find(a), find(b)
        if root_a != root_b:
            parent[root_b] = root_a

    neighbours = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    for voxel in keyed:
        for delta in neighbours:
            other = (voxel[0] + delta[0], voxel[1] + delta[1], voxel[2] + delta[2])
            if other in keyed:
                union(voxel, other)

    groups: dict[tuple[int, int, int], list[int]] = {}
    for voxel in keyed:
        groups.setdefault(find(voxel), []).extend(keyed[voxel])

    clusters: list[dict[str, Any]] = []
    labels = np.full(len(points), -1, dtype=np.int64)
    for number, (_, indices) in enumerate(
        sorted(groups.items(), key=lambda item: -len(item[1]))
    ):
        if len(indices) < min_points:
            continue
        index_array = np.array(sorted(indices), dtype=np.int64)
        clusters.append(
            {
                "label": number,
                "indices": index_array,
                "points": points[index_array],
            }
        )
        labels[index_array] = number
    return labels, clusters


def min_area_rectangle(
    xy: np.ndarray, step_deg: float = 1.0
) -> tuple[float, float, float, float]:
    """Return ``(yaw_rad, width_m, depth_m, area_m2)`` for an XY set."""
    if len(xy) == 0:
        return 0.0, 0.0, 0.0, 0.0
    angles = np.deg2rad(np.arange(0.0, 90.0 + step_deg, step_deg))
    cosine = np.cos(angles)[:, None]
    sine = np.sin(angles)[:, None]
    rotated_x = xy[None, :, 0] * cosine - xy[None, :, 1] * sine
    rotated_y = xy[None, :, 0] * sine + xy[None, :, 1] * cosine
    widths = rotated_x.max(axis=1) - rotated_x.min(axis=1)
    depths = rotated_y.max(axis=1) - rotated_y.min(axis=1)
    areas = widths * depths
    best = int(np.argmin(areas))
    return (
        float(angles[best]),
        float(widths[best]),
        float(depths[best]),
        float(areas[best]),
    )


def fit_slab(points: np.ndarray, slab_m: float = 0.005) -> dict[str, Any]:
    """Fit the highest slab (top face) of a cluster."""
    if len(points) == 0:
        raise ValueError("empty cluster")
    z_max = float(points[:, 2].max())
    slab = points[points[:, 2] >= z_max - slab_m]
    centroid = slab.mean(axis=0)
    centred = slab - centroid
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))
    yaw, width, depth, area = min_area_rectangle(slab[:, :2])
    return {
        "slab_points": int(len(slab)),
        "slab_thickness_m": slab_m,
        "centroid": [float(value) for value in centroid],
        "normal": [float(value) for value in normal],
        "tilt_deg": tilt_deg,
        "yaw_rad": yaw,
        "extent_m": [width, depth],
        "area_m2": area,
        "z_max": z_max,
    }


def top_view_gate(
    top: dict[str, Any], min_extent_m: float
) -> tuple[bool, str | None, float]:
    """Reject a top face clipped by the image border."""
    extent = min(float(value) for value in top["extent_m"])
    if extent < min_extent_m:
        return False, "partial_top_view", extent
    return True, None, extent


def describe_cluster(points: np.ndarray) -> dict[str, Any]:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    return {
        "points": int(len(points)),
        "min": [float(value) for value in minimum],
        "max": [float(value) for value in maximum],
        "size": [float(value) for value in (maximum - minimum)],
        "centroid": [float(value) for value in points.mean(axis=0)],
    }


def select_cluster(
    clusters: list[dict[str, Any]],
    floor_z: float | None,
    gates: dict[str, Any],
) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
    """Pick the component that looks like the expected floor cube."""
    reasons = []
    best = None
    for cluster in clusters:
        points = cluster["points"]
        bounds = describe_cluster(points)
        size = bounds["size"]
        centroid = bounds["centroid"]
        height = (
            bounds["max"][2] - floor_z if floor_z is not None else float("nan")
        )
        ok = True
        why = []
        if len(points) < gates["min_points"]:
            ok = False
            why.append("too_few_points")
        if max(size[0], size[1]) > gates["max_xy_m"]:
            ok = False
            why.append("too_wide")
        if not gates["min_height_m"] <= height <= gates["max_height_m"]:
            ok = False
            why.append("height_%.3f" % height)
        if not gates["x_range"][0] <= centroid[0] <= gates["x_range"][1]:
            ok = False
            why.append("x_out_of_region")
        if abs(centroid[1]) > gates["y_range"]:
            ok = False
            why.append("y_out_of_region")
        reasons.append(
            {
                "label": cluster["label"],
                "bounds": bounds,
                "height_m": float(height),
                "ok": ok,
                "why": why,
            }
        )
        if ok and (best is None or len(points) > len(best["points"])):
            best = cluster
    return best, reasons


def ransac_plane(
    points: np.ndarray,
    threshold: float,
    iterations: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, float, np.ndarray, int]:
    """Return ``(normal, offset, inlier_mask, count)`` for the floor plane."""
    if len(points) < 3:
        raise ValueError("not enough points for a plane fit")
    best = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -float(normal @ sample[0])
        distances = np.abs(points @ normal + offset)
        mask = distances < threshold
        count = int(mask.sum())
        if best is None or count > best[3]:
            best = (normal, offset, mask, count)
    if best is None:
        raise ValueError("RANSAC found no plane")
    return best


def plane_from_points(
    points: np.ndarray,
) -> tuple[np.ndarray, float, float]:
    """Return ``(normal, offset, rms)`` for the SVD plane of the points."""
    centroid = points.mean(axis=0)
    centred = points - centroid
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = vh[-1]
    offset = -float(normal @ centroid)
    distances = points @ normal + offset
    rms = float(np.sqrt(np.mean(distances**2)))
    return normal, offset, rms


def plane_z_at(
    normal: np.ndarray, offset: float, x: float, y: float
) -> float | None:
    if abs(normal[2]) < 1e-9:
        return None
    return -(normal[0] * x + normal[1] * y + offset) / normal[2]


def recover_clipped_top(
    points_base: np.ndarray,
    top: dict[str, Any],
    T_base_cloud: np.ndarray,
    params: LocateParams,
    *,
    min_valid_range_m: float,
) -> dict[str, Any] | None:
    """Rebuild the cube centre when the min-range sphere clips the near edge.

    The visible top is the part of the face outside the sensor's minimum range
    (a sphere around the camera). That boundary is an arc, not a cube edge. If
    exactly one side of the top's minimum-area rectangle lies on that sphere
    while the opposite edge and both side edges are intact, the cube centre is
    ``far_edge -/+ cube_size/2`` along the clipped axis and the midpoint along
    the intact axis. Returns ``None`` whenever those conditions do not hold.
    """
    if params.cube_size_m <= 0 or min_valid_range_m <= 0:
        return None
    z_max = float(top["z_max"])
    slab = points_base[points_base[:, 2] >= z_max - params.slab_m]
    if len(slab) < 20:
        return None
    yaw = float(top["yaw_rad"])
    axis_a = np.array([math.cos(yaw), math.sin(yaw)])
    axis_b = np.array([-math.sin(yaw), math.cos(yaw)])
    a = slab[:, :2] @ axis_a
    b = slab[:, :2] @ axis_b
    rotation = np.asarray(T_base_cloud, dtype=np.float64)[:3, :3]
    translation = np.asarray(T_base_cloud, dtype=np.float64)[:3, 3]
    camera = (slab - translation) @ rotation
    ranges = np.linalg.norm(camera, axis=1)
    tol = params.recovery_edge_tol_m
    camera_xy = translation[:2]

    sides: dict[str, dict[str, Any]] = {}
    for name, values, high, normal in (
        ("a_min", a, False, axis_a),
        ("a_max", a, True, -axis_a),
        ("b_min", b, False, axis_b),
        ("b_max", b, True, -axis_b),
    ):
        edge = (
            values >= values.max() - tol if high else values <= values.min() + tol
        )
        mean = float(ranges[edge].mean())
        std = float(ranges[edge].std())
        # The min-range cut is a sphere around the camera, so its boundary in
        # the horizontal top plane is perpendicular to the camera->edge
        # direction. A cube edge at an arbitrary yaw is not.
        edge_xy = slab[edge][:, :2].mean(axis=0)
        radial = edge_xy - camera_xy
        radial_norm = float(np.linalg.norm(radial))
        alignment = (
            abs(float(radial @ normal)) / radial_norm
            if radial_norm > 1e-9
            else 0.0
        )
        sides[name] = {
            "mean_m": mean,
            "std_m": std,
            "alignment": alignment,
            "clipped": std <= params.recovery_range_std_m
            and mean <= min_valid_range_m + params.recovery_range_band_m
            and alignment >= params.recovery_radial_min,
        }
    clipped = [name for name, info in sides.items() if info["clipped"]]
    if len(clipped) != 1:
        return None
    side = clipped[0]
    extent_a = float(a.max() - a.min())
    extent_b = float(b.max() - b.min())
    if side.startswith("a"):
        if extent_b < params.recovery_intact_extent_m:
            return None
        if extent_a < params.recovery_min_visible_m:
            return None
        centre_a = (
            float(a.max()) - params.cube_size_m / 2.0
            if side == "a_min"
            else float(a.min()) + params.cube_size_m / 2.0
        )
        centre_b = float((b.min() + b.max()) / 2.0)
    else:
        if extent_a < params.recovery_intact_extent_m:
            return None
        if extent_b < params.recovery_min_visible_m:
            return None
        centre_b = (
            float(b.max()) - params.cube_size_m / 2.0
            if side == "b_min"
            else float(b.min()) + params.cube_size_m / 2.0
        )
        centre_a = float((a.min() + a.max()) / 2.0)
    centre = centre_a * axis_a + centre_b * axis_b
    point = np.array([centre[0], centre[1], float(np.median(slab[:, 2]))])
    centroid = np.asarray(top["centroid"], dtype=np.float64)
    return {
        "reason": "min_range_clip",
        "clipped_side": side,
        "cube_size_m": params.cube_size_m,
        "visible_extent_m": extent_a if side.startswith("a") else extent_b,
        "intact_extent_m": extent_b if side.startswith("a") else extent_a,
        "edge_range_mean_m": sides[side]["mean_m"],
        "edge_range_std_m": sides[side]["std_m"],
        "min_valid_range_m": min_valid_range_m,
        "centroid_bias_mm": float(np.linalg.norm(point - centroid) * 1000.0),
        "top_centroid": [float(value) for value in centroid],
        "sides": {
            name: {
                "mean_m": round(info["mean_m"], 5),
                "std_m": round(info["std_m"], 5),
                "alignment": round(info["alignment"], 3),
                "clipped": bool(info["clipped"]),
            }
            for name, info in sides.items()
        },
        "centroid": [float(value) for value in point],
    }


def segment_floor_cube(
    points: np.ndarray,
    T_base_cloud: np.ndarray,
    params: LocateParams,
    *,
    cloud_meta: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Segment one floor cube; return the phase-3 report shape.

    ``points`` is the raw cloud in the camera optical frame; ``T_base_cloud``
    is ``T_base_link4 @ T_link4_camera``. The returned report includes
    ``selected.grasp_point_base`` (or ``rejected_point_base``) and the
    cluster details used by the grip-verification box.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(points) == 0:
        return {
            "schema_version": 1,
            "mode": "markerless_rgbd_segment",
            "success": False,
            "error": "empty point cloud",
            "segmentation": {"clusters": 0, "cluster_details": []},
        }
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    ranges = np.linalg.norm(points, axis=1)
    keep = (ranges >= params.min_range_m) & (ranges <= params.max_range_m)
    points = points[keep]
    min_valid_range_m = float(np.linalg.norm(points, axis=1).min()) if len(points) else 0.0
    if len(points) < 50:
        return {
            "schema_version": 1,
            "mode": "markerless_rgbd_segment",
            "success": False,
            "error": "too few points in the configured range",
            "cloud": {
                "points_total": int(len(finite)),
                "points_in_range": int(len(points)),
                **(cloud_meta or {}),
            },
            "segmentation": {"clusters": 0, "cluster_details": []},
        }
    rng = np.random.default_rng(params.seed)
    subset = points
    if len(subset) > params.ransac_points:
        subset = subset[
            rng.choice(len(subset), params.ransac_points, replace=False)
        ]
    normal, offset, _, _ = ransac_plane(
        subset, params.floor_threshold_m, params.ransac_iterations, rng
    )
    distances = points @ normal + offset
    inliers = np.abs(distances) < params.floor_threshold_m
    normal, offset, rms = plane_from_points(points[inliers])

    T_base_cloud = np.asarray(T_base_cloud, dtype=np.float64).reshape(4, 4)
    rotation = T_base_cloud[:3, :3]
    translation = T_base_cloud[:3, 3]
    normal_base = rotation @ normal
    offset_base = offset - float(normal_base @ translation)
    if normal_base[2] < 0:
        normal_base = -normal_base
        offset_base = -offset_base

    points_base = (rotation @ points.T).T + translation
    signed = points_base @ normal_base + offset_base
    above = signed > params.floor_margin_m
    candidates = points_base[above]

    labels, clusters = connected_components(
        candidates, params.voxel_m, params.min_cluster_points
    )
    del labels
    floor_z = plane_z_at(normal_base, offset_base, 0.24, 0.0)
    gates = {
        "min_points": params.min_cluster_points,
        "max_xy_m": params.max_xy_m,
        "min_height_m": params.min_height_m,
        "max_height_m": params.max_height_m,
        "x_range": params.x_range_m,
        "y_range": params.y_range_m,
    }
    selected, reasons = select_cluster(clusters, floor_z, gates)

    report: dict[str, Any] = {
        "schema_version": 1,
        "mode": "markerless_rgbd_segment",
        "cloud": {
            "points_total": int(len(finite)),
            "points_in_range": int(len(points)),
            **(cloud_meta or {}),
        },
        "floor": {
            "normal_base": [float(value) for value in normal_base],
            "offset_base": float(offset_base),
            "rms_m": float(rms),
            "inliers": int(inliers.sum()),
            "z_at_0p24": None if floor_z is None else float(floor_z),
            "angle_to_z_deg": math.degrees(
                math.acos(max(-1.0, min(1.0, float(normal_base[2]))))
            ),
        },
        "segmentation": {
            "voxel_m": params.voxel_m,
            "points_above_floor": int(len(candidates)),
            "clusters": len(clusters),
            "cluster_details": reasons,
        },
        "success": False,
    }
    if selected is not None:
        top = fit_slab(selected["points"], params.slab_m)
        target = np.asarray(top["centroid"], dtype=np.float64)
        view_ok, view_reason, view_extent = top_view_gate(
            top, params.min_top_extent_m
        )
        report["selected"] = {
            "points": int(len(selected["points"])),
            "bounds": describe_cluster(selected["points"]),
            "top": top,
            "height_above_floor_m": (
                None
                if floor_z is None
                else float(target[2] - floor_z)
            ),
        }
        if view_ok:
            report["selected"]["grasp_point_base"] = [
                float(value) for value in target
            ]
            report["success"] = True
        else:
            recovered = recover_clipped_top(
                selected["points"],
                top,
                T_base_cloud,
                params,
                min_valid_range_m=min_valid_range_m,
            )
            if recovered is not None:
                report["selected"]["grasp_point_base"] = recovered["centroid"]
                report["selected"]["top_recovered"] = recovered
                report["success"] = True
            else:
                report["selected"]["rejected_point_base"] = [
                    float(value) for value in target
                ]
                report["rejection"] = {
                    "reason": view_reason,
                    "min_extent_m": view_extent,
                    "threshold_m": params.min_top_extent_m,
                }
    return report


def cube_point(
    report: dict[str, Any],
) -> tuple[np.ndarray | None, bool]:
    """Return the measured grasp point and whether it passed the top-view gate."""
    selected = report.get("selected") or {}
    if selected.get("grasp_point_base") is not None:
        return np.asarray(selected["grasp_point_base"], dtype=np.float64), True
    if selected.get("rejected_point_base") is not None:
        return np.asarray(selected["rejected_point_base"], dtype=np.float64), False
    return None, False


def cube_box_present(
    report: dict[str, Any],
    point: np.ndarray,
    radius: float = 0.04,
    min_height: float = 0.01,
) -> bool:
    """True when a cluster sits in the box around the expected cube."""
    point = np.asarray(point, dtype=np.float64)
    for cluster in report.get("segmentation", {}).get("cluster_details", []):
        centroid = np.asarray(cluster["bounds"]["centroid"])
        if (
            abs(centroid[0] - point[0]) <= radius
            and abs(centroid[1] - point[1]) <= radius
            and float(cluster.get("height_m", 0.0)) >= min_height
        ):
            return True
    return False
