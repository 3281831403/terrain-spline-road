"""terrain-spline-road 的评测测试。

初始状态下 road/spline.py 的 8 个函数全部是桩，因此本文件中的 15 个
测试都会因 NotImplementedError 而失败；测试本身必须保持不变。
"""

import math

import pytest

from road.spline import (
    RoadError,
    build_ribbon,
    catmull_rom,
    cross_section,
    framing,
    join_segments,
    resample_by_length,
    road_batch,
    validate_road,
)

UP = (0.0, 0.0, 1.0)
EPS = 1e-6


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(v):
    return math.sqrt(_dot(v, v))


def _normalize(v):
    length = _norm(v)
    return tuple(c / length for c in v)


def _assert_point_close(got, want, eps=EPS):
    assert len(got) == len(want)
    assert math.dist(got, want) <= eps


# 缺陷 1：Catmull-Rom 不重复端点，首尾切线为 0。
def test_catmull_rom_duplicates_endpoints():
    points = [(0.0, 0.0, 0.0), (2.0, 1.0, 0.0), (4.0, -1.0, 1.0), (6.0, 0.0, 0.0)]
    samples = 6
    curve = catmull_rom(points, samples)

    assert len(curve) == (len(points) - 1) * samples + 1
    _assert_point_close(curve[0], points[0], 1e-9)
    _assert_point_close(curve[-1], points[-1], 1e-9)
    # 端点切线不能退化为零：首尾相邻输出点必须有实际距离。
    assert math.dist(curve[0], curve[1]) > EPS
    assert math.dist(curve[-1], curve[-2]) > EPS


# 缺陷 2：重采样按段数等分而不是弧长，弯道处变密。
def test_resample_uses_arc_length():
    points = [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0), (4.0, 3.0, 0.0)]
    sampled = resample_by_length(points, 1.0)
    expected = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 1.0, 0.0),
        (4.0, 2.0, 0.0),
        (4.0, 3.0, 0.0),
    ]

    assert len(sampled) == 8
    for got, want in zip(sampled, expected):
        _assert_point_close(got, want)


# 缺陷 3：切线不归一，横截面宽度随速度变化。
def test_framing_tangents_are_normalized():
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.5, 0.1),
        (2.0, 2.0, 0.4),
        (4.0, 2.5, 1.2),
    ]
    frames = framing(points, UP)

    assert len(frames) == len(points)
    for frame in frames:
        tangent = frame["tangent"]
        normal = frame["normal"]
        assert abs(_norm(tangent) - 1.0) <= EPS
        assert abs(_dot(tangent, normal)) <= EPS


# 缺陷 4：副法线用切线叉 up 后未再归一，路面扭曲。
def test_framing_binormals_are_normalized():
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.8),
        (2.0, 1.0, 1.6),
        (3.0, 3.0, 2.0),
    ]
    frames = framing(points, UP)

    for frame in frames:
        tangent = frame["tangent"]
        normal = frame["normal"]
        binormal = frame["binormal"]
        assert abs(_norm(binormal) - 1.0) <= EPS
        assert abs(_dot(binormal, tangent)) <= EPS
        assert abs(_dot(binormal, normal)) <= EPS


# 缺陷 5：横截面车道数不校验，出现负值。
def test_cross_section_rejects_invalid_lane_count():
    point = (1.0, 2.0, 3.0)
    frame = {
        "tangent": (1.0, 0.0, 0.0),
        "normal": (0.0, 0.0, 1.0),
        "binormal": (0.0, 1.0, 0.0),
    }

    with pytest.raises(RoadError):
        cross_section(point, frame, 6.0, 0)
    with pytest.raises(RoadError):
        cross_section(point, frame, 6.0, -2)

    section = cross_section(point, frame, 6.0, 3)
    assert len(section) == 4
    assert abs(math.dist(section[0], section[-1]) - 6.0) <= EPS


# 缺陷 6：路面接缝重复顶点，产生 Z-fighting。
def test_build_ribbon_has_no_duplicate_seam_vertices():
    points = [
        (0.0, 0.0, 0.0),
        (2.0, 1.0, 0.2),
        (4.0, -1.0, 0.5),
        (6.0, 0.0, 0.3),
    ]
    lanes = 2
    mesh = build_ribbon(points, 6.0, lanes)

    assert len(mesh["vertices"]) == len(points) * (lanes + 1)
    keys = [tuple(round(c, 9) for c in vertex) for vertex in mesh["vertices"]]
    assert len(set(keys)) == len(keys)
    assert len(mesh["indices"]) == (len(points) - 1) * lanes * 6
    assert len(mesh["rings"]) == len(points)


# 缺陷 7：接缝法线不插值，出现硬边。
def test_join_segments_interpolates_seam_normals():
    a = {
        "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 1.0, 0.0)],
        "normals": [(0.0, 0.0, 1.0)] * 4,
        "uvs": [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)],
        "indices": [(0, 1, 2), (1, 3, 2)],
        "rings": [[0, 1], [2, 3]],
        "lanes": 1,
        "width": 1.0,
    }
    b = {
        "vertices": [(0.0, 1.0, 0.0), (1.0, 1.0, 0.0), (0.0, 2.0, 0.0), (1.0, 2.0, 0.0)],
        "normals": [(1.0, 0.0, 0.0)] * 4,
        "uvs": [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)],
        "indices": [(0, 1, 2), (1, 3, 2)],
        "rings": [[0, 1], [2, 3]],
        "lanes": 1,
        "width": 1.0,
    }

    merged = join_segments(a, b, EPS)

    # 接缝两个顶点各保留一份，b 的其余两个顶点接在后面。
    assert len(merged["vertices"]) == 6
    assert len(merged["indices"]) == 12
    assert len(merged["rings"]) == 3

    seam_positions = {(0.0, 1.0, 0.0), (1.0, 1.0, 0.0)}
    seam_indices = [
        index
        for index, vertex in enumerate(merged["vertices"])
        if tuple(vertex) in seam_positions
    ]
    assert len(seam_indices) == 2

    # 两侧接缝法线分别为 +Z 和 +X，融合后应为归一化的 (1, 0, 1)。
    expected = _normalize((1.0, 0.0, 1.0))
    for index in seam_indices:
        _assert_point_close(merged["normals"][index], expected)


# 缺陷 8：急弯不检测，路面自交。
def test_validate_road_detects_sharp_turn():
    points = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (5.0, 5.0, 0.0)]
    issues = validate_road(points, 0.5)

    assert isinstance(issues, list)
    assert any("急弯" in issue for issue in issues)


# 缺陷 9：批量共用顶点缓冲，道路之间串数据。
def test_road_batch_does_not_share_vertex_buffers():
    config = {
        "width": 6.0,
        "lanes": 2,
        "spacing": 1.0,
        "min_spacing": 0.5,
    }
    roads = [
        [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)],
        [(0.0, 20.0, 0.0), (4.0, 20.0, 0.0)],
    ]
    batch = road_batch(roads, config)

    assert batch["count"] == 2
    first, second = batch["roads"]
    assert first["vertices"] is not second["vertices"]
    assert first["normals"] is not second["normals"]
    assert set(first["vertices"]).isdisjoint(set(second["vertices"]))


# 缺陷 10：spacing=0 不校验，死循环。
def test_zero_spacing_is_rejected():
    with pytest.raises(RoadError):
        resample_by_length([(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)], 0.0)


# 组合 1：直线道路按弧长重采样点数正确。
def test_straight_road_resample_point_count():
    sampled = resample_by_length([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)], 2.0)
    expected = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (4.0, 0.0, 0.0),
                (6.0, 0.0, 0.0), (8.0, 0.0, 0.0), (10.0, 0.0, 0.0)]

    assert len(sampled) == 6
    for got, want in zip(sampled, expected):
        _assert_point_close(got, want)


# 组合 2：spacing < 0 也必须抛 RoadError。
def test_negative_spacing_raises_road_error():
    with pytest.raises(RoadError):
        resample_by_length([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)], -2.0)


# 组合 3：只有一段路时没有额外接缝顶点。
def test_single_segment_road_has_no_seam():
    points = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    mesh = build_ribbon(points, 6.0, 2)

    assert len(mesh["rings"]) == 2
    assert len(mesh["vertices"]) == 6
    assert len(mesh["indices"]) == 12
    assert len(set(mesh["vertices"])) == 6


# 组合 4：空控制点返回空结果。
def test_empty_control_points_return_empty():
    assert catmull_rom([], 8) == []
    assert resample_by_length([], 1.0) == []
    assert framing([], UP) == []
    assert validate_road([], 1.0) == []

    mesh = build_ribbon([], 6.0, 2)
    assert mesh["vertices"] == []
    assert mesh["normals"] == []
    assert mesh["uvs"] == []
    assert mesh["indices"] == []
    assert mesh["rings"] == []


# 组合 5：批量全部成功时保持输入顺序。
def test_road_batch_preserves_input_order_on_success():
    config = {
        "width": 6.0,
        "lanes": 2,
        "spacing": 1.0,
        "min_spacing": 0.5,
    }
    roads = [
        [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)],
        [(10.0, 0.0, 0.0), (14.0, 0.0, 0.0)],
        [(20.0, 0.0, 0.0), (24.0, 0.0, 0.0)],
    ]
    batch = road_batch(roads, config)

    assert batch["ok"] is True
    assert batch["count"] == 3
    centers = [
        sum(vertex[0] for vertex in mesh["vertices"]) / len(mesh["vertices"])
        for mesh in batch["roads"]
    ]
    assert centers == sorted(centers)
