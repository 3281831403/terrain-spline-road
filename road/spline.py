"""盘山公路样条生成核心。

坐标系与单位约定
----------------
* 右手坐标系，控制点与生成点均为 ``(x, y, z)`` 三元组。
* ``z`` 轴向上；默认的 ``up`` 为 ``(0.0, 0.0, 1.0)``。
* 所有长度、间距、宽度的单位都是米。
* 文档中未特别说明时，几何比较容差均为 ``1e-6``。
"""

import math


DEFAULT_UP = (0.0, 0.0, 1.0)
EPS = 1e-6
# 浮点弧长配对时使用的内部容差，小于公开的 1e-6 容差。
_EPS_AL = 1e-12


class RoadError(Exception):
    """道路参数或道路几何数据非法。"""


# ---------------------------------------------------------------------------
# 基础向量工具与入参校验
# ---------------------------------------------------------------------------
def _check_vec3(value, name="向量"):
    """校验并拷贝一个有限三维向量，非法时抛 :class:`RoadError`。"""
    if value is None:
        raise RoadError(f"{name} 不能为 None")
    try:
        if len(value) != 3:
            raise RoadError(f"{name} 必须是三维点/向量，实际维度: {len(value)}")
        coords = tuple(float(component) for component in value)
    except (TypeError, ValueError) as exc:
        raise RoadError(f"{name} 不是合法的三维向量: {value!r}") from exc
    if not all(math.isfinite(component) for component in coords):
        raise RoadError(f"{name} 含非有限坐标: {value!r}")
    return coords


def _check_positive_number(value, name):
    """校验严格正的有限数值。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RoadError(f"{name} 必须是数值，实际: {value!r}")
    if not math.isfinite(value) or value <= 0:
        raise RoadError(f"{name} 必须严格大于 0，实际: {value!r}")
    return float(value)


def _check_lanes(lanes):
    """车道数必须是 >= 1 的整数（布尔值不算整数）。"""
    if isinstance(lanes, bool) or not isinstance(lanes, int) or lanes < 1:
        raise RoadError(f"车道数必须是 >= 1 的整数，实际: {lanes!r}")
    return lanes


def _check_samples(samples):
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 1:
        raise RoadError(f"采样段数必须是 >= 1 的整数，实际: {samples!r}")
    return samples


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, factor):
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    return math.sqrt(_dot(a, a))


def _normalize(a):
    length = _norm(a)
    if length <= 0.0:
        raise RoadError(f"零向量无法归一化: {a!r}")
    return (a[0] / length, a[1] / length, a[2] / length)


def _lerp(a, b, factor):
    return (
        a[0] + (b[0] - a[0]) * factor,
        a[1] + (b[1] - a[1]) * factor,
        a[2] + (b[2] - a[2]) * factor,
    )


# ---------------------------------------------------------------------------
# 1. Catmull-Rom：首尾复制幻影端点，保证端点切线非零
# ---------------------------------------------------------------------------
def _catmull_rom_point(p0, p1, p2, p3, t):
    """均匀 Catmull-Rom 基函数，p1 -> p2 之间、参数 t ∈ [0, 1]。"""
    t2 = t * t
    t3 = t2 * t
    return tuple(
        0.5
        * (
            2.0 * p1[k]
            + (-p0[k] + p2[k]) * t
            + (2.0 * p0[k] - 5.0 * p1[k] + 4.0 * p2[k] - p3[k]) * t2
            + (-p0[k] + 3.0 * p1[k] - 3.0 * p2[k] + p3[k]) * t3
        )
        for k in range(3)
    )


def catmull_rom(points, samples):
    """用 Catmull-Rom 样条插值折线控制点。"""
    _check_samples(samples)
    if not points:
        return []
    pts = [_check_vec3(point, "控制点") for point in points]
    if len(pts) == 1:
        return [tuple(pts[0])]

    # 幻影端点：首尾控制点各复制一份，端点切线来自真实相邻点而非零向量。
    extended = [pts[0]] + pts + [pts[-1]]

    result = []
    for segment in range(len(pts) - 1):
        p0 = extended[segment]
        p1 = extended[segment + 1]
        p2 = extended[segment + 2]
        p3 = extended[segment + 3]
        for j in range(samples):
            if j == 0:
                # t=0 时直接取控制点，保证端点精确重合。
                result.append(tuple(p1))
            else:
                result.append(_catmull_rom_point(p0, p1, p2, p3, j / samples))
    # 末端点精确取最后一个控制点。
    result.append(tuple(pts[-1]))
    return result


# ---------------------------------------------------------------------------
# 2. 按弧长等距重采样，spacing <= 0 在任何循环之前被拒绝
# ---------------------------------------------------------------------------
def resample_by_length(points, spacing):
    """按折线弧长对道路中心线等距重采样。"""
    # 先校验 spacing，杜绝 spacing=0 进入死循环。
    if isinstance(spacing, bool) or not isinstance(spacing, (int, float)):
        raise RoadError(f"spacing 必须是数值，实际: {spacing!r}")
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise RoadError(f"spacing 必须严格大于 0，实际: {spacing!r}")

    if not points:
        return []
    pts = [_check_vec3(point, "控制点") for point in points]
    if len(pts) == 1:
        return [tuple(pts[0])]

    segment_lengths = [
        math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)
    ]
    total_length = sum(segment_lengths)
    if total_length <= 0.0:
        # 所有点重合：退化为单点。
        return [tuple(pts[0])]

    # 目标采样距离 0, spacing, 2*spacing, ...
    interval_count = int(math.floor(total_length / spacing + 1e-12))
    targets = [index * spacing for index in range(interval_count + 1)]
    remainder = total_length - targets[-1]
    if remainder > 1e-9:
        # 总弧长不是 spacing 的整数倍：补末端点，最后一段允许更短。
        targets.append(total_length)
    else:
        # 吸收浮点误差，末端严格落在终点上。
        targets[-1] = total_length

    sampled = []
    segment_index = 0
    segment_start = 0.0
    for target in targets:
        # 推进到包含目标距离的段（零长度段会被自然跳过）。
        while (
            segment_index < len(segment_lengths) - 1
            and segment_start + segment_lengths[segment_index]
            < target - _EPS_AL
        ):
            segment_start += segment_lengths[segment_index]
            segment_index += 1
        segment_length = segment_lengths[segment_index]
        if segment_length <= 0.0:
            sampled.append(tuple(pts[segment_index]))
            continue
        fraction = (target - segment_start) / segment_length
        fraction = min(1.0, max(0.0, fraction))
        sampled.append(_lerp(pts[segment_index], pts[segment_index + 1], fraction))
    return sampled


# ---------------------------------------------------------------------------
# 3/4. 单位正交切线标架：切线、法线、副法线全部归一化
# ---------------------------------------------------------------------------
_WORLD_AXES = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def _tangent_at(pts, index):
    """端点单侧差分、内部点中心差分；零差分时向邻近段搜索非零方向。"""
    n = len(pts)
    if index == 0:
        direction = _sub(pts[1], pts[0])
    elif index == n - 1:
        direction = _sub(pts[n - 1], pts[n - 2])
    else:
        direction = _sub(pts[index + 1], pts[index - 1])

    if _norm(direction) > 0.0:
        return _normalize(direction)

    # 退化：相邻点重合，向两侧搜索任意非零位移。
    for radius in range(1, n):
        for neighbor in (index - radius, index + radius):
            if 0 <= neighbor < n:
                candidate = _sub(pts[neighbor], pts[index])
                if _norm(candidate) > 0.0:
                    return _normalize(candidate)
    raise RoadError("中心线所有点重合，无法建立切线标架")


def _make_frame(tangent, up):
    """由单位切线与参考 up 构造单位正交 (tangent, normal, binormal)。"""
    tangent = _normalize(tangent)
    # 把 up 投影到切线垂面。
    projected = _sub(up, _scale(tangent, _dot(up, tangent)))
    if _norm(projected) <= 1e-12:
        # up 与切线平行：选与切线夹角最大的世界轴做确定性备用。
        fallback = min(_WORLD_AXES, key=lambda axis: abs(_dot(axis, tangent)))
        projected = _sub(fallback, _scale(tangent, _dot(fallback, tangent)))
    normal = _normalize(projected)
    # 叉乘后再次归一化，杜绝副法线长度漂移导致的路面扭曲。
    binormal = _normalize(_cross(tangent, normal))
    return {"tangent": tangent, "normal": normal, "binormal": binormal}


def framing(points, up):
    """为每个中心线点建立单位正交的切线标架。"""
    if not points:
        return []
    pts = [_check_vec3(point, "中心线点") for point in points]
    if len(pts) == 1:
        raise RoadError("单点无法建立切线标架，至少需要 2 个中心线点")

    reference_up = _check_vec3(up, "up")
    if _norm(reference_up) <= 0.0:
        raise RoadError("up 不能是零向量")
    reference_up = _normalize(reference_up)

    frames = []
    for index in range(len(pts)):
        tangent = _tangent_at(pts, index)
        frames.append(_make_frame(tangent, reference_up))
    return frames


# ---------------------------------------------------------------------------
# 5. 横截面：严格校验车道数与宽度，沿单位副法线等分
# ---------------------------------------------------------------------------
def cross_section(point, frame, width, lanes):
    """在中心线点处生成一条横截面顶点链（含全部车道分界）。"""
    center = _check_vec3(point, "横截面中心点")
    if not isinstance(frame, dict):
        raise RoadError("frame 必须是 framing 返回的标架字典")
    for key in ("tangent", "normal", "binormal"):
        if key not in frame:
            raise RoadError(f"frame 缺少必需向量: {key!r}")
        _check_vec3(frame[key], f"frame[{key!r}]")
    width = _check_positive_number(width, "路面宽度")
    lanes = _check_lanes(lanes)

    binormal = _normalize(_check_vec3(frame["binormal"], "frame['binormal']"))
    half_width = width / 2.0
    return [
        _add(center, _scale(binormal, -half_width + width * j / lanes))
        for j in range(lanes + 1)
    ]


# ---------------------------------------------------------------------------
# 6. 无缝路面网格：每圈只生成一次，接缝共享顶点索引而非重复顶点
# ---------------------------------------------------------------------------
def _build_ribbon(points, width, lanes, up):
    if not points:
        return {
            "vertices": [],
            "normals": [],
            "uvs": [],
            "indices": [],
            "rings": [],
            "lanes": lanes,
            "width": width,
        }

    pts = [_check_vec3(point, "中心线点") for point in points]
    width = _check_positive_number(width, "路面宽度")
    lanes = _check_lanes(lanes)
    reference_up = _normalize(_check_vec3(up, "up"))

    if len(pts) == 1:
        # 单点：给一个确定性标架，只产出一圈、无索引。
        tangent = min(_WORLD_AXES, key=lambda axis: abs(_dot(axis, reference_up)))
        tangent = _normalize(_sub(tangent, _scale(reference_up, _dot(tangent, reference_up))))
        frames = [_make_frame(tangent, reference_up)]
    else:
        frames = framing(pts, reference_up)

    stride = lanes + 1
    vertices = []
    normals = []
    uvs = []
    rings = []

    for ring_index, (center, frame) in enumerate(zip(pts, frames)):
        section = cross_section(center, frame, width, lanes)
        base = len(vertices)
        vertices.extend(section)
        # 同一圈共享平滑法线（单位向量），接缝处不再有硬边。
        normals.extend([frame["normal"]] * stride)
        uvs.extend((j / lanes, float(ring_index)) for j in range(stride))
        rings.append(list(range(base, base + stride)))

    indices = []
    for ring_index in range(len(pts) - 1):
        base_a = ring_index * stride
        base_b = base_a + stride
        for lane in range(lanes):
            a0 = base_a + lane
            a1 = a0 + 1
            b0 = base_b + lane
            b1 = b0 + 1
            # 相邻圈共享顶点索引，不复制接缝顶点；索引为扁平整数列表。
            indices.extend((a0, a1, b0))
            indices.extend((a1, b1, b0))

    return {
        "vertices": vertices,
        "normals": normals,
        "uvs": uvs,
        "indices": indices,
        "rings": rings,
        "lanes": lanes,
        "width": width,
    }


def build_ribbon(points, width, lanes):
    """沿中心线生成一条无缝路面网格。"""
    return _build_ribbon(points, width, lanes, DEFAULT_UP)


# ---------------------------------------------------------------------------
# 7. 网格融合：接缝顶点只保留一份，法线取两侧归一化平均
# ---------------------------------------------------------------------------
_REQUIRED_MESH_KEYS = ("vertices", "normals", "indices", "rings")


def join_segments(a, b, tolerance):
    """把两条 ``build_ribbon`` 网格在接缝处融合为一个网格。"""
    if not isinstance(a, dict) or not isinstance(b, dict):
        raise RoadError("融合输入必须是网格字典")
    for key in _REQUIRED_MESH_KEYS:
        if key not in a or key not in b:
            raise RoadError(f"网格缺少必需键: {key!r}")
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)):
        raise RoadError(f"tolerance 必须是数值，实际: {tolerance!r}")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise RoadError(f"tolerance 必须严格大于 0，实际: {tolerance!r}")

    a_vertices = a["vertices"]
    b_vertices = b["vertices"]
    a_normals = a["normals"]
    b_normals = b["normals"]
    if len(a_vertices) != len(a_normals) or len(b_vertices) != len(b_normals):
        raise RoadError("顶点数与法线数不一致")
    if not a["rings"] or not b["rings"]:
        raise RoadError("融合网格至少要各含一圈顶点")

    seam_a = a["rings"][-1]
    seam_b = b["rings"][0]
    if len(seam_a) != len(seam_b):
        raise RoadError(f"接缝顶点数不匹配: {len(seam_a)} != {len(seam_b)}")

    vertex_count_a = len(a_vertices)
    vertex_count_b = len(b_vertices)
    for ring in a["rings"] + b["rings"]:
        if any(not isinstance(i, int) or i < 0 for i in ring):
            raise RoadError("rings 含非法顶点索引")
    if max(i for ring in a["rings"] for i in ring) >= vertex_count_a:
        raise RoadError("网格 a 的 rings 索引越界")
    if max(i for ring in b["rings"] for i in ring) >= vertex_count_b:
        raise RoadError("网格 b 的 rings 索引越界")

    # 接缝两圈必须按顺序一一配对且落在容差内。
    for index_a, index_b in zip(seam_a, seam_b):
        if math.dist(a_vertices[index_a], b_vertices[index_b]) > tolerance:
            raise RoadError(
                f"接缝顶点 {index_a}/{index_b} 距离超过容差 {tolerance}"
            )

    # 只拷贝、不修改输入字典。
    merged_vertices = list(a_vertices)
    merged_normals = list(a_normals)
    remap = {}

    # 接缝：保留 a 侧顶点，法线融合为 normalize(n_a + n_b)。
    for index_a, index_b in zip(seam_a, seam_b):
        remap[index_b] = index_a
        blended = _add(a_normals[index_a], b_normals[index_b])
        if _norm(blended) <= 1e-12:
            # 两侧法线恰好反向：退化为保留 a 侧法线。
            blended = a_normals[index_a]
        merged_normals[index_a] = _normalize(blended)

    # b 的其余顶点按原顺序追加，分配新的全局索引。
    for index_b in range(vertex_count_b):
        if index_b in remap:
            continue
        remap[index_b] = len(merged_vertices)
        merged_vertices.append(b_vertices[index_b])
        merged_normals.append(b_normals[index_b])

    def _triangles(mesh_indices):
        """兼容扁平整数列表与三元组序列，统一按三元组迭代。"""
        if mesh_indices and isinstance(mesh_indices[0], (tuple, list)):
            for triangle in mesh_indices:
                if len(triangle) != 3:
                    raise RoadError("索引三元组必须恰好含 3 个顶点索引")
                yield tuple(triangle)
        else:
            if len(mesh_indices) % 3 != 0:
                raise RoadError("扁平索引列表长度必须是 3 的倍数")
            for offset in range(0, len(mesh_indices), 3):
                yield tuple(mesh_indices[offset:offset + 3])

    # 输出统一为扁平整数列表；索引按新顶点编号重建。
    merged_indices = []
    for triangle in _triangles(a["indices"]):
        merged_indices.extend(triangle)
    for triangle in _triangles(b["indices"]):
        merged_indices.extend(remap[index_b] for index_b in triangle)

    merged_rings = [list(ring) for ring in a["rings"]]
    merged_rings.extend(
        [remap[index_b] for index_b in ring] for ring in b["rings"][1:]
    )

    merged = {
        "vertices": merged_vertices,
        "normals": merged_normals,
        "indices": merged_indices,
        "rings": merged_rings,
    }

    # 任一侧带 UV 就保留 UV（缺失侧用 (0, 0) 补齐）。
    if "uvs" in a or "uvs" in b:
        a_uvs = a.get("uvs", [(0.0, 0.0)] * vertex_count_a)
        b_uvs = b.get("uvs", [(0.0, 0.0)] * vertex_count_b)
        merged_uvs = list(a_uvs)
        for index_b in range(vertex_count_b):
            if index_b not in remap or remap[index_b] >= vertex_count_a:
                merged_uvs.append(b_uvs[index_b])
        merged["uvs"] = merged_uvs

    for passthrough_key in ("lanes", "width"):
        if passthrough_key in a:
            merged[passthrough_key] = a[passthrough_key]
    return merged


# ---------------------------------------------------------------------------
# 8. 道路校验：重复点 / 间距过小 / 急弯（>60°）
# ---------------------------------------------------------------------------
def validate_road(points, min_spacing):
    """检查道路控制点并返回问题描述列表。"""
    if isinstance(min_spacing, bool) or not isinstance(min_spacing, (int, float)):
        raise RoadError(f"min_spacing 必须是数值，实际: {min_spacing!r}")
    if not math.isfinite(min_spacing) or min_spacing <= 0.0:
        raise RoadError(
            f"min_spacing 必须严格大于 0，实际: {min_spacing!r}"
        )
    if not points:
        return []
    pts = [_check_vec3(point, "控制点") for point in points]
    if len(pts) == 1:
        return []

    issues = []
    distances = []
    for index in range(len(pts) - 1):
        distance = math.dist(pts[index], pts[index + 1])
        distances.append(distance)
        if distance <= EPS:
            issues.append(f"重复点: 点 {index} 与点 {index + 1} 距离 {distance:.6f} <= 1e-6")
        if distance < min_spacing:
            issues.append(
                f"间距过小: 点 {index}->点 {index + 1} 间距 "
                f"{distance:.4f}m < {min_spacing:.4f}m"
            )

    # 急弯：中间点入向/出向夹角 > 60°。
    for index in range(1, len(pts) - 1):
        incoming = _sub(pts[index], pts[index - 1])
        outgoing = _sub(pts[index + 1], pts[index])
        length_in = _norm(incoming)
        length_out = _norm(outgoing)
        if length_in <= 0.0 or length_out <= 0.0:
            continue  # 重合点已在上面报过。
        cosine = _dot(incoming, outgoing) / (length_in * length_out)
        cosine = min(1.0, max(-1.0, cosine))
        angle = math.degrees(math.acos(cosine))
        if angle > 60.0:
            issues.append(f"急弯: 点 {index} 转角 {angle:.2f}° > 60°")
    return issues


# ---------------------------------------------------------------------------
# 9/10. 批量生成：独立缓冲、先全部成功再返回、非法 spacing 提前拦截
# ---------------------------------------------------------------------------
_REQUIRED_CONFIG_KEYS = ("width", "lanes", "spacing", "min_spacing")


def road_batch(roads, config):
    """批量生成道路网格，失败时整体回滚。"""
    if not isinstance(config, dict):
        raise RoadError("config 必须是字典")
    for key in _REQUIRED_CONFIG_KEYS:
        if key not in config:
            raise RoadError(f"config 缺少必需配置项: {key!r}")

    width = _check_positive_number(config["width"], "config['width']")
    lanes = _check_lanes(config["lanes"])
    spacing_value = config["spacing"]
    if (
        isinstance(spacing_value, bool)
        or not isinstance(spacing_value, (int, float))
        or not math.isfinite(spacing_value)
        or spacing_value <= 0.0
    ):
        raise RoadError(
            f"config['spacing'] 必须严格大于 0，实际: {spacing_value!r}"
        )
    min_spacing = config["min_spacing"]
    if (
        isinstance(min_spacing, bool)
        or not isinstance(min_spacing, (int, float))
        or not math.isfinite(min_spacing)
        or min_spacing <= 0.0
    ):
        raise RoadError(
            f"config['min_spacing'] 必须严格大于 0，实际: {min_spacing!r}"
        )
    up = _check_vec3(config.get("up", DEFAULT_UP), "config['up']")
    if _norm(up) <= 0.0:
        raise RoadError("config['up'] 不能是零向量")

    road_list = list(roads)
    if not road_list:
        return {"ok": True, "roads": [], "count": 0}

    # 先校验并构建全部道路；任何一条失败都不会返回部分结果。
    built = []
    for road_index, control_points in enumerate(road_list):
        issues = validate_road(control_points, min_spacing)
        if issues:
            raise RoadError(f"道路 {road_index} 校验失败: {'; '.join(issues)}")
        centerline = resample_by_length(control_points, spacing_value)
        # 每次调用都新建顶点/法线/UV/索引缓冲，道路之间互不共享可变列表。
        mesh = _build_ribbon(centerline, width, lanes, up)
        built.append(mesh)

    return {"ok": True, "roads": built, "count": len(built)}
