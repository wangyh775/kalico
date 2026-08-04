#!/usr/bin/env python3
"""
生成多层台阶柱 STL，用于诊断 CoreXY 逐层 XY 正向漂移。

设计：
- 底座 60×60×3mm，定位基准
- 每 10mm 高度做一个台阶，台阶宽度递增
- 总高 103mm，10 个台阶
- 台阶从 16×16mm 开始，每层 +4mm（对称），顶层 52×52mm
- 每个台阶 +X 和 +Y 侧面有 0.5mm 深的标记槽，作为卡尺测量基准

成型尺寸：500×400×300mm，这个件完全在范围内。
"""

import math
import os
import struct


def make_rect_prism(x, y, z, w, d, h):
    """
    生成一个长方体，底面中心在 (x, y, z)，尺寸 w×d×h。
    返回 (顶点列表, 三角形索引列表)。
    每个三角形索引相对于本组件顶点列表（从 0 开始）。
    """
    hw, hd = w / 2, d / 2
    verts = [
        (x - hw, y - hd, z),  # 0 底面左下
        (x + hw, y - hd, z),  # 1 底面右下
        (x + hw, y + hd, z),  # 2 底面右上
        (x - hw, y + hd, z),  # 3 底面左上
        (x - hw, y - hd, z + h),  # 4 顶面左下
        (x + hw, y - hd, z + h),  # 5 顶面右下
        (x + hw, y + hd, z + h),  # 6 顶面右上
        (x - hw, y + hd, z + h),  # 7 顶面左上
    ]
    tris = [
        (0, 2, 1),
        (0, 3, 2),  # 底面
        (4, 5, 6),
        (4, 6, 7),  # 顶面
        (0, 1, 5),
        (0, 5, 4),  # 前面 (y - hd)
        (1, 2, 6),
        (1, 6, 5),  # 右面 (x + hw)
        (2, 3, 7),
        (2, 7, 6),  # 后面 (y + hd)
        (3, 0, 4),
        (3, 4, 7),  # 左面 (x - hw)
    ]
    return verts, tris


def write_binary_stl(filename, all_verts, all_tris):
    """将三角网格写入二进制 STL。"""
    total_tris = sum(len(tris) for tris in all_tris)
    with open(filename, "wb") as f:
        f.write(b"Drift test column - Kalico diagnostics v1.0" + b"\x00" * 40)
        f.write(struct.pack("<I", total_tris))
        for verts, tris in zip(all_verts, all_tris):
            for i0, i1, i2 in tris:
                v0, v1, v2 = verts[i0], verts[i1], verts[i2]
                ux, uy, uz = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
                vx, vy, vz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
                nx = uy * vz - uz * vy
                ny = uz * vx - ux * vz
                nz = ux * vy - uy * vx
                length = math.sqrt(nx * nx + ny * ny + nz * nz)
                if length > 0:
                    nx, ny, nz = nx / length, ny / length, nz / length
                f.write(struct.pack("<fff", nx, ny, nz))
                f.write(struct.pack("<fff", v0[0], v0[1], v0[2]))
                f.write(struct.pack("<fff", v1[0], v1[1], v1[2]))
                f.write(struct.pack("<fff", v2[0], v2[1], v2[2]))
                f.write(b"\x00\x00")


def main():
    all_verts = []
    all_tris = []

    # 1. 底座
    v, t = make_rect_prism(0, 0, 0, 60, 60, 3)
    all_verts.append(v)
    all_tris.append(t)

    # 2. 台阶
    STEP_H = 10
    STEP_COUNT = 10
    STEP_START = 16
    STEP_INCR = 4
    BASE_H = 3

    for i in range(STEP_COUNT):
        w = STEP_START + i * STEP_INCR
        z = BASE_H + i * STEP_H

        # 台阶主体
        v, t = make_rect_prism(0, 0, z, w, w, STEP_H)
        all_verts.append(v)
        all_tris.append(t)

        # +X 侧面标记槽（测量基准点）
        # 一个薄片凸出在台阶侧面，便于卡尺定位
        nd = 0.5  # 槽深度
        nw = 5.0  # 槽宽度
        nh = 3.0  # 槽高度
        cz = z + (STEP_H - nh) / 2
        # +X 面
        vn, tn = make_rect_prism(w / 2 + nd / 2, 0, cz, nd, nw, nh)
        all_verts.append(vn)
        all_tris.append(tn)
        # +Y 面
        vn, tn = make_rect_prism(0, w / 2 + nd / 2, cz, nw, nd, nh)
        all_verts.append(vn)
        all_tris.append(tn)

    # 输出路径
    outdir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "Figure")
    os.makedirs(outdir, exist_ok=True)
    outpath = os.path.join(outdir, "drift_test_column.stl")

    write_binary_stl(outpath, all_verts, all_tris)
    total_height = BASE_H + STEP_COUNT * STEP_H
    print(f"✅ 已生成: {outpath}")
    print(f"   总高: {total_height:.0f}mm, 台阶数: {STEP_COUNT}")
    print(
        f"   底座: 60×60mm, 顶层: {STEP_START + (STEP_COUNT - 1) * STEP_INCR:.0f}×{STEP_START + (STEP_COUNT - 1) * STEP_INCR:.0f}mm"
    )
    print(f"   三角形数: {sum(len(t) for t in all_tris)}")
    print()
    print("使用说明:")
    print("  - 切片时底座贴在热床上，X=0 Y=0 对齐到打印机原点")
    print("  - 打印完成后，用卡尺测量每个台阶 +X 侧标记槽到参考边的距离")
    print("  - 如果距离随 Z 增加而递增 → 存在 XY 正向漂移")
    print("  - 同法测量 +Y 侧标记槽，判断漂移方向")


if __name__ == "__main__":
    main()
