#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import trimesh

def get_aabb_from_stl(stl_path):
    """仅计算 STL 文件的轴对齐包围盒 (AABB)，返回尺寸和中心"""
    mesh = trimesh.load(stl_path)
    extents = mesh.extents                     # [dx, dy, dz]
    center = mesh.bounds.mean(axis=0)          # 中心坐标
    return extents.tolist(), center.tolist()

def main():
    stl_dir = os.path.join(os.path.dirname(__file__), 'meshes')
    if not os.path.isdir(stl_dir):
        print(f"错误：找不到目录 {stl_dir}")
        print("请将脚本放在 ar3_description 包的根目录，或修改 stl_dir 变量。")
        sys.exit(1)
    
    stl_files = [
        'base_link.STL',
        'link_1.STL',
        'link_2.STL',
        'link_3.STL',
        'link_4.STL',
        'link_5.STL',
        'link_6.STL'
    ]
    
    results = []
    for fname in stl_files:
        full_path = os.path.join(stl_dir, fname)
        if not os.path.isfile(full_path):
            print(f"警告：文件 {full_path} 不存在，跳过。")
            continue
        print(f"正在处理 {fname} ...")
        try:
            dims, center = get_aabb_from_stl(full_path)
            results.append((fname, dims, center))
        except Exception as e:
            print(f"处理 {fname} 时出错: {e}")
            continue

    print("\n" + "="*60)
    print("连杆简化碰撞参数（轴对齐包围盒 AABB）")
    print("可直接用于 xacro 文件")
    print("="*60)
    
    for fname, dims, center in results:
        print(f"\n【{fname}】")
        print(f"尺寸: [{dims[0]:.6f}, {dims[1]:.6f}, {dims[2]:.6f}]")
        print(f"中心: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]")
        print("RPY : [0.0, 0.0, 0.0]")
        print("\nxacro 片段:")
        print(f'<collision>')
        print(f'  <origin xyz="{center[0]:.6f} {center[1]:.6f} {center[2]:.6f}" rpy="0 0 0"/>')
        print(f'  <geometry><box size="{dims[0]:.6f} {dims[1]:.6f} {dims[2]:.6f}"/></geometry>')
        print(f'</collision>')
        print("-"*40)

if __name__ == "__main__":
    main()