import open3d as o3d
import numpy as np

pcd_path = r"D:\mouse\cloud_raw.ply"
mesh_out = r"D:\mouse\model_alpha.ply"

print("📥 正在加载点云：", pcd_path)
pcd = o3d.io.read_point_cloud(pcd_path)
print("原始点数：", np.asarray(pcd.points).shape[0])

# ① 下采样
pcd = pcd.voxel_down_sample(voxel_size=0.005)

# ② 法向量（用于平面分割）
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
)

# ③ RANSAC 分割桌面
print("🧹 正在去除背景平面…")
_, inliers = pcd.segment_plane(
    distance_threshold=0.003,
    ransac_n=3,
    num_iterations=1000
)

mouse_pcd = pcd.select_by_index(inliers, invert=True)

print(f"小鼠点数量：{len(mouse_pcd.points)}")
if len(mouse_pcd.points) < 1000:
    print("⚠ 警告：小鼠点太少，可能 distance_threshold 过大，请调小。")

# 去噪（让 OBB 更稳）
if len(mouse_pcd.points) > 200:
    mouse_pcd, _ = mouse_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)

# 保存小鼠点云（调试）
mouse_only_path = r"D:\mouse\mouse_only_points.pcd"
o3d.io.write_point_cloud(mouse_only_path, mouse_pcd)
print("✔ 已输出点云：", mouse_only_path)

# ④ 表面重建
print("🧩 正在进行表面重建…")
mouse_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30)
)

radii = o3d.utility.DoubleVector([0.002, 0.004, 0.008])
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(mouse_pcd, radii)
mesh.compute_vertex_normals()

# ⑤ 输出模型
o3d.io.write_triangle_mesh(mesh_out, mesh)
print("🎉 三维小鼠表面已生成：", mesh_out)

# ⑥ OBB 包围盒
obb = mouse_pcd.get_oriented_bounding_box()
obb.color = (0, 1, 0)

# =======================================================
# ✅ 关键：原点直接取 OBB 的真实顶点（get_box_points）
# =======================================================
corners = np.asarray(obb.get_box_points())  # shape (8,3)

# 你要“在包围盒顶点上”，这里默认选“最上面的顶点”(世界坐标 Z 最大)
origin = corners[np.argmax(corners[:, 2])]

# 如果你想换别的顶点：比如最小Z（最低点）用 np.argmin(corners[:,2])
# origin = corners[np.argmin(corners[:, 2])]

# 坐标轴方向：对齐 OBB
R = np.asarray(obb.R, dtype=np.float64,copy=True)
R[:, 0] *= -1 
R[:, 1] *= -1   # 翻转 Y（绿）
R[:, 2] *= -1   # 翻转 Z（蓝）

extent = np.asarray(obb.extent, dtype=np.float64)
axis_size = float(np.max(extent) * 0.6) if np.max(extent) > 0 else 0.1

axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_size, origin=[0, 0, 0])
axis.rotate(R, center=(0, 0, 0))
axis.translate(origin)

# 原点标记（小球），让你一眼确认“原点就是顶点”
marker_radius = max(axis_size * 0.05, 0.002)
origin_marker = o3d.geometry.TriangleMesh.create_sphere(radius=marker_radius)
origin_marker.translate(origin)
origin_marker.compute_vertex_normals()

# 比例尺：从该顶点出发，沿 OBB 局部 +X 方向画 20cm
scale_len = 0.2  # 0.2m
p0 = origin
p1 = origin + R @ np.array([scale_len, 0.0, 0.0], dtype=np.float64)

scale_line = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.vstack([p0, p1])),
    lines=o3d.utility.Vector2iVector([[0, 1]])
)
scale_line.paint_uniform_color([1, 0, 0])

# ⑦ 可视化
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Mouse Mesh + OBB + Axis@OBB-Vertex")

vis.add_geometry(mesh)
vis.add_geometry(obb)
vis.add_geometry(axis)
vis.add_geometry(origin_marker)
vis.add_geometry(scale_line)

vc = vis.get_view_control()
vc.set_lookat(np.asarray(obb.center))
vc.set_up([0, -1, 0])
vc.set_zoom(0.6)

vis.run()
vis.destroy_window()
