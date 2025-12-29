# # import open3d as o3d
# # import numpy as np

# # pcd_path = "D:\mouse\cloud_raw.ply"
# # mesh_out = "D:\mouse\model_alpha.ply"   # 输出三维轮廓

# # pcd = o3d.io.read_point_cloud(pcd_path)
# # print("原始点数：", len(pcd.points))

# # # ① 下采样
# # pcd = pcd.voxel_down_sample(voxel_size=0.005)

# # # ② 去噪
# # pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# # # ③ 法向量
# # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
# #     radius=0.02, max_nn=30))

# # # ④ Alpha Shape 重建（适合生成“轮廓”）
# # alpha = 0.02       # 关键参数，越小越贴近点云
# # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
# # mesh.compute_vertex_normals()

# # # 显示
# # o3d.visualization.draw_geometries([mesh])

# # # 保存
# # o3d.io.write_triangle_mesh(mesh_out, mesh)
# # print("三维轮廓已保存：", mesh_out)



# # import open3d as o3d
# # import numpy as np

# # pcd_path = "D:\mouse\cloud_raw.ply"
# # mesh_out = "D:\mouse\model_alpha.ply"

# # print("📥 正在加载点云：", pcd_path)
# # pcd = o3d.io.read_point_cloud(pcd_path)
# # points = np.asarray(pcd.points)
# # print("原始点数：", points.shape[0])

# # # ① 下采样（可选）
# # pcd = pcd.voxel_down_sample(voxel_size=0.005)

# # # ② 估计法向量（用于平面分割）
# # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
# #     radius=0.05, max_nn=30))


# # # -----------------------------------------
# # # ③ RANSAC 分割平面（背景桌面）
# # # -----------------------------------------
# # print("🧹 正在去除背景平面…")

# # plane_model, inliers = pcd.segment_plane(
# #     distance_threshold=0.003,    # 越小越严格，可调
# #     ransac_n=3,
# #     num_iterations=1000
# # )

# # # 背景：inliers
# # background = pcd.select_by_index(inliers)

# # # 小鼠：非平面点 = invert=True
# # mouse_pcd = pcd.select_by_index(inliers, invert=True)

# # print(f"背景点数量：{len(background.points)}")
# # print(f"小鼠点数量：{len(mouse_pcd.points)}")

# # if len(mouse_pcd.points) < 1000:
# #     print("⚠ 警告：小鼠点太少，可能平面阈值过大，请调小 distance_threshold。")

# # # 保存纯小鼠点云（可调试）
# # o3d.io.write_point_cloud("mouse_only_points.pcd", mouse_pcd)
# # print("✔ 已输出点云：mouse_only_points.pcd")


# # # -----------------------------------------
# # # ④ 对“小鼠点云”进行表面重建
# # # -----------------------------------------
# # print("🧩 正在进行表面重建…")

# # # 重新估计法向量（更准确）
# # mouse_pcd.estimate_normals(
# #     search_param=o3d.geometry.KDTreeSearchParamHybrid(
# #         radius=0.03, max_nn=30))

# # # 使用 ball pivoting surface reconstruction（适合动物体表）
# # radii = o3d.utility.DoubleVector([0.002, 0.004, 0.008])
# # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
# #     mouse_pcd,
# #     radii
# # )

# # mesh.compute_vertex_normals()

# # # -----------------------------------------
# # # ⑤ 输出模型
# # # -----------------------------------------
# # o3d.io.write_triangle_mesh(mesh_out, mesh)
# # print("🎉 三维小鼠表面已生成：", mesh_out)

# # # 放大模型
# # scale_factor = 1000  # 设置放大倍数
# # mesh.scale(scale_factor, center=[0, 0, 0])

# # # 可视化
# # o3d.visualization.draw_geometries([mesh])



# import open3d as o3d
# import numpy as np

# pcd_path = "D:\\mouse\\cloud_raw.ply"
# mesh_out = "D:\\mouse\\model_alpha.ply"

# print("📥 正在加载点云：", pcd_path)
# pcd = o3d.io.read_point_cloud(pcd_path)
# points = np.asarray(pcd.points)
# print("原始点数：", points.shape[0])

# # ① 下采样（可选）
# pcd = pcd.voxel_down_sample(voxel_size=0.005)

# # ② 估计法向量（用于平面分割）
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
#     radius=0.05, max_nn=30))

# # -----------------------------------------
# # ③ RANSAC 分割平面（背景桌面）
# # -----------------------------------------
# print("🧹 正在去除背景平面…")

# plane_model, inliers = pcd.segment_plane(
#     distance_threshold=0.003,    # 越小越严格，可调
#     ransac_n=3,
#     num_iterations=1000
# )

# # 背景：inliers
# background = pcd.select_by_index(inliers)

# # 小鼠：非平面点 = invert=True
# mouse_pcd = pcd.select_by_index(inliers, invert=True)

# print(f"背景点数量：{len(background.points)}")
# print(f"小鼠点数量：{len(mouse_pcd.points)}")

# if len(mouse_pcd.points) < 1000:
#     print("⚠ 警告：小鼠点太少，可能平面阈值过大，请调小 distance_threshold。")

# # 保存纯小鼠点云（可调试）
# o3d.io.write_point_cloud("mouse_only_points.pcd", mouse_pcd)
# print("✔ 已输出点云：mouse_only_points.pcd")

# # -----------------------------------------
# # ④ 对“小鼠点云”进行表面重建
# # -----------------------------------------
# print("🧩 正在进行表面重建…")

# # 重新估计法向量（更准确）
# mouse_pcd.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(
#         radius=0.03, max_nn=30))

# # 使用 ball pivoting surface reconstruction（适合动物体表）
# radii = o3d.utility.DoubleVector([0.002, 0.004, 0.008])
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     mouse_pcd,
#     radii
# )

# mesh.compute_vertex_normals()

# # -----------------------------------------
# # ⑤ 输出模型
# # -----------------------------------------
# o3d.io.write_triangle_mesh(mesh_out, mesh)
# print("🎉 三维小鼠表面已生成：", mesh_out)

# # 放大模型
# scale_factor = 1000  # 设置放大倍数
# mesh.scale(scale_factor, center=[0, 0, 0])

# # -----------------------------------------
# # ⑥ 调整视角，自动适应
# # -----------------------------------------
# vis = o3d.visualization.Visualizer()
# vis.create_window()

# # 添加点云或网格
# vis.add_geometry(mesh)

# # 获取视角控制器
# view_control = vis.get_view_control()

# # 自动调整缩放，确保模型可视
# view_control.set_zoom(0.5)  # 调整这个值确保模型适应视窗大小
# view_control.set_lookat([0, 0, 0])  # 设置模型居中
# view_control.set_up([0, -1, 0])  # 设置视角方向

# # 展示
# vis.run()
# vis.destroy_window()



import open3d as o3d
import numpy as np

pcd_path = "D:\\mouse\\cloud_raw.ply"
mesh_out = "D:\\mouse\\model_alpha.ply"

print("📥 正在加载点云：", pcd_path)
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)
print("原始点数：", points.shape[0])

# ① 下采样（可选）
pcd = pcd.voxel_down_sample(voxel_size=0.005)

# ② 估计法向量（用于平面分割）
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    radius=0.05, max_nn=30))

# -----------------------------------------
# ③ RANSAC 分割平面（背景桌面）
# -----------------------------------------
print("🧹 正在去除背景平面…")

plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.003,    # 越小越严格，可调
    ransac_n=3,
    num_iterations=1000
)

# 背景：inliers
background = pcd.select_by_index(inliers)

# 小鼠：非平面点 = invert=True
mouse_pcd = pcd.select_by_index(inliers, invert=True)

print(f"背景点数量：{len(background.points)}")
print(f"小鼠点数量：{len(mouse_pcd.points)}")

if len(mouse_pcd.points) < 1000:
    print("⚠ 警告：小鼠点太少，可能平面阈值过大，请调小 distance_threshold。")

# 保存纯小鼠点云（可调试）
o3d.io.write_point_cloud("mouse_only_points.pcd", mouse_pcd)
print("✔ 已输出点云：mouse_only_points.pcd")

# -----------------------------------------
# ④ 对“小鼠点云”进行表面重建
# -----------------------------------------
print("🧩 正在进行表面重建…")

# 重新估计法向量（更准确）
mouse_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.03, max_nn=30))

# 使用 ball pivoting surface reconstruction（适合动物体表）
radii = o3d.utility.DoubleVector([0.002, 0.004, 0.008])
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    mouse_pcd,
    radii
)

mesh.compute_vertex_normals()

# -----------------------------------------
# ⑤ 输出模型
# -----------------------------------------
o3d.io.write_triangle_mesh(mesh_out, mesh)
print("🎉 三维小鼠表面已生成：", mesh_out)

# 放大模型
scale_factor = 1000  # 设置放大倍数
mesh.scale(scale_factor, center=[0, 0, 0])

# -----------------------------------------
# ⑥ 调整视角，自动适应
# -----------------------------------------
vis = o3d.visualization.Visualizer()
vis.create_window()

# 添加点云或网格
vis.add_geometry(mesh)

# -----------------------------------------
# ⑦ 添加三维坐标轴
# -----------------------------------------
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
vis.add_geometry(axis)


# -----------------------------------------
# ⑧ 添加坐标尺（比例尺）
# -----------------------------------------
# 创建一个线条对象，表示比例尺
line_set = o3d.geometry.LineSet()

# 设置比例尺的起点和终点（增加长度）
points = np.array([[0, 0, 0], [0.2, 0, 0]])  # 增大比例尺长度为 20cm
line_set.points = o3d.utility.Vector3dVector(points)

# 设置连接的两点
lines = [[0, 1]]  # 连接点 0 和点 1
line_set.lines = o3d.utility.Vector2iVector(lines)

# 设置线条颜色
line_set.paint_uniform_color([1, 0, 0])  # 红色

# 添加到可视化中
vis.add_geometry(line_set)

# 获取视角控制器
view_control = vis.get_view_control()

# 自动调整缩放，确保模型可视
view_control.set_zoom(0.5)  # 调整这个值确保模型适应视窗大小
view_control.set_lookat([0, 0, 0])  # 设置模型居中
view_control.set_up([0, -1, 0])  # 设置视角方向

# 展示
vis.run()
vis.destroy_window()





