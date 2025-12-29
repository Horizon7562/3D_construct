import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import os
import sys




bag_path = r"D:\mouse\01_3D.bag"    # 换成你的 3D bag
output_pcd = "D:\mouse\cloud_raw.ply"

pipeline = rs.pipeline()
config = rs.config()

rs.config.enable_device_from_file(config, bag_path, repeat_playback=False)
config.enable_all_streams()

profile = pipeline.start(config)
playback = profile.get_device().as_playback()
playback.set_real_time(False)

pc = rs.pointcloud()

while True:
    try:
        frames = pipeline.wait_for_frames()
    except RuntimeError:
        break

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame:
        continue

    if color_frame:
        pc.map_to(color_frame)

    points = pc.calculate(depth_frame)

    # 顶点
    vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vtx)

    # 如果有彩色帧，加颜色
    if color_frame:
        color_img = np.asanyarray(color_frame.get_data())
        h, w, _ = color_img.shape

        tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        colors = np.zeros((vtx.shape[0], 3))

        for i, (u, v) in enumerate(tex):
            x = int(u * w)
            y = int(v * h)
            if 0 <= x < w and 0 <= y < h:
                colors[i] = color_img[y, x] / 255.0

        pcd.colors = o3d.utility.Vector3dVector(colors)

 

    o3d.io.write_point_cloud(output_pcd, pcd)
    print("Saved:", output_pcd)
    break  # 用第一帧足够做轮廓

pipeline.stop()



