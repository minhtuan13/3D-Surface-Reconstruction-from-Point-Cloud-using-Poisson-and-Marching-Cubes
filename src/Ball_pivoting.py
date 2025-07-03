import open3d as o3d
import numpy as np

# === 1. Đọc file point cloud ===
input_file = "your_pointcloud.ply"  # <-- Đổi tên file tại đây
pcd = o3d.io.read_point_cloud(input_file)
print(f"Loaded point cloud with {len(pcd.points)} points.")

# === 2. Lọc nhiễu (tùy chọn) ===
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
# pcd, ind = pcd.remove_radius_outlier(nb_points=16, radius=0.01)
print(f"After denoising: {len(pcd.points)} points.")

# === 3. Tính normal nếu chưa có ===
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd.normalize_normals()

# === 4. Tính khoảng cách trung bình để ước lượng bán kính ===
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist  # Hệ số này bạn có thể điều chỉnh

# === 5. Áp dụng Ball Pivoting Algorithm ===
radii = [radius, radius * 2]  # thử nhiều bán kính
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd,
    o3d.utility.DoubleVector(radii)
)

# === 6. Làm mịn mesh (tùy chọn) ===
bpa_mesh.remove_duplicated_triangles()
bpa_mesh.remove_degenerate_triangles()
bpa_mesh.remove_duplicated_vertices()
bpa_mesh.remove_non_manifold_edges()

# === 7. Hiển thị kết quả ===
o3d.visualization.draw_geometries([bpa_mesh], mesh_show_back_face=True)

# === 8. Lưu kết quả ===
output_file = "output_mesh_bpa.obj"
o3d.io.write_triangle_mesh(output_file, bpa_mesh)
print(f"Mesh saved to: {output_file}")
