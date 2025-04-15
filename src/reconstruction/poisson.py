import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from .base import BaseReconstructor

# Lớp tái tạo mesh sử dụng thuật toán Poisson
class PoissonReconstructor(BaseReconstructor):
    def reconstruct(self, depth=9):
        # Tái tạo bề mặt từ point cloud bằng thuật toán Poisson
        print("Running Poisson reconstruction...")
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug):
            self.mesh, self.densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.pcd, depth=depth)

    def show_density(self):
        # Hiển thị mật độ của các đỉnh trong mesh bằng màu sắc
        print("Visualizing vertex density...")
        d = np.asarray(self.densities)
        cmap = plt.get_cmap('plasma')((d - d.min()) / (d.max() - d.min()))
        self.mesh.vertex_colors = o3d.utility.Vector3dVector(cmap[:, :3])
        o3d.visualization.draw_geometries([self.mesh], window_name="Vertex Density")

    def remove_low_density(self, threshold=0.01):
        # Loại bỏ các đỉnh có mật độ thấp hơn ngưỡng được chỉ định
        print("Removing low-density vertices...")
        d = np.asarray(self.densities)
        mask = d < np.quantile(d, threshold)
        self.mesh.remove_vertices_by_mask(mask)

    def simplify(self, ratio=0.7):
        # Đơn giản hóa mesh bằng thuật toán quadric decimation
        print("Simplifying mesh...")
        target = int(len(self.mesh.triangles) * ratio)
        self.mesh = self.mesh.simplify_quadric_decimation(target)
