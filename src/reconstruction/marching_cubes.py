import numpy as np
import open3d as o3d
import skimage.measure
from scipy.spatial import cKDTree
from src.utils import estimate_parameters  # Hàm tự động ước lượng voxel_size và iso_level_percentile
from .base import BaseReconstructor  # Lớp cơ sở chứa pcd và thuộc tính mesh


class MarchingCubesReconstructor(BaseReconstructor):
    def reconstruct(self):
        # Chuyển point cloud thành mảng numpy
        points = np.asarray(self.pcd.points)

        # Ước lượng tự động kích thước voxel và ngưỡng isosurface từ dữ liệu đầu vào
        voxel_size, iso_level_percentile = estimate_parameters(self.pcd)
        print("Voxel_size:", voxel_size)
        print("Iso_level_percentile:", iso_level_percentile)

        # Nếu point cloud quá lớn, thực hiện giảm mẫu bằng voxel
        if len(points) > 200000:
            print("Downsampling point cloud...")
            self.pcd = self.pcd.voxel_down_sample(voxel_size=voxel_size)
            points = np.asarray(self.pcd.points)

        # Tính giới hạn không gian (min/max) cho mỗi chiều x, y, z
        mins, maxs = points.min(axis=0), points.max(axis=0)

        # Tạo lưới voxel 3D bằng np.meshgrid theo kích thước voxel đã tính
        x, y, z = np.meshgrid(
            np.arange(mins[0], maxs[0], voxel_size),
            np.arange(mins[1], maxs[1], voxel_size),
            np.arange(mins[2], maxs[2], voxel_size),
            indexing='ij'
        )

        # Nếu lưới quá lớn, tăng kích thước voxel để giảm số lượng điểm lưới
        if x.size > 300000:
            print("Reducing grid resolution")
            voxel_size *= 1.5
            x, y, z = np.meshgrid(
                np.arange(mins[0], maxs[0], voxel_size),
                np.arange(mins[1], maxs[1], voxel_size),
                np.arange(mins[2], maxs[2], voxel_size),
                indexing='ij'
            )

        # Xây dựng cây tìm kiếm KDTree để tính khoảng cách nhanh hơn
        tree = cKDTree(points)

        # Ghép các tọa độ điểm trong lưới thành mảng N x 3
        grid_points = np.vstack([x.ravel(), y.ravel(), z.ravel()]).T

        # Tính khoảng cách gần nhất từ mỗi điểm trong lưới đến point cloud
        distances, _ = tree.query(grid_points)

        # Chuyển thành trường vô hướng 3D (scalar field)
        scalar_field = distances.reshape(x.shape)

        # Tính giá trị iso-level để sử dụng trong marching cubes
        iso_level = np.percentile(distances, iso_level_percentile)

        # Áp dụng thuật toán Marching Cubes từ Skimage để tạo mesh
        verts, faces, _, _ = skimage.measure.marching_cubes(scalar_field, level=iso_level)

        # Điều chỉnh tọa độ vertices theo vị trí thực tế trong không gian
        verts = verts * voxel_size + mins

        # Tạo đối tượng mesh trong Open3D
        self.mesh = o3d.geometry.TriangleMesh()
        self.mesh.vertices = o3d.utility.Vector3dVector(verts)
        self.mesh.triangles = o3d.utility.Vector3iVector(faces)
        self.mesh.compute_vertex_normals()

        # Trả về mesh đã dựng từ Marching Cubes
        return self.mesh
