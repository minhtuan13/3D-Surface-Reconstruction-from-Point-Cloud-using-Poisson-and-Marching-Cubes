import open3d as o3d
import numpy as np

class PointCloudProcessor:
    def __init__(self, file_path):
        # Load point cloud từ file
        self.pcd = o3d.io.read_point_cloud(file_path)
        print("Loaded point cloud:", self.pcd)

    def show(self, window_name="Point Cloud"):
        # Hiển thị point cloud
        o3d.visualization.draw_geometries([self.pcd], window_name=window_name)

    def filter_with_dbscan(self, eps=0.13, min_points=3):
        # Lọc nhiễu sử dụng thuật toán DBSCAN
        print("Filtering outliers using DBSCAN...")
        labels = np.array(self.pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
        if labels.max() < 0:
            print("No clusters found.")
            return

        # Giữ lại cụm có số lượng điểm lớn nhất
        largest_cluster = np.argmax(np.bincount(labels[labels >= 0]))
        indices = np.where(labels == largest_cluster)[0]
        self.pcd = self.pcd.select_by_index(indices)
        print("Filtered point cloud.")

    def remove_outliers(self):
        # Lọc nhiễu bằng phương pháp thống kê (Statistical Outlier Removal)
        print("Removing noise using Statistical Outlier Removal...")
        cl_stat, ind_stat = self.pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        self.display_inlier_outlier(self.pcd, ind_stat)
        self.pcd = self.pcd.select_by_index(ind_stat)

        # Nếu muốn dùng thêm Radius Outlier Removal, bỏ comment đoạn dưới
        # print("Removing noise using Radius Outlier Removal...")
        # cl_rad, ind_rad = self.pcd.remove_radius_outlier(nb_points=16, radius=0.05)
        # self.display_inlier_outlier(self.pcd, ind_rad)
        # self.pcd = self.pcd.select_by_index(ind_rad)

    def display_inlier_outlier(self, cloud, ind):
        # Hiển thị điểm inlier (màu xám) và outlier (màu đỏ)
        inlier_cloud = cloud.select_by_index(ind)
        outlier_cloud = cloud.select_by_index(ind, invert=True)
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        print("Displaying inliers (gray) and outliers (red)...")
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def estimate_normals(self, radius=0.01, max_nn=30):
        # Ước lượng vector pháp tuyến cho point cloud
        print("Estimating normals...")
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        self.pcd.normalize_normals()

    def get(self):
        # Trả về point cloud đã xử lý
        return self.pcd
