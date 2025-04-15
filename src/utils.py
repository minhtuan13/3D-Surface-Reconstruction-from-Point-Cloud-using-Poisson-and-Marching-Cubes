import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree

# Hàm dùng để hiển thị đối tượng (point cloud hoặc mesh) bằng Open3D
def visualize_object(obj, title="Open3D Visualization"):
    o3d.visualization.draw_geometries([obj], window_name=title)

# Hàm ước lượng tự động voxel_size và ngưỡng iso_level (dưới dạng percentile) cho Marching Cubes
def estimate_parameters(pcd):
    points = np.asarray(pcd.points)
    
    # Nếu số lượng điểm quá ít thì không thể xử lý
    if len(points) < 2:
        raise ValueError("Point cloud too small.")

    # Xây dựng cây tìm kiếm lân cận gần nhất (KDTree) để ước lượng khoảng cách trung bình giữa các điểm
    k = min(30, len(points) - 1)
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k+1)  # k+1 vì tính luôn khoảng cách đến chính nó ở vị trí đầu tiên

    # Lấy trung vị khoảng cách lân cận (bỏ khoảng cách đến chính nó) và nhân với hệ số để có voxel_size
    voxel_size = np.median(distances[:, 1:]) * 0.6

    # Ước lượng độ biến thiên của khoảng cách lân cận gần nhất để tính ngưỡng iso_level_percentile
    all_distances, _ = tree.query(points, k=4)
    nearest = all_distances[:, 1]  # khoảng cách đến điểm gần nhất (không tính chính nó)
    cv = np.std(nearest) / np.mean(nearest)  # hệ số biến thiên (coefficient of variation)

    # Tính iso_level_percentile sao cho càng gần biên vật thể thì ngưỡng càng thấp
    iso_percentile = np.clip(17 * (1 - cv), 5, 10)

    return voxel_size, iso_percentile
