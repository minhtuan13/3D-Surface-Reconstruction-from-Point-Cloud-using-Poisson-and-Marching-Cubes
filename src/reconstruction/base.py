from abc import ABC, abstractmethod

# Lớp cơ sở dùng để tái tạo bề mặt (mesh) từ đám mây điểm (point cloud)
class BaseReconstructor(ABC):
    def __init__(self, pcd):
        self.pcd = pcd        # Lưu trữ đối tượng point cloud đầu vào
        self.mesh = None      # Khởi tạo biến để chứa mesh sau khi tái tạo

    @abstractmethod
    def reconstruct(self):
        # Phương thức trừu tượng: các lớp con phải cài đặt lại phương thức này
        pass

    def transfer_colors_from_pcd(self):
        # Chuyển màu từ point cloud sang mesh nếu point cloud có màu
        import open3d as o3d

        if not self.pcd.has_colors():
            print("No colors to transfer.")
            return

        print("Transferring colors from point cloud to mesh...")
        pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        mesh_colors = []

        # Duyệt từng đỉnh trong mesh và tìm điểm gần nhất trong point cloud để lấy màu
        for v in self.mesh.vertices:
            [_, idx, _] = pcd_tree.search_knn_vector_3d(v, 1)
            mesh_colors.append(self.pcd.colors[idx[0]])

        # Gán màu cho các đỉnh trong mesh
        self.mesh.vertex_colors = o3d.utility.Vector3dVector(mesh_colors)

    def save(self, filename):
        # Lưu mesh ra file
        import open3d as o3d
        o3d.io.write_triangle_mesh(filename, self.mesh)
        print(f"Mesh saved as {filename}")

    def show(self):
        # Hiển thị mesh bằng công cụ visualize có sẵn
        from src.utils import visualize_object
        visualize_object(self.mesh)
