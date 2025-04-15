from src.point_cloud_processor import PointCloudProcessor
from src.reconstruction.poisson import PoissonReconstructor
from src.reconstruction.marching_cubes import MarchingCubesReconstructor
import os

def main():
    print("Surface Reconstruction Options:")
    print("1 - Poisson Reconstruction")
    print("2 - Marching Cubes Reconstruction")
    choice = input("Chọn 1 hoặc 2: ").strip()

    input_file = "data/fragment.ply"
    output_dir = "outputs"
    os.makedirs(output_dir, exist_ok=True)

    processor = PointCloudProcessor(input_file)
    processor.show("Original Point Cloud")
    processor.filter_with_dbscan()
    processor.show("Filtered Point Cloud")

    processor.remove_outliers()
    processor.estimate_normals()

    if choice == "1":
        recon = PoissonReconstructor(processor.get())
        recon.reconstruct()
        recon.show_density()
        recon.remove_low_density()
        recon.simplify()
        recon.transfer_colors_from_pcd()
        recon.show()
        recon.save(os.path.join(output_dir, "poisson_output.obj"))

    elif choice == "2":
        recon = MarchingCubesReconstructor(processor.get())
        recon.reconstruct()
        recon.transfer_colors_from_pcd()
        recon.show()
        recon.save(os.path.join(output_dir, "marching_output.obj"))

    else:
        print("Invalid choice.")

if __name__ == "__main__":
    main()

