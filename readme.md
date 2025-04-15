# 3D Surface Reconstruction from Point Cloud

This project implements two common 3D surface reconstruction algorithms — **Poisson Surface Reconstruction** and **Marching Cubes** — using the Open3D library. It processes a raw point cloud, filters noise, reconstructs a mesh, transfers colors, and visualizes or exports the result.

---

## Features

- **Point Cloud Loading & Visualization**: Load and visualize 3D point cloud (.ply format)
- **Noise Filtering**:
  - **DBSCAN** (Density-Based Spatial Clustering of Applications with Noise)
  - **Statistical Outlier Removal**
  - **Radius Outlier Removal**
- **Normal Estimation**: Estimate normals for the point cloud
- **Surface Reconstruction**:
  - **Poisson Reconstruction**: Generates watertight surfaces from point clouds
  - **Marching Cubes**: Uses nearest-neighbor distances to create surfaces
- **Mesh Simplification** (for Poisson method)
- **Vertex Color Transfer**: Transfer color information from the point cloud to the reconstructed mesh
- **Mesh Export**: Save the reconstructed mesh in `.obj` format

---

## Project Structure

```
surface_reconstruction_project/
│
├── main.py                        # Entry point to choose reconstruction mode
├── data/
│   └── wheel.ply                  # Sample input point cloud
├── outputs/
│   └── (exported mesh files)
├── src/
│   ├── utils.py                   # Utility functions (visualization, estimation)
│   ├── point_cloud_processor.py   # Noise filtering, normal estimation
│   └── reconstruction/
│       ├── base.py                # Abstract base class for reconstructors
│       ├── poisson.py             # Poisson reconstruction implementation
│       └── marching_cubes.py      # Marching Cubes implementation
```

---

## Getting Started

### 1. Install dependencies

Before running the project, you need to install the required dependencies. Run the following command to install the required Python packages:

```bash
pip install open3d numpy scipy matplotlib scikit-image
```

### 2. Run the main script

To run the project, simply execute the following command in your terminal:

```bash
python main.py
```

You will be prompted to choose the reconstruction method:

```
Surface Reconstruction Options:
1 - Poisson Reconstruction
2 - Marching Cubes Reconstruction
Choose 1 or 2:
```

### 3. Output

- The program will visualize the reconstructed mesh after the reconstruction process.
- The output mesh will be saved in the `outputs/` folder as a `.obj` file.
- Example output file names: `poisson_output.obj`, `marching_output.obj`.

---

## Notes

- **Point Cloud Format**: Ensure that the input point cloud file is in `.ply` format and contains color and normal information. If the normals are missing, the script will attempt to estimate them.
- **Parameter Adjustments**: You can modify parameters such as DBSCAN `eps`, normal estimation radius, Poisson reconstruction `depth`, and mesh simplification ratio directly in the code.
- **Point Cloud Size**: If the point cloud is too large, the script will downsample the cloud before applying the Marching Cubes algorithm.

---

## Reconstruction Methods

### Poisson Surface Reconstruction

- Poisson reconstruction generates a watertight surface using implicit functions. It's well-suited for noisy or sparse data and provides high-quality results.
- **Features**:
  - Handles noisy data effectively.
  - Capable of reconstructing closed surfaces (watertight meshes).
  - Supports density-based vertex filtering and mesh simplification.

### Marching Cubes

- Marching Cubes reconstructs surfaces by converting point cloud data into a scalar field (distance field) and applying the Marching Cubes algorithm to extract the surface.
- **Features**:
  - Generally faster than Poisson but may produce holes or artifacts in sparse regions.
  - More suitable for large point clouds.

---

