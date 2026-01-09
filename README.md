# pointcloud_to_ply

ROS 2 package for converting PointCloud2 messages to triangle meshes (OBJ/PLY) using PCL.

## Features

- **Preprocessing**: Voxel downsampling, statistical outlier removal
- **Normal Estimation**: Hybrid radius/kNN search
- **Mesh Reconstruction**:
  - Poisson surface reconstruction
  - Ball Pivoting Algorithm (BPA)
- **Export**: OBJ or PLY format

## Dependencies

All dependencies are available via APT:

```bash
sudo apt install ros-jazzy-pcl-ros ros-jazzy-pcl-conversions libpcl-dev
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_to_ply
source install/setup.bash
```

## Usage

### Launch with default parameters

```bash
ros2 launch pointcloud_to_ply pointcloud_to_ply.launch.xml
```

### Launch with custom parameters

Edit `launch/pointcloud_to_ply.launch.xml` to modify parameters, or override via command line:

```bash
ros2 launch pointcloud_to_ply pointcloud_to_ply.launch.xml \
  pointcloud_topic:=/camera/depth/points \
  output_path:=/home/user/meshes \
  reconstruction_method:=bpa
```

## Parameters

| Parameter                     | Type     | Default               | Description                         |
| ----------------------------- | -------- | --------------------- | ----------------------------------- |
| `pointcloud_topic`            | string   | `/rc_viscore/points2` | Input PointCloud2 topic             |
| `output_path`                 | string   | `/tmp/mesh_output`    | Output directory                    |
| `output_basename`             | string   | `mesh`                | Output filename (without extension) |
| `output_format`               | string   | `obj`                 | Output format: `obj` or `ply`       |
| `voxel_downsample_size`       | double   | 0.01                  | Voxel size (meters), 0 to disable   |
| `remove_statistical_outliers` | bool     | true                  | Enable outlier removal              |
| `nso_nb_neighbors`            | int      | 20                    | Neighbors for outlier detection     |
| `nso_std_ratio`               | double   | 2.0                   | Std dev threshold for outliers      |
| `normal_radius`               | double   | 0.05                  | Search radius for normals (meters)  |
| `normal_max_nn`               | int      | 30                    | Max neighbors for normal estimation |
| `reconstruction_method`       | string   | `poisson`             | Method: `poisson` or `bpa`          |
| `poisson_depth`               | int      | 9                     | Octree depth (8-12 typical)         |
| `poisson_density_quantile`    | double   | 0.05                  | Density threshold (0-1)             |
| `bpa_radii`                   | double[] | [0.02, 0.04, 0.08]    | Ball radii for BPA                  |
| `bpa_smooth`                  | bool     | false                 | Enable smoothing                    |
| `shutdown_after_save`         | bool     | true                  | Shutdown node after saving          |

## Example Workflows

### High-quality Poisson reconstruction

```xml
<param name="voxel_downsample_size" value="0.005"/>
<param name="poisson_depth" value="11"/>
<param name="poisson_density_quantile" value="0.01"/>
```

### Fast BPA reconstruction

```xml
<param name="reconstruction_method" value="bpa"/>
<param name="voxel_downsample_size" value="0.02"/>
<param name="bpa_radii" value="[0.03, 0.06]"/>
```

## License

BSD

| Parameter                     | Type   | Default               | Description                         |
| ----------------------------- | ------ | --------------------- | ----------------------------------- |
| `pointcloud_topic`            | string | `/rc_viscore/points2` | Input PointCloud2 topic             |
| `output_path`                 | string | `/tmp/mesh_output`    | Output directory                    |
| `output_basename`             | string | `mesh`                | Output filename (without extension) |
| `output_format`               | string | `obj`                 | Output format: `obj` or `ply`       |
| `voxel_downsample_size`       | double | 0.01                  | Voxel size (meters), 0 to disable   |
| `remove_statistical_outliers` | bool   | true                  | Enable outlier removal              |
| `nso_nb_neighbors`            | int    | 20                    | Neighbors for outlier detection     |
| `nso_std_ratio`               | double | 2.0                   | Std dev threshold for outliers      |
| `normal_radius`               | double | 0.05                  | Search radius for normals (meters)  |
| `normal_max_nn`               | int    | 30                    | Max neighbors for normal estimation |
| `poisson_depth`               | int    | 9                     | Octree depth (8-12 typical)         |
| `poisson_density_quantile`    | double | 0.05                  | Not fully supported in PCL          |
| `shutdown_after_save`         | bool   | true                  | Shutdown node after saving          |
