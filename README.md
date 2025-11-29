# `pointcloud_to_ply`

Capture a point cloud from a sensor by subscribing to the topic where it publishes messages to, and store it in `.ply` or `.obj` form on disk.

Set your specific point cloud sensor topic, save location, and other params in `launch/pointcloud_to_ply.launch.xml`. 



Then launch the node with

```bash
ros2 launch pointcloud_to_ply pointcloud_to_ply.launch.xml
```
