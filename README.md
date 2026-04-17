# PointCloud Converter

- **Livox2PCL**: 将 Livox `CustomMsg` 转换为 PCL `PointXYZI` 类型的点云。
- **Livox2ROS**: 将 Livox `CustomMsg` 转换为 ROS `PointCloud2` 消息。
- **ROS2PCL**: 将 ROS `PointCloud2` 消息转换为 PCL `PointXYZI` 类型的点云。
- **PCL2ROS**: 将 PCL `PointXYZI` 点云转换为 ROS `PointCloud2` 消息。
- **ROS2PCLRGB / PCLRGB2ROS**: 支持带颜色的点云（`PointXYZRGB`）与 ROS 消息之间的转换。
- **NormalToIntensity**: 将带法向量的点云（`PointXYZINormal`）转换为带强度的点云（`PointXYZI`）。