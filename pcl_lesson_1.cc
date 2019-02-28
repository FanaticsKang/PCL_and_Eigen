#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
int main(int argc, char** argv) {
  const int range = 10;
  const int range_x_min = -range;
  const int range_x_max = range;
  const int range_y_min = -range;
  const int range_y_max = range;
  const int range_z_min = -range;
  const int range_z_max = range;

  const double scale = 0.1;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  // cloud.width    = 2 * range;
  // cloud.height   = 2 * range;
  cloud.is_dense = false;
  cloud.points.reserve(pow(2 * range, 3));

  for (int i = range_x_min; i < range_x_max; ++i) {
    for (int j = range_y_min; j < range_y_max; ++j) {
      for (int k = range_z_min; k < range_z_max; ++k) {
        pcl::PointXYZ tmp_point(i * scale, j * scale, k * scale);
        // std::cout << tmp_point.x << std::endl;
        cloud.points.emplace_back(tmp_point);
      }
    }
  }

  std::cerr << "Saved " << cloud.points.size()
            << " data points to test_pcd.pcd." << std::endl;

  cloud.height = 1;
  cloud.width = cloud.points.size();

  pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);

  return (0);
}
