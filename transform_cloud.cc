#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <algorithm>
#include <iostream>
// This function displays the help
void showHelp(char *program_name) {
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]"
            << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}
// typedef typename pcl::PointCloud<pcl::PointXYZ>::iterator Itr_PCL;
// This is the main function
int main(int argc, char *argv[]) {
  // Show help
  if (pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help")) {
    showHelp(argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;
  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

  if (filenames.size() != 1) {
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if (filenames.size() != 1) {
      showHelp(argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }
  std::cout << "File Name: " << argv[filenames[0]] << std::endl;
  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_in_l(
      new pcl::PointCloud<pcl::PointXYZ>());

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile(argv[filenames[0]], *point_in_l) < 0) {
      std::cout << "Error loading point cloud " << argv[filenames[0]]
                << std::endl
                << std::endl;
      showHelp(argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile(argv[filenames[0]], *point_in_l) < 0) {
      std::cout << "Error loading point cloud " << argv[filenames[0]]
                << std::endl
                << std::endl;
      showHelp(argv[0]);
      return -1;
    }
  }

  //获得旋转信息
  Eigen::Matrix4f transform_1;
  transform_1 << -0.000406265, -0.99995703, 0.0092275403, -0.082944, 0.0545328,
      -0.0092358598, -0.998469, -0.448038, 0.99851203, 9.7483397e-05,
      0.054534201, -0.706973, 0, 0, 0, 1;

  // from lidar coordinate to camera coordinate
  Eigen::Affine3f T_cl(transform_1);

  // from Camera coordinate to Lidar coordinate
  Eigen::Affine3f T_lc = T_cl.inverse();

  printf("\nT_lc\n");
  std::cout << T_lc.matrix() << std::endl;

  // Delete the back points.
  pcl::PointCloud<pcl::PointXYZ>::iterator itr = point_in_l->begin();
  while (itr != point_in_l->end()) {
    if (itr->x < 0) {
      itr = point_in_l->erase(itr);
    } else {
      ++itr;
    }
  }
  // Point in camera coordinate
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_in_c(
      new pcl::PointCloud<pcl::PointXYZ>());
  // 将点云有激光坐标系下point_in_l, 转换到相机坐标系下point_in_c
  pcl::transformPointCloud(*point_in_l, *point_in_c, T_cl);

  // Visualization
  // viewer1 是点云在激光坐标系下, 同时生成相机的坐标系位置
  pcl::visualization::PCLVisualizer viewer1("Point Cloud in Lidar Coordinate");

  // 激光坐标系下的原始点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      point_in_l_color_handler(point_in_l, 255, 255, 255);
  // We add the point cloud to the viewer1 and pass the color handler
  viewer1.addPointCloud(point_in_l, point_in_l_color_handler, "original_cloud");

  // lidar坐标系
  viewer1.addCoordinateSystem(1.0, "lidar", 0);
  // 相机坐标系
  viewer1.addCoordinateSystem(1.0, T_lc, "camera", 0);
  // Setting background to a dark grey
  viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer1.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

  pcl::visualization::PCLVisualizer viewer2("Point Cloud in Camera Coordinate");

  // 相机坐标系下的激光点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      point_in_c_color_handler(point_in_c, 230, 20, 20);  // Red

  viewer2.addPointCloud(point_in_c, point_in_c_color_handler, "point_in_c");
  viewer2.addCoordinateSystem(1.0, "camera", 0);
  viewer2.addCoordinateSystem(1.0, T_cl, "lidar", 0);

  viewer2.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point_in_c");

  // Display the visualiser until 'q' key is pressed
  while (!viewer1.wasStopped() && !viewer2.wasStopped()) {
    viewer1.spinOnce();
    // viewer2.spinOnce();
  }

  return 0;
}
