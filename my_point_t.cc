#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
// #include <pcl/memory.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// enforce SSE padding for correct memory alignment
struct EIGEN_ALIGN16 MyPointType {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float test;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    MyPointType,  // here we assume a XYZ + "test" (as fields)
    (float, x, x)(float, y, y)(float, z, z)(float, test, test))

int main(int argc, char** argv) {
  pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>());
  cloud->resize(2);
  cloud->width = 2;
  cloud->height = 1;

  cloud->at(0).test = 1;
  cloud->at(1).test = 2;
  cloud->at(0).x = cloud->at(0).y = cloud->at(0).z = 0;
  cloud->at(1).x = cloud->at(1).y = cloud->at(1).z = 3;

  pcl::io::savePCDFile("test.pcd", *cloud);

  pcl::VoxelGrid<MyPointType> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);

  pcl::PointCloud<MyPointType>::Ptr cloud22(new pcl::PointCloud<MyPointType>());
  sor.filter(*cloud22);

  std::cout << "filtered size: " << cloud22->size() << std::endl;
}