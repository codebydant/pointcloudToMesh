#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void vizualizeMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PolygonMesh &mesh) {
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("MAP3D MESH"));
  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor(0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor(0, 0, 0, PORT2);
  viewer->addText("MESH", 10, 10, "PORT2", PORT2);
  viewer->addPolygonMesh(mesh, "mesh", PORT2);

  viewer->addCoordinateSystem();
  pcl::PointXYZ p1, p2, p3;

  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0, 0.1, 1;

  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer->addText3D("z", p3, 0.2, 0, 0, 1, "z_");

  if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
    viewer->removeAllPointClouds(0);
    viewer->addPointCloud(cloud, color_handler, "original_cloud", PORT1);
  } else {
    viewer->addPointCloud(cloud, "original_cloud", PORT1);
  }

  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "Press [q] to exit!" << std::endl;
  while (!viewer->wasStopped()) {
    viewer->spin();
  }
}