#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

/*
estimation method to calculate normals
supports normal or mls estimation
*/
void set_normal_est_method(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int &method,
                           pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals) {
  // Translated point cloud to origin
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_points(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree_for_points->setInputCloud(cloudTranslated);

  const int normal_mode = 1, mls_mode = 2;
  switch (method) {
    case normal_mode: {
      std::cout << "Using normal method estimation...";

      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

      n.setInputCloud(cloudTranslated);
      n.setSearchMethod(kdtree_for_points);
      n.setKSearch(20);     // It was 20
      n.compute(*normals);  // Normals are estimated using standard method.
      pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
      break;
    }
    case mls_mode: {
      std::cout << "Using mls method estimation...";

      pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>());
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

      // parameters
      mls.setComputeNormals(true);
      mls.setInputCloud(cloudTranslated);
      mls.setSearchMethod(kdtree_for_points);
      mls.setSearchRadius(0.03);
      mls.process(*mls_points);
      // mls.setDilationIterations(10);
      // mls.setDilationVoxelSize(0.5);
      // mls.setSqrGaussParam(2.0);
      // mls.setUpsamplingRadius(5);
      // mls.setPolynomialOrder (2);
      // mls.setPointDensity(30);

      pcl::concatenateFields(*cloud, *mls_points, *cloud_with_normals);
      break;
    }
  }
  std::cout << "[OK]" << std::endl;
}

/*
estimation method to calculate surface
supports poisson or gp3 estimation
*/
void set_surface_est_method(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals, int &method,
                            pcl::PolygonMesh &triangles) {
  std::cout << "Applying surface meshing...";

  const int poisson_mode = 1, gp3_mode = 2;
  switch (method) {
    case poisson_mode: {
      // https://pointclouds.org/documentation/classpcl_1_1_poisson.html
      std::cout << "Using surface method: poisson ..." << std::endl;

      int nThreads = 8;
      int setKsearch = 10;
      int depth = 7;
      float pointWeight = 2.0;
      float samplePNode = 1.5;
      float scale = 1.1;
      int isoDivide = 8;
      bool confidence = true;
      bool outputPolygons = true;
      bool manifold = true;
      int solverDivide = 8;

      pcl::Poisson<pcl::PointNormal> poisson;

      poisson.setDepth(depth);                  // 9
      poisson.setInputCloud(cloud_with_normals);
      poisson.setPointWeight(pointWeight);      // 4
      poisson.setDegree(2);
      poisson.setSamplesPerNode(samplePNode);   // 1.5
      poisson.setScale(scale);                  // 1.1
      poisson.setIsoDivide(isoDivide);          // 8
      poisson.setConfidence(confidence);
      poisson.setOutputPolygons(outputPolygons);
      poisson.setManifold(manifold);
      poisson.setSolverDivide(solverDivide);    // 8
      poisson.reconstruct(triangles);

      break;
    }
    case gp3_mode: {
      // https://pointclouds.org/documentation/classpcl_1_1_greedy_projection_triangulation.html
      std::cout << "Using surface method: gp3 ..." << std::endl;

      int searchK = 100;
      int search_radius = 10;
      int setMU = 5;
      int maxiNearestNeighbors = 100;
      bool normalConsistency = false;

      // Create search tree
      pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree_for_normals(new pcl::search::KdTree<pcl::PointNormal>);
      kdtree_for_normals->setInputCloud(cloud_with_normals);

      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

      gp3.setSearchRadius(search_radius);                    // It was 0.025
      gp3.setMu(setMU);                                      // It was 2.5
      gp3.setMaximumNearestNeighbors(maxiNearestNeighbors);  // It was 100
      gp3.setNormalConsistency(normalConsistency);           // It was false
      gp3.setInputCloud(cloud_with_normals);
      gp3.setSearchMethod(kdtree_for_normals);
      gp3.reconstruct(triangles);
      // gp3.setMaximumSurfaceAngle(M_PI/4);                  // 45 degrees
      // gp3.setMinimumAngle(M_PI/18);                        // 10 degrees
      // gp3.setMaximumAngle(M_PI/1.5);                       // 120 degrees

      break;
    }
  }
  std::cout << "[OK]" << std::endl;
}

void create_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int &normal_mode, int &surface_mode,
                 pcl::PolygonMesh &triangles) {
  // convert PointXYZRGB to PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new pcl::PointCloud<pcl::PointXYZ>());
  // cloudPointFilter(cloud_xyz, cloud_xyz_filtered);

  // estimate normals
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  set_normal_est_method(cloud_xyz, normal_mode, cloud_with_normals);

  // calculate surface
  set_surface_est_method(cloud_with_normals, surface_mode, triangles);
}
