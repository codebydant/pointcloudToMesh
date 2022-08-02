#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

void create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int &surface_mode, int &normal_method,
                 pcl::PolygonMesh &triangles) {
  /* ****Translated point cloud to origin**** */
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

  /* ****kdtree search and msl object**** */
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_points(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree_for_points->setInputCloud(cloudTranslated);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());

  bool mls_mode = false;
  bool normal_mode = false;

  if (normal_method == 1) {
    normal_mode = true;
  } else if (normal_method == 2) {
    mls_mode = true;
  } else {
    std::cout << "Select:\n '1' for normal estimation \n '2' for mls normal "
                 "estimation "
              << std::endl;
    std::exit(-1);
  }

  bool gp3_mode = false;
  bool poisson_mode = false;

  if (surface_mode == 1) {
    poisson_mode = true;
  } else if (surface_mode == 2) {
    gp3_mode = true;
  } else {
    std::cout << "Select: \n'1' for surface poisson method \n '2' for surface "
                 "gp3 method "
              << std::endl;
    std::exit(-1);
  }

  if (mls_mode) {
    std::cout << "Using mls method estimation...";

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    // Set parameters
    mls.setComputeNormals(true);
    mls.setInputCloud(cloudTranslated);
    // mls.setDilationIterations(10);
    // mls.setDilationVoxelSize(0.5);
    // mls.setSqrGaussParam(2.0);
    // mls.setUpsamplingRadius(5);
    // mls.setPolynomialOrder (2);
    // mls.setPointDensity(30);
    mls.setSearchMethod(kdtree_for_points);
    mls.setSearchRadius(0.03);
    mls.process(*mls_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < mls_points->points.size(); i++) {
      pcl::PointXYZ pt;
      pt.x = cloud->points[i].x;
      pt.y = cloud->points[i].y;
      pt.z = cloud->points[i].z;

      temp->points.push_back(pt);
    }

    pcl::concatenateFields(*temp, *mls_points, *cloud_with_normals);
    std::cout << "[OK]" << std::endl;

  } else if (normal_mode) {
    std::cout << "Using normal method estimation...";

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    n.setInputCloud(cloudTranslated);
    n.setSearchMethod(kdtree_for_points);
    n.setKSearch(20);     // It was 20
    n.compute(*normals);  // Normals are estimated using standard method.

    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new
    // pcl::PointCloud<pcl::PointNormal> ());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    std::cout << "[OK]" << std::endl;

  } else {
    std::cout << "Select: '1' for normal method estimation \n '2' for mls "
                 "normal estimation "
              << std::endl;
    std::exit(-1);
  }

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree_for_normals(new pcl::search::KdTree<pcl::PointNormal>);
  kdtree_for_normals->setInputCloud(cloud_with_normals);

  std::cout << "Applying surface meshing...";

  if (gp3_mode) {
    std::cout << "Using surface method: gp3 ..." << std::endl;

    int searchK = 100;
    int search_radius = 10;
    int setMU = 5;
    int maxiNearestNeighbors = 100;
    bool normalConsistency = false;

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    gp3.setSearchRadius(search_radius);                    // It was 0.025
    gp3.setMu(setMU);                                      // It was 2.5
    gp3.setMaximumNearestNeighbors(maxiNearestNeighbors);  // It was 100
    // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
    // gp3.setMinimumAngle(M_PI/18); // 10 degrees //It was 18
    // gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
    gp3.setNormalConsistency(normalConsistency);  // It was false
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(kdtree_for_normals);
    gp3.reconstruct(triangles);

    // vtkSmartPointer<vtkPolyData> polydata =
    // vtkSmartPointer<vtkPolyData>::New(); pcl::PolygonMesh mesh_pcl;
    // pcl::VTKUtils::convertToVTK(triangles,polydata);
    // pcl::VTKUtils::convertToPCL(polydata,mesh_pcl);

    // pcl::io::savePolygonFilePLY("mesh.ply", mesh_pcl);

    std::cout << "[OK]" << std::endl;

  } else if (poisson_mode) {
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

    poisson.setDepth(depth);  // 9
    poisson.setInputCloud(cloud_with_normals);
    poisson.setPointWeight(pointWeight);  // 4
    poisson.setDegree(2);
    poisson.setSamplesPerNode(samplePNode);  // 1.5
    poisson.setScale(scale);                 // 1.1
    poisson.setIsoDivide(isoDivide);         // 8
    poisson.setConfidence(confidence);
    poisson.setOutputPolygons(outputPolygons);
    poisson.setManifold(manifold);
    poisson.setSolverDivide(solverDivide);  // 8
    poisson.reconstruct(triangles);

    // pcl::PolygonMesh mesh2;
    // poisson.reconstruct(mesh2);
    // pcl::surface::SimplificationRemoveUnusedVertices rem;
    // rem.simplify(mesh2,triangles);

    std::cout << "[OK]" << std::endl;

  } else {
    std::cout << "Select: \n'1' for surface poisson method \n '2' for surface "
                 "gp3 method "
              << std::endl;
    std::exit(-1);
  }
}