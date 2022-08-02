/*********************************
           HEADERS
**********************************/

#include <pcl/io/file_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <pcl/range_image/range_image.h>

#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/features/gasd.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <boost/algorithm/algorithm.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include "argparse/argparse.hpp"
#include <fstream>
#include <iostream>
#include <string>

void printUsage(const char *progName) {
  std::cout << "\nUsage: " << progName
            << " <input cloud> <surface method> <normal estimation method> "
               "<output dir>"
            << std::endl;
  std::cout << "surface method: \n '1' for poisson \n '2' for gp3" << std::endl;
  std::cout << "normal estimation method: \n '1' for normal estimation \n '2' "
               "for mls normal estimation"
            << std::endl;
}

void create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int &surface_mode,
                 int &normal_method, pcl::PolygonMesh &triangles) {

  /* ****Translated point cloud to origin**** */
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

  /* ****kdtree search and msl object**** */
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_points(
      new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree_for_points->setInputCloud(cloudTranslated);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());

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

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(
        new pcl::PointCloud<pcl::PointNormal>());
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(
        new pcl::PointCloud<pcl::PointXYZ>());

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
    n.setKSearch(20);    // It was 20
    n.compute(*normals); // Normals are estimated using standard method.

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
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree_for_normals(
      new pcl::search::KdTree<pcl::PointNormal>);
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

    gp3.setSearchRadius(search_radius);                   // It was 0.025
    gp3.setMu(setMU);                                     // It was 2.5
    gp3.setMaximumNearestNeighbors(maxiNearestNeighbors); // It was 100
    // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
    // gp3.setMinimumAngle(M_PI/18); // 10 degrees //It was 18
    // gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
    gp3.setNormalConsistency(normalConsistency); // It was false
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

    poisson.setDepth(depth); // 9
    poisson.setInputCloud(cloud_with_normals);
    poisson.setPointWeight(pointWeight); // 4
    poisson.setDegree(2);
    poisson.setSamplesPerNode(samplePNode); // 1.5
    poisson.setScale(scale);                // 1.1
    poisson.setIsoDivide(isoDivide);        // 8
    poisson.setConfidence(confidence);
    poisson.setOutputPolygons(outputPolygons);
    poisson.setManifold(manifold);
    poisson.setSolverDivide(solverDivide); // 8
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

void vizualizeMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   pcl::PolygonMesh &mesh) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("MAP3D MESH"));

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

  if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and
      cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
        color_handler(cloud, 255, 255, 0);
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
/*
void cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &
cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& filterCloud){

  std::cout << "Filtering point cloud..." << std::endl;
  std::cout << "Point cloud before filter:" << cloud->points.size()<< std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
  radius_outlier_removal.setInputCloud(cloud);
  radius_outlier_removal.setRadiusSearch(0.01);
  radius_outlier_removal.setMinNeighborsInRadius(1);
  radius_outlier_removal.filter(*filterCloud);

  std::cout << "Point cloud after filter:" << filterCloud->points.size() <<
std::endl;
}
*/

int main(int argc, char **argv) {

  argparse::ArgumentParser arg_parser(argv[0]);

  arg_parser.add_argument("--file").required().help("input cloud file");
  arg_parser.add_argument("--surface")
      .default_value(int(2))
      .scan<'i', int>()
      .help("surface method 1: poisson 2: gp3");
  arg_parser.add_argument("--normal")
      .default_value(int(1))
      .scan<'i', int>()
      .help("normal estimation method 1:normal 2: mls");
  arg_parser.add_argument("-o", "--output-dir")
      .default_value(std::string("-"))
      .help("output dir to save mesh file");
  arg_parser.add_argument("-d", "--display")
      .default_value(false)
      .implicit_value(true)
      .help("display mesh in the pcl visualizer");

  try {
    arg_parser.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << arg_parser;
    std::exit(0);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = true;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;

  pcl::console::TicToc tt;
  pcl::console::print_highlight("Loading ");

  std::string filename = arg_parser.get<std::string>("--file");

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile(filename, *cloud) < 0) {
      std::cout << "Error loading point cloud " << filename << "\n";
      return -1;
    }
    pcl::console::print_info("\nFound pcd file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->size());
    pcl::console::print_info(" points]\n");
  } else if (file_is_ply) {
    pcl::io::loadPLYFile(filename, *cloud);
    if (cloud->points.size() <= 0 or cloud->points[0].x <= 0 and
                                         cloud->points[0].y <= 0 and
                                         cloud->points[0].z <= 0) {
      pcl::console::print_warn("\nloadPLYFile could not read the cloud, "
                               "attempting to loadPolygonFile...\n");
      pcl::io::loadPolygonFile(filename, cl);
      pcl::fromPCLPointCloud2(cl.cloud, *cloud);
      if (cloud->points.size() <= 0 or cloud->points[0].x <= 0 and
                                           cloud->points[0].y <= 0 and
                                           cloud->points[0].z <= 0) {
        pcl::console::print_warn("\nloadPolygonFile could not read the cloud, "
                                 "attempting to PLYReader...\n");
        pcl::PLYReader plyRead;
        plyRead.read(filename, *cloud);
        if (cloud->points.size() <= 0 or cloud->points[0].x <= 0 and
                                             cloud->points[0].y <= 0 and
                                             cloud->points[0].z <= 0) {
          pcl::console::print_error("\nError. ply file is not compatible.\n");
          return -1;
        }
      }
    }

    pcl::console::print_info("\nFound ply file.");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->size());
    pcl::console::print_info(" points]\n");

  } else if (file_is_txt) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      std::cout << "Error: Could not find " << filename << std::endl;
      return -1;
    }

    std::cout << "file opened." << std::endl;
    double x_, y_, z_;
    unsigned int r, g, b;

    while (file >> x_ >> y_ >> z_ >> r >> g >> b) {
      pcl::PointXYZRGB pt;
      pt.x = x_;
      pt.y = y_;
      pt.z = z_;

      uint8_t r_, g_, b_;
      r_ = uint8_t(r);
      g_ = uint8_t(g);
      b_ = uint8_t(b);

      uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_);
      pt.rgb = *reinterpret_cast<float *>(&rgb_);

      cloud->points.push_back(pt);
      // std::cout << "pointXYZRGB:" <<  pt << std::endl;
    }

    pcl::console::print_info("\nFound txt file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->points.size());
    pcl::console::print_info(" points]\n");

  } else if (file_is_xyz) {

    std::ifstream file(filename);
    if (!file.is_open()) {
      std::cout << "Error: Could not find " << filename << std::endl;
      return -1;
    }

    std::cout << "file opened." << std::endl;
    double x_, y_, z_;

    while (file >> x_ >> y_ >> z_) {

      pcl::PointXYZRGB pt;
      pt.x = x_;
      pt.y = y_;
      pt.z = z_;

      cloud->points.push_back(pt);
      // std::cout << "pointXYZRGB:" <<  pt << std::endl;
    }

    pcl::console::print_info("\nFound xyz file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->points.size());
    pcl::console::print_info(" points]\n");
  }

  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  std::string output_dir = arg_parser.get<std::string>("--output-dir");

  int surface_mode = arg_parser.get<int>("--surface");
  int normal_method = arg_parser.get<int>("--normal");

  boost::filesystem::path dirPath(output_dir);

  if (not boost::filesystem::exists(dirPath) or
      not boost::filesystem::is_directory(dirPath)) {
    pcl::console::print_error("\nError. does not exist or it's not valid: ");
    std::cout << output_dir << std::endl;
    std::exit(-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new
  // pcl::PointCloud<pcl::PointXYZ>());
  // cloudPointFilter(cloud_xyz,cloud_xyz_filtered);

  pcl::PolygonMesh cloud_mesh;
  create_mesh(cloud_xyz, surface_mode, normal_method, cloud_mesh);

  output_dir += "/cloud_mesh.ply";

  std::string sav = "saved mesh in:";
  sav += output_dir;

  // typedef pcl::geometry::DefaultMeshTraits <>      MeshTraits;
  // typedef pcl::geometry::TriangleMesh <MeshTraits> Mesh;
  // typedef pcl::geometry::MeshIO <Mesh>             MeshIO;

  pcl::console::print_info(sav.c_str());
  std::cout << std::endl;

  pcl::io::savePLYFileBinary(output_dir.c_str(), cloud_mesh);

  vtkObject::GlobalWarningDisplayOff(); // Disable vtk render warning
  vizualizeMesh(cloud, cloud_mesh);

  return 0;
}
