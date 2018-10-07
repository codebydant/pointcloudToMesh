/*********************************
           HEADERS
**********************************/
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply/ply.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/gp3.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/gasd.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/filters/crop_hull.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <fstream>
#include <string>

void printUsage (const char* progName){
  std::cout << "\nUsage: " << progName << " <file.ply> -o <output dir>"  << std::endl;
}

/*

void create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh){

pcl::console::print_info("Creating mesh...pleas wait.");
pcl::console::TicToc tt;

pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
ne.setNumberOfThreads (8);
// ne.setInputCloud (cloud_smoothed);
ne.setInputCloud (cloud);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
ne.setSearchMethod (tree);
ne.setKSearch (10); //20
//ne.setRadiusSearch (0.5); // no funciona
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
ne.compute(*cloud_normals);

for(std::size_t i = 0; i < cloud_normals->size (); ++i){
  cloud_normals->points[i].normal_x *= -1;
  cloud_normals->points[i].normal_y *= -1;
  cloud_normals->points[i].normal_z *= -1;
}
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
pcl::concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);//x

pcl::Poisson<pcl::PointNormal> poisson;

poisson.setDepth(7);//9
poisson.setInputCloud (cloud_smoothed_normals);
poisson.setPointWeight(2);//4
//poisson.setDegree(5);
poisson.setSamplesPerNode(1.5);//1.5
poisson.setScale(1.1);//1.1
poisson.setIsoDivide(8);//8
poisson.setConfidence(1);
poisson.setOutputPolygons(true);
poisson.setManifold(0);
//poisson.setOutputPolygons(0);
poisson.setSolverDivide(8);//8
pcl::PolygonMesh mesh2;
poisson.reconstruct(mesh2);

pcl::surface::SimplificationRemoveUnusedVertices rem;
rem.simplify(mesh2,mesh);

pcl::console::print_info ("[done, ");
pcl::console::print_value ("%g", tt.toc ());
pcl::console::print_info (" ms : ");
pcl::console::print_value ("%d", mesh.cloud.width);
pcl::console::print_info (" points]\n");
}

*/
 static void create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh,int nThreads=8,
                           int setKsearch=10, int depth=10, float pointWeight=4.0,float samplePNode=1.5,
                           float scale=1.0,int isoDivide=5, bool confidence=true, bool outputPolygons=true,
                           bool manifold=true,int solverDivide=5){

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  std::cout << centroid << std::endl;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  std::cout << transform.matrix() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *cloudTranslated, transform);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (10);

    seg.setInputCloud (cloudTranslated);
    seg.segment (*inliers, *coefficients);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloudTranslated);
    proj.setIndices (inliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    std::cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
    std::cout << "Convex hull volume:" << chull.getTotalVolume() << std::endl;

    pcl::PolygonMesh ms2;

 //   chull.performReconstruction(*cloud_hull,ms2);

    pcl::PCDWriter writer;
    writer.write ("cloud_convex_hull.pcd", *cloud_hull, false);
   // pcl::io::savePolygonFilePLY("cloud_convex_hull2.ply", ms2, false);


  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

//  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//  ne.setNumberOfThreads(nThreads);
//  ne.setInputCloud(cloud);

//  ne.setSearchMethod (tree);
//  ne.setKSearch(setKsearch); //20
//  //ne.setRadiusSearch (0.5); // no funciona
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
//  ne.compute(*cloud_normals);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_mls (new pcl::PointCloud<pcl::PointXYZ>());
//  const double radius;




//    pcl::MovingLeastSquares<T, T> mls;
//    mls.setComputeNormals(false);
//    mls.setInputCloud(cloud);
//    mls.setPolynomialFit(true);
//    mls.setSearchMethod(get_search_kdtree<T>(cloud));
//    mls.setSearchRadius(radius);
//    mls.process(*smoothed);

//  for(std::size_t i = 0; i < mls_points->size (); ++i){
//    cloud_normals->points[i].normal_x *= -1;
//    cloud_normals->points[i].normal_y *= -1;
//    cloud_normals->points[i].normal_z *= -1;
//  }

  //cropH->filter(indices);
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>());

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloudTranslated);
  mls.setDilationIterations(10);
  //mls.setDilationVoxelSize(0.5);
  //mls.setSqrGaussParam(2.0);
  //mls.setUpsamplingRadius(5);
  mls.setPointDensity(30);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(5);

  // Reconstruct
  mls.process (*mls_points);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloudTranslated, *mls_points, *cloud_smoothed_normals);//x

  pcl::Poisson<pcl::PointNormal> poisson;

  poisson.setDepth(depth);//9
  poisson.setInputCloud(cloud_smoothed_normals);
  poisson.setPointWeight(pointWeight);//4
  poisson.setDegree(2);
  poisson.setSamplesPerNode(samplePNode);//1.5
  poisson.setScale(scale);//1.1
  poisson.setIsoDivide(isoDivide);//8
  poisson.setConfidence(confidence);
  poisson.setOutputPolygons(outputPolygons);
  poisson.setManifold(manifold);
  poisson.setSolverDivide(solverDivide);//8

  pcl::PolygonMesh mesh2;
  poisson.reconstruct(mesh2);

  pcl::surface::SimplificationRemoveUnusedVertices rem;
  rem.simplify(mesh2,mesh);



/*
  std::string out = output_dir;
  out += "/3D_Mapping/mesh.ply";

  std::cout << "\nSaving mesh:" << out << std::endl;

  pcl::io::savePolygonFilePLY(out.c_str(),mesh);
  */
  
}

 void createMeshFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PolygonMesh& triangles){

     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

     Eigen::Vector4f centroid;
     pcl::compute3DCentroid(*cloud, centroid);
     std::cout << centroid << std::endl;

     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
     transform.translation() << -centroid[0], -centroid[1], -centroid[2];

     std::cout << transform.matrix() << std::endl;

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
     pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

     tree->setInputCloud (cloudTranslated);
     n.setInputCloud (cloudTranslated);
     n.setSearchMethod (tree);
     n.setKSearch (100);        //It was 20
     n.compute (*normals);                //Normals are estimated using standard method.

     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);//x

     // Create search tree*
     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
     tree2->setInputCloud (cloud_with_normals);

     // Initialize objects
     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

     std::cout << "Applying surface meshing...";

     // Set the maximum distance between connected points (maximum edge length)
     gp3.setSearchRadius(10);           //It was 0.025

     // Set typical values for the parameters
     gp3.setMu (5); //It was 2.5
     gp3.setMaximumNearestNeighbors (100);    //It was 100
     //gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
     //gp3.setMinimumAngle(M_PI/18); // 10 degrees //It was 18
     //gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
     gp3.setNormalConsistency(false); //It was false

     // Get result
     gp3.setInputCloud (cloud_with_normals);
     gp3.setSearchMethod (tree2);
     gp3.reconstruct (triangles);

     vtkSmartPointer<vtkPolyData> polydata= vtkSmartPointer<vtkPolyData>::New();

     pcl::PolygonMesh mms2;

     pcl::VTKUtils::convertToVTK(triangles,polydata);
     pcl::VTKUtils::convertToPCL(polydata,mms2);

     pcl::io::savePolygonFilePLY("mesh2.ply", mms2);

 }

void vizualizeMesh(pcl::PolygonMesh &mesh){

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));
viewer->setBackgroundColor (0, 0, 0);
viewer->addPolygonMesh(mesh,"meshes",0);
viewer->addCoordinateSystem (1.0);
viewer->initCameraParameters ();
viewer->resetCamera();

std::cout << "Press [q] to exit!" << std::endl;
while (!viewer->wasStopped ()){
    viewer->spin();
}
}

void cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& filterCloud){

  std::cout << "Filtering point cloud..." << std::endl;
  std::cout << "Point cloud before filter:" << cloud->points.size()<< std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
  radius_outlier_removal.setInputCloud(cloud);
  radius_outlier_removal.setRadiusSearch(0.01);
  radius_outlier_removal.setMinNeighborsInRadius(1);
  radius_outlier_removal.filter(*filterCloud);

  std::cout << "Point cloud after filter:" << filterCloud->points.size() << std::endl;
}

using namespace std;

int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;

  if(argc < 4 or argc > 4){
      printUsage (argv[0]);
      return -1;
  }

  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if(filenames.size()<=0){
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()<=0){
          filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
          if(filenames.size()<=0){
              filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
              if(filenames.size()<=0){
                  printUsage (argv[0]);
                  return -1;
              }else if(filenames.size() == 1){
                  file_is_xyz = true;
              }
          }else if(filenames.size() == 1){
             file_is_txt = true;
        }
    }else if(filenames.size() == 1){
          file_is_pcd = true;
    }
  }
  else if(filenames.size() == 1){
      file_is_ply = true;
  }else{
      printUsage (argv[0]);
      return -1;
  }

  std::string output_dir;
  if(pcl::console::parse_argument(argc, argv, "-o", output_dir) == -1){
      PCL_ERROR ("Need an output directory! Please use <input cloud> -o <output dir>\n");
      return(-1);
  }

  if(file_is_pcd){ 
      if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0){
              std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
              return -1;
      }
      pcl::console::print_info("\nFound pcd file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_ply){
      pcl::io::loadPLYFile(argv[filenames[0]],*cloud);
      if(cloud->points.size()<=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *cloud);
          if(cloud->points.size()<=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*cloud);
              if(cloud->points.size()<=0){
                  pcl::console::print_error("\nError. ply file is not compatible.\n");
                  return -1;
              }
          }
       }

      pcl::console::print_info("\nFound ply file.");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");

    }else if(file_is_txt or file_is_xyz){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      double x_,y_,z_;
      while(file >> x_ >> y_ >> z_){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;
          cloud->points.push_back(pt);
      }

      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
  }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,*cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new pcl::PointCloud<pcl::PointXYZ>());
  cloudPointFilter(cloud_xyz,cloud_xyz_filtered);

  pcl::PolygonMesh cloud_mesh;
  createMeshFromCloud(cloud_xyz,cloud_mesh);

  output_dir += "/cloudConvertedMESH";

  std::string sav = "saved mesh in:";
  sav += output_dir;

  pcl::console::print_info(sav.c_str());
  std::cout << std::endl;
  pcl::io::savePolygonFilePLY(output_dir.c_str(),cloud_mesh,true);

  vizualizeMesh(cloud_mesh);
   
  return 0;
}

