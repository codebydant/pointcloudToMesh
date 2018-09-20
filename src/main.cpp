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

#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/gasd.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

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
  create_mesh(cloud_xyz_filtered,cloud_mesh);

  output_dir += "/cloudConvertedMESH";

  std::string sav = "saved mesh in:";
  sav += output_dir;

  pcl::console::print_info(sav.c_str());
  std::cout << std::endl;
  pcl::io::savePolygonFilePLY(output_dir.c_str(),cloud_mesh,true);

  vizualizeMesh(cloud_mesh);
   
  return 0;
}

