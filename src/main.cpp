#include <boost/filesystem.hpp>

#include "argparse/argparse.hpp"
#include "cloudparse/parser.hpp"
#include "surface.hpp"
#include "visualizer.hpp"

int main(int argc, char **argv) {
  argparse::ArgumentParser arg_parser(argv[0]);

  arg_parser.add_argument("--file").required().help("input cloud file");
  arg_parser.add_argument("--surface").default_value(int(2)).scan<'i', int>().help("surface method 1: poisson 2: gp3");
  arg_parser.add_argument("--normal")
      .default_value(int(1))
      .scan<'i', int>()
      .help("normal estimation method 1:normal 2: mls");
  arg_parser.add_argument("-o", "--output-dir").required().help("output dir to save mesh file");
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

  // parse data from command line
  int normal_method = arg_parser.get<int>("--normal");
  int surface_mode = arg_parser.get<int>("--surface");
  std::string filename = arg_parser.get<std::string>("--file");
  std::string output_dir = arg_parser.get<std::string>("--output-dir");

  // check if output dir is path
  boost::filesystem::path dirPath(output_dir);
  if (not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)) {
    pcl::console::print_error("\nError. does not exist or it's not valid: ");
    std::cout << output_dir << std::endl;
    std::exit(-1);
  }

  // -----------------Read input cloud file -----------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  // cloud parser object
  CloudParserLibrary::ParserCloudFile cloud_parser;
  cloud_parser.load_cloudfile(filename, cloud);

  // set cloud metadata
  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  // -----------------Compute mesh surface -----------------
  pcl::PolygonMesh cloud_mesh;
  create_mesh(cloud, normal_method, surface_mode, cloud_mesh);

  // export mesh file
  dirPath /= "cloud_mesh.ply";
  pcl::io::savePLYFileBinary(dirPath.c_str(), cloud_mesh);
  pcl::console::print_info("saved mesh in: %s \n", dirPath.c_str());

  // -----------------Visualize mesh -----------------
  if (arg_parser["--display"] == true) {
    // Disable vtk render warning
    vtkObject::GlobalWarningDisplayOff();
    vizualizeMesh(cloud, cloud_mesh);
  }

  return 0;
}
