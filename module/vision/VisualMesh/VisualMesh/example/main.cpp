#include <dirent.h>
#include <fcntl.h>
#include <fmt/format.h>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <system_error>
#include "ArrayPrint.hpp"
#include "Timer.hpp"
#include "engine/opencl/opencl_engine.hpp"
#include "geometry/Sphere.hpp"
#include "util/fourcc.hpp"
#include "visualmesh.hpp"

// List the contents of a directory
std::vector<std::string> listdir(const std::string& path) {

  auto dir = ::opendir(path.c_str());
  std::vector<std::string> result;

  if (dir != nullptr) {
    for (dirent* ent = readdir(dir); ent != nullptr; ent = readdir(dir)) {

      auto file = std::string(ent->d_name);

      if (file == "." || file == "..") { continue; }

      if (ent->d_type & DT_DIR) { result.push_back(file + "/"); }
      else {
        result.push_back(file);
      }
    }

    closedir(dir);
  }
  else {
    throw std::system_error(errno, std::system_category(), "Failed to open directory " + path);
  }

  return result;
}

int main() {

  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  // Input image path
  std::string image_path = "../example/images";
  std::string model_path = "../example/model.yaml";

  // Construct our VisualMesh
  Timer t;
  visualmesh::geometry::Sphere<float> sphere(0.075, 4, 10);
  visualmesh::VisualMesh<float, visualmesh::engine::opencl::Engine> cl_mesh(sphere, 0.5, 1.5, 100);
  visualmesh::VisualMesh<float, visualmesh::engine::cpu::Engine> cpu_mesh(sphere, 0.5, 1.5, 100);
  t.measure("Built Visual Mesh");

  // Build our classification network
  std::vector<std::vector<std::pair<std::vector<std::vector<float>>, std::vector<float>>>> network;

  YAML::Node config = YAML::LoadFile(model_path);

  for (const auto& conv : config) {

    // New conv layer
    network.emplace_back();
    auto& net_conv = network.back();

    for (const auto& layer : conv) {

      // New network layer
      net_conv.emplace_back();
      auto& net_layer = net_conv.back();

      // Copy across our weights
      for (const auto& l : layer["weights"]) {
        net_layer.first.emplace_back();
        auto& weight = net_layer.first.back();

        for (const auto& v : l) {
          weight.push_back(v.as<double>());
        }
      }

      // Copy across our biases
      for (const auto& v : layer["biases"]) {
        net_layer.second.push_back(v.as<double>());
      }
    }
  }

  // Make a classifier for this network
  auto cl_classifier  = cl_mesh.make_classifier(network);
  auto cpu_classifier = cpu_mesh.make_classifier(network);

  // Go through all our training data
  std::cerr << "Looping through training data" << std::endl;
  auto files = listdir(image_path);
  std::sort(files.begin(), files.end());

  for (const auto& p : files) {
    if (p.substr(0, 4) == "meta") {

      // Extract the number so we can find the other files
      auto number = p.substr(4, 7);

      std::cerr << "Processing file " << number << std::endl;

      Timer t;

      // Load our metadata and two images
      YAML::Node meta = YAML::LoadFile(image_path + "/" + p);
      cv::Mat img     = cv::imread(image_path + "/image" + number + ".jpg");

      {
        std::vector<cv::Mat> split;
        cv::split(img, split);

        split.push_back(split.back());

        cv::merge(split, img);
      }

      cv::Mat stencil = cv::imread(image_path + "/stencil" + number + ".png");

      t.measure("\tLoaded files");

      // Construct our rotation matrix from the camera angles
      auto r       = meta["camera"]["rotation"];
      float height = meta["camera"]["height"].as<float>();

      // Oh no! the coordinate systems are wrong!
      // We are expecting
      //      x forward
      //      y to the left
      //      z up
      // However blender's camera objects have
      //      z facing away from the object,
      //      y up
      //      x to the right
      // So to fix this we have to make x = -z, y = -x, z = y (swap cols)
      std::array<std::array<float, 4>, 4> Hoc = {{
        {{-float(r[0][2].as<float>()), -float(r[0][0].as<float>()), r[0][1].as<float>(), 0}},
        {{-float(r[1][2].as<float>()), -float(r[1][0].as<float>()), r[1][1].as<float>(), 0}},
        {{-float(r[2][2].as<float>()), -float(r[2][0].as<float>()), r[2][1].as<float>(), height}},
        {{0, 0, 0, 1}},
      }};

      // Make our lens object
      visualmesh::Lens<float> lens;
      lens.dimensions = {{img.size().width, img.size().height}};
      lens.centre     = {{0.0, 0.0}};
      if (meta["camera"]["lens"]["type"].as<std::string>() == "PERSPECTIVE") {

        // Horizontal field of view
        float h_fov = meta["camera"]["lens"]["fov"].as<float>();

        // Construct rectilinear projection
        lens.projection   = visualmesh::RECTILINEAR;
        lens.fov          = h_fov;
        lens.focal_length = (lens.dimensions[0] * 0.5) / std::tan(h_fov * 0.5);
      }
      else if (meta["camera"]["lens"]["type"].as<std::string>() == "FISHEYE") {
        float fov             = meta["camera"]["lens"]["fov"].as<float>();
        float height_mm       = meta["camera"]["lens"]["sensor_height"].as<float>();
        float width_mm        = meta["camera"]["lens"]["sensor_width"].as<float>();
        float focal_length_mm = meta["camera"]["lens"]["focal_length"].as<float>();

        // Get conversion from mm to pixels
        float sensor_density = lens.dimensions[0] / width_mm;

        // Blender was rendered with an equisolid lens type
        lens.projection   = visualmesh::EQUISOLID;
        lens.fov          = fov;
        lens.focal_length = focal_length_mm * sensor_density;
      }
      t.measure("\tSetup Lens");

      // Run our classifier
      {
        auto classified = cl_classifier(cl_mesh.height(Hoc[2][3]), img.data, visualmesh::fourcc("BGRA"), Hoc, lens);

        auto& neighbourhood                                 = classified.neighbourhood;
        std::vector<std::array<float, 2>> pixel_coordinates = classified.pixel_coordinates;
        auto classifications                                = classified.classifications;

        t.measure("\tOpenCL Classified Mesh");

        cv::Mat scratch = img.clone();

        for (int i = 0; i < pixel_coordinates.size(); ++i) {
          cv::Point p1(pixel_coordinates[i][0], pixel_coordinates[i][1]);

          cv::Scalar colour(uint8_t(classifications[i * 2 + 0] * 255), 0, uint8_t(classifications[i * 2 + 1] * 255));
          // cv::Scalar colour( classifications[i * 2 + 0] > 0.5 ? 255 : 0, 0, classifications[i * 2 + 1] >= 0.5 ? 255 :
          // 0, 255);

          for (const auto& n : neighbourhood[i]) {
            if (n < pixel_coordinates.size()) {
              cv::Point p2(pixel_coordinates[n][0], pixel_coordinates[n][1]);
              cv::Point p2x = p1 + ((p2 - p1) * 0.5);
              cv::line(scratch, p1, p2x, colour, 1);
            }
          }
        }

        cv::imshow("Image", scratch);
        // Wait for esc key
        if (char(cv::waitKey(0)) == 27) break;
      }


      // Run our classifier
      {
        t.reset();
        auto classified = cpu_classifier(cpu_mesh.height(Hoc[2][3]), img.data, visualmesh::fourcc("BGRA"), Hoc, lens);

        auto& neighbourhood                                 = classified.neighbourhood;
        std::vector<std::array<float, 2>> pixel_coordinates = classified.pixel_coordinates;
        auto classifications                                = classified.classifications;

        t.measure("\tCPU Classified Mesh");

        cv::Mat scratch = img.clone();

        for (int i = 0; i < pixel_coordinates.size(); ++i) {
          cv::Point p1(pixel_coordinates[i][0], pixel_coordinates[i][1]);

          cv::Scalar colour(uint8_t(classifications[i * 2 + 0] * 255), 0, uint8_t(classifications[i * 2 + 1] * 255));
          // cv::Scalar colour( classifications[i * 2 + 0] > 0.5 ? 255 : 0, 0, classifications[i * 2 + 1] >= 0.5 ? 255 :
          // 0, 255);

          for (const auto& n : neighbourhood[i]) {
            if (n < pixel_coordinates.size()) {
              cv::Point p2(pixel_coordinates[n][0], pixel_coordinates[n][1]);
              cv::Point p2x = p1 + ((p2 - p1) * 0.5);
              cv::line(scratch, p1, p2x, colour, 1);
            }
          }
        }

        cv::imshow("Image", scratch);
        // Wait for esc key
        if (char(cv::waitKey(0)) == 27) break;
      }
    }
  }
}
