#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts TUM formatted RGB and depth files to SRRG format (json/bag)", 0};
// clang-format on

void loadImageInformation(const std::string& infile_,
                          std::vector<std::string>& image_filenames_,
                          std::vector<double>& timestamps_seconds_);

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_depth_input_file(
    &command_line_parser, "d",  "depth", "depth file to convert", "depth.txt");
  ArgumentString argument_rgb_input_file(
    &command_line_parser, "r",  "rgb", "RGB file to convert", "rgb.txt");
  ArgumentString argument_output_file(
    &command_line_parser, "o",  "output", "converted output file", "messages.json");
  ArgumentString argument_ground_truth_file(
     &command_line_parser, "gt",  "ground-truth", "ground truth file", "groundtruth.txt");
  ArgumentInt argument_camera_identifier(
     &command_line_parser, "ci",  "camera-identifier", "used camera identifier (1, 2 or 3)", 1);

  // clang-format on
  command_line_parser.parse();
  if (argument_depth_input_file.value().empty()) {
    std::cerr << "ERROR: input depth file is empty (set with -d)" << std::endl;
    return 0;
  }
  if (argument_rgb_input_file.value().empty()) {
    std::cerr << "ERROR: input RGB file is empty (set with -r)" << std::endl;
    return 0;
  }
  if (argument_output_file.value().empty()) {
    std::cerr << "ERROR: output filename is empty (set with -o)" << std::endl;
    return 0;
  }
  if (argument_ground_truth_file.value().empty()) {
    std::cerr << "ERROR: ground truth filename is empty (set with -gt)" << std::endl;
    return 0;
  }

  // ds buffer ground truth data
  StdVectorEigenIsometry3f camera_poses_in_world;
  std::vector<double> timestamps_gt;
  {
    std::ifstream ground_truth_file(argument_ground_truth_file.value());
    if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
      throw std::runtime_error("ERROR: unable to open ground truth file");
    }

    // ds skip the first 3 lines (TUM format header information)
    std::cerr << "<header>" << std::endl;
    std::string buffer;
    std::getline(ground_truth_file, buffer);
    std::cerr << buffer << std::endl;
    std::getline(ground_truth_file, buffer);
    std::cerr << buffer << std::endl;
    std::getline(ground_truth_file, buffer);
    std::cerr << buffer << std::endl;
    std::cerr << "</header>" << std::endl;

    double timestamp_seconds;
    double t_x, t_y, t_z;
    double q_x, q_y, q_z, q_w;
    while (ground_truth_file >> timestamp_seconds >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >>
           q_w) {
      if (timestamp_seconds == 0) {
        throw std::runtime_error("ERROR: GT timestamps are expected to start at > 0");
      }
      Isometry3f camera_in_world;
      camera_in_world.translation() = Vector3f(t_x, t_y, t_z);
      camera_in_world.linear()      = Quaternionf(q_w, q_x, q_y, q_z).toRotationMatrix();
      camera_poses_in_world.push_back(camera_in_world);
      timestamps_gt.push_back(timestamp_seconds);
    }
    ground_truth_file.close();
  }
  if (timestamps_gt.empty()) {
    throw std::runtime_error("ERROR: no ground truth poses loaded");
  }
  std::cerr << "loaded ground truth poses: " << timestamps_gt.size() << std::endl;

  // ds configure serializer
  Serializer serializer;
  serializer.setFilePath(argument_output_file.value());

  // ds read depth and RGB images and timestamps
  std::vector<std::string> image_filenames_rgb;
  std::vector<double> timestamps_rgb;
  std::vector<std::string> image_filenames_depth;
  std::vector<double> timestamps_depth;

  // ds parse image information
  loadImageInformation(argument_rgb_input_file.value(), image_filenames_rgb, timestamps_rgb);
  if (timestamps_rgb.empty()) {
    throw std::runtime_error("ERROR: no RGB image information retrieved");
  }
  loadImageInformation(argument_depth_input_file.value(), image_filenames_depth, timestamps_depth);
  if (timestamps_depth.empty()) {
    throw std::runtime_error("ERROR: no Depth image information retrieved");
  }

  // ds TUM calibration TODO load from disk instead of hardcoded switch
  Matrix3f camera_calibration_matrix;
  Vector_<double, 5> distortion_coefficients;

  const size_t camera_identifier(argument_camera_identifier.value());
  std::cerr << "setting calibration for camera index: " << camera_identifier << std::endl;
  switch (camera_identifier) {
    case 1: {
      camera_calibration_matrix << 517.3, 0, 318.6, 0, 516.5, 255.3, 0, 0, 1;
      distortion_coefficients << 0.2624, -0.9531, -0.0054, 0.0026, 1.1633;
      break;
    }
    case 2: {
      camera_calibration_matrix << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
      distortion_coefficients << 0.2312, -0.7849, -0.0033, -0.0001, 0.9172;
      break;
    }
    case 3: {
      camera_calibration_matrix << 535.4, 0, 320.1, 0, 539.2, 247.6, 0, 0, 1;
      distortion_coefficients.setZero();
      break;
    }
    default: {
      throw std::runtime_error("ERROR: unsupported camera number: " +
                               std::to_string(camera_identifier));
    }
  }
  std::cerr << "camera matrix: " << std::endl;
  std::cerr << camera_calibration_matrix << std::endl;
  std::cerr << "distortion coefficients: " << std::endl;
  std::cerr << distortion_coefficients.transpose() << std::endl;

  // ds common parameters for all TUM sequences TODO verify
  const std::string distortion_model_name = "radial_tangential";
  const float factor_to_srrg_depth        = 1.0 / 5;
  std::cerr << "depth factor: " << factor_to_srrg_depth << std::endl;
  std::cerr << "press [ENTER] to start conversion" << std::endl;
  getchar();

  // ds playback configuration UAGHS
  constexpr double timestamp_tolerance_seconds = 0.01;
  double timestamp_oldest                      = 0;
  size_t index_gt                              = 0;
  size_t index_rgb                             = 0;
  size_t index_depth                           = 0;

  // ds determine initial, smallest timestamp
  timestamp_oldest = timestamps_gt[0];
  if (timestamp_oldest > timestamps_rgb[0]) {
    timestamp_oldest = timestamps_rgb[0];
  }
  if (timestamp_oldest > timestamps_depth[0]) {
    timestamp_oldest = timestamps_depth[0];
  }
  std::cerr << std::setprecision(16);
  std::cerr << "initial timestamp (s): " << timestamp_oldest << std::endl;

  // ds playback all buffers
  while (index_gt < timestamps_gt.size() || index_rgb < timestamps_rgb.size() ||
         index_depth < timestamps_depth.size()) {
    // ds if we still have ground truth information available
    if (index_gt < timestamps_gt.size()) {
      const double& timestamp_candidate = timestamps_gt[index_gt];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        // ds write message
        TransformEventsMessagePtr transform_message(
          new TransformEventsMessage("/tf", "/camera_rgb", index_gt, timestamp_candidate));
        transform_message->events.resize(1);
        TransformEvent event_gt(
          timestamp_candidate, "camera_rgb", camera_poses_in_world[index_gt], "world");
        transform_message->events.setValue(0, event_gt);
        serializer.writeObject(*transform_message);
        std::cerr << "G";
        ++index_gt;
      }
    }

    // ds if we have RGB data available
    if (index_rgb < timestamps_rgb.size()) {
      const double& timestamp_candidate = timestamps_rgb[index_rgb];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        // ds create image message
        ImageMessagePtr image_message(
          new ImageMessage("/camera/rgb/image", "/camera_rgb", index_rgb, timestamp_candidate));

        // ds load RGB image from disk and convert
        const cv::Mat image_opencv =
          cv::imread(image_filenames_rgb[index_rgb], CV_LOAD_IMAGE_GRAYSCALE);
        if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
          std::cerr << "\nWARNING: skipping invalid RGB image: " << image_filenames_rgb[index_rgb]
                    << " with dimensions: " << image_opencv.rows << " x " << image_opencv.cols
                    << std::endl;
          continue;
        }
        ImageUInt8* base_image(new ImageUInt8());
        base_image->fromCv(image_opencv);
        image_message->setImage(base_image);
        image_message->image_cols.setValue(image_opencv.cols);
        image_message->image_rows.setValue(image_opencv.rows);

        // ds create camera info message
        CameraInfoMessagePtr camera_info_message(
          new CameraInfoMessage(image_message->topic.value() + "/info",
                                image_message->frame_id.value(),
                                image_message->seq.value(),
                                image_message->timestamp.value()));
        camera_info_message->depth_scale.setValue(1e-3f);
        camera_info_message->projection_model.setValue("pinhole");
        camera_info_message->distortion_model.setValue(distortion_model_name);
        camera_info_message->distortion_coefficients.setValue(distortion_coefficients);
        camera_info_message->camera_matrix.setValue(camera_calibration_matrix);

        // ds write messages
        serializer.writeObject(*image_message);
        serializer.writeObject(*camera_info_message);
        std::cerr << "I";

        // ds display converted images
        cv::imshow("processed image RGB", image_opencv);
        cv::waitKey(1);
        ++index_rgb;
      }
    }

    // ds if we have depth data available
    if (index_depth < timestamps_depth.size()) {
      const double& timestamp_candidate = timestamps_depth[index_depth];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        // ds create image message
        ImageMessagePtr image_message(new ImageMessage(
          "/camera/depth/image", "/camera_depth", index_depth, timestamp_candidate));

        // ds load Depth image from disk and convert
        const cv::Mat image_opencv =
          cv::imread(image_filenames_depth[index_depth], CV_LOAD_IMAGE_ANYDEPTH);
        if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
          std::cerr << "\nWARNING: skipping invalid Depth image: "
                    << image_filenames_depth[index_depth]
                    << " with dimensions: " << image_opencv.rows << " x " << image_opencv.cols
                    << std::endl;
          continue;
        }
        cv::Mat image_opencv_depth_scaled(image_opencv);
        image_opencv_depth_scaled *= factor_to_srrg_depth;
        ImageUInt16* image_depth(new ImageUInt16());
        image_depth->fromCv(image_opencv_depth_scaled);
        image_message->setImage(image_depth); // ImageFloat takes up double the space
        image_message->image_cols.setValue(image_opencv.cols);
        image_message->image_rows.setValue(image_opencv.rows);

        // ds create camera info message
        CameraInfoMessagePtr camera_info_message(
          new CameraInfoMessage(image_message->topic.value() + "/info",
                                image_message->frame_id.value(),
                                image_message->seq.value(),
                                image_message->timestamp.value()));
        camera_info_message->depth_scale.setValue(1e-3f);
        camera_info_message->projection_model.setValue("pinhole");
        camera_info_message->distortion_model.setValue(distortion_model_name);
        camera_info_message->distortion_coefficients.setValue(distortion_coefficients);
        camera_info_message->camera_matrix.setValue(camera_calibration_matrix);

        // ds write messages
        serializer.writeObject(*image_message);
        serializer.writeObject(*camera_info_message);
        std::cerr << "D";

        // ds display converted images
        image_opencv *= 5; // ds visibility
        cv::imshow("processed image Depth", image_opencv);
        cv::waitKey(1);
        ++index_depth;
      }
    }

    // ds update timestamp
    bool timestamp_updated = false;
    if (index_gt < timestamps_gt.size()) {
      // ds always move timestamp
      timestamp_oldest  = timestamps_gt[index_gt];
      timestamp_updated = true;
    }
    if (index_rgb < timestamps_rgb.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_rgb[index_rgb] || !timestamp_updated) {
        timestamp_oldest  = timestamps_rgb[index_rgb];
        timestamp_updated = true;
      }
    }
    if (index_depth < timestamps_depth.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_depth[index_depth] || !timestamp_updated) {
        timestamp_oldest  = timestamps_depth[index_depth];
        timestamp_updated = true;
      }
    }
  }
  std::cerr << std::endl;
  std::cerr << "# converted RGB messages: " << index_rgb << "/" << timestamps_rgb.size()
            << std::endl;
  std::cerr << "# converted Depth messages: " << index_depth << "/" << timestamps_depth.size()
            << std::endl;
  std::cerr << "# converted GT messages: " << index_gt << "/" << timestamps_gt.size() << std::endl;
  return 0;
}

void loadImageInformation(const std::string& infile_name_,
                          std::vector<std::string>& image_filenames_,
                          std::vector<double>& timestamps_seconds_) {
  std::ifstream infile(infile_name_);
  if (!infile.good() || !infile.is_open()) {
    throw std::runtime_error("loadImageInformation|ERROR: unable to open file: " + infile_name_);
  }
  image_filenames_.clear();
  timestamps_seconds_.clear();

  // ds skip the first 3 lines (TUM format header information)
  std::cerr << "loadImageInformation|<header>" << std::endl;
  std::string buffer;
  std::getline(infile, buffer);
  std::cerr << buffer << std::endl;
  std::getline(infile, buffer);
  std::cerr << buffer << std::endl;
  std::getline(infile, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "loadImageInformation|</header>" << std::endl;

  // ds parse image information by tokens
  double timestamp_seconds;
  std::string filename;
  while (infile >> timestamp_seconds >> filename) {
    image_filenames_.push_back(filename);
    timestamps_seconds_.push_back(timestamp_seconds);
  }
  std::cerr << "loadImageInformation|# loaded entries: " << timestamps_seconds_.size()
            << " for file '" << infile_name_ << "'" << std::endl;
  infile.close();
}
