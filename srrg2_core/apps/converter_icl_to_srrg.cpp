#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts TUM formatted RGB and depth files to SRRG format (json/bag)", 0};
// clang-format on

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_input_file (
    &command_line_parser, "a",  "associations", "associations file to convert (TUM)", "associations.txt");
  ArgumentString argument_output_file (
    &command_line_parser, "o",  "output", "converted output file", "messages.json");
  ArgumentString argument_ground_truth_file (
     &command_line_parser, "gt",  "ground-truth", "ground truth file (TUM)", "gt.txt");

  // clang-format on
  command_line_parser.parse();
  if (argument_input_file.value().empty()) {
    std::cerr << "ERROR: input file is empty (set with -a)" << std::endl;
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
  std::map<size_t,
           Isometry3f,
           std::less<size_t>,
           Eigen::aligned_allocator<std::pair<size_t, Isometry3f>>>
    camera_poses_in_world;
  {
    std::ifstream ground_truth_file(argument_ground_truth_file.value());
    if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
      throw std::runtime_error("ERROR: unable to open ground truth file");
    }
    size_t timestamp = 0;
    double t_x, t_y, t_z;
    double q_x, q_y, q_z, q_w;
    while (ground_truth_file >> timestamp >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w) {
      if (timestamp == 0) {
        throw std::runtime_error("ERROR: GT timestamps are expected to start at 1 for ICL");
      }
      Isometry3f camera_in_world;
      camera_in_world.translation() = Vector3f(t_x, t_y, t_z);
      camera_in_world.linear()      = Quaternionf(q_w, q_x, q_y, q_z).toRotationMatrix();
      camera_poses_in_world.insert(std::make_pair(timestamp - 1, camera_in_world));
    }
    ground_truth_file.close();
  }
  std::cerr << "loaded ground truth poses: " << camera_poses_in_world.size() << std::endl;

  // ds configure serializer
  Serializer serializer;
  serializer.setFilePath(argument_output_file.value());

  // ds open file
  std::ifstream input_file(argument_input_file.value());
  if (!input_file.good() || !input_file.is_open()) {
    throw std::runtime_error("ERROR: unable to open input file");
  }

  // ds TUM camera matrix and depth factor TODO load from disk
  Matrix3f camera_calibration_matrix;
  camera_calibration_matrix << 481.2, 0, 319.5, 0, -481, 239.5, 0, 0, 1;
  std::cerr << "current camera matrix: [TODO automate this]: " << std::endl;
  std::cerr << camera_calibration_matrix << std::endl;
  const float factor_to_srrg_depth = 1.0 / 5;
  std::cerr << "depth factor: [TODO automate this]: " << factor_to_srrg_depth << std::endl;
  std::cerr << "press [ENTER] to start conversion" << std::endl;
  getchar();

  // ds parse associations by tokens
  size_t number_of_converted_messages = 0;
  size_t timestamp_rgb                = 0;
  std::string path_image_rgb          = "";
  size_t timestamp_depth              = 0;
  std::string path_image_depth        = "";
  while (input_file >> timestamp_depth >> path_image_depth >> timestamp_rgb >> path_image_rgb) {
    // ds image messages
    ImageMessagePtr image_message_depth(new ImageMessage(
      "/camera/depth/image", "/openni_rgb_optical_frame", timestamp_depth, timestamp_depth));
    ImageMessagePtr image_message_rgb(new ImageMessage(
      "/camera/rgb/image_color", "/openni_rgb_optical_frame", timestamp_rgb, timestamp_rgb));

    // ds load Depth image from disk and convert
    cv::Mat image_opencv_depth = cv::imread(path_image_depth, CV_LOAD_IMAGE_ANYDEPTH);
    if (image_opencv_depth.rows <= 0 || image_opencv_depth.cols <= 0) {
      std::cerr << "\nWARNING: skipping invalid image: " << path_image_depth
                << " with dimensions: " << image_opencv_depth.rows << " x "
                << image_opencv_depth.cols << std::endl;
      continue;
    }
    cv::Mat image_opencv_depth_scaled(image_opencv_depth);
    image_opencv_depth_scaled *= factor_to_srrg_depth;
    ImageUInt16* image_depth(new ImageUInt16());
    image_depth->fromCv(image_opencv_depth_scaled);
    image_message_depth->setImage(image_depth); // ImageFloat takes up double the space
    image_message_depth->image_cols.setValue(image_opencv_depth.cols);
    image_message_depth->image_rows.setValue(image_opencv_depth.rows);

    // ds load RGB image from disk and convert
    const cv::Mat image_opencv_rgb = cv::imread(path_image_rgb, CV_LOAD_IMAGE_GRAYSCALE);
    if (image_opencv_rgb.rows <= 0 || image_opencv_rgb.cols <= 0) {
      std::cerr << "\nWARNING: skipping invalid image: " << path_image_rgb
                << " with dimensions: " << image_opencv_rgb.rows << " x " << image_opencv_rgb.cols
                << std::endl;
      continue;
    }
    ImageUInt8* base_image(new ImageUInt8());
    base_image->fromCv(image_opencv_rgb);
    image_message_rgb->setImage(base_image);
    image_message_rgb->image_cols.setValue(image_opencv_rgb.cols);
    image_message_rgb->image_rows.setValue(image_opencv_rgb.rows);

    // ds display converted images
    image_opencv_depth *= 5; // ds visibility
    cv::imshow("processed image Depth", image_opencv_depth);
    cv::imshow("processed image RGB", image_opencv_rgb);
    cv::waitKey(1);

    // ds camera calibration messages
    CameraInfoMessagePtr camera_info_message_depth(
      new CameraInfoMessage(image_message_depth->topic.value() + "/info",
                            image_message_depth->frame_id.value(),
                            image_message_depth->seq.value(),
                            image_message_depth->timestamp.value()));
    camera_info_message_depth->depth_scale.setValue(1e-3f);
    camera_info_message_depth->projection_model.setValue("pinhole");
    camera_info_message_depth->distortion_model.setValue("undistorted");
    camera_info_message_depth->camera_matrix.setValue(camera_calibration_matrix);
    CameraInfoMessagePtr camera_info_message_rgb(
      new CameraInfoMessage(image_message_rgb->topic.value() + "/info",
                            image_message_rgb->frame_id.value(),
                            image_message_rgb->seq.value(),
                            image_message_rgb->timestamp.value()));
    camera_info_message_rgb->depth_scale.setValue(1e-3f);
    camera_info_message_rgb->projection_model.setValue("pinhole");
    camera_info_message_rgb->distortion_model.setValue("undistorted");
    camera_info_message_rgb->camera_matrix.setValue(camera_calibration_matrix);

    // ds write messages
    serializer.writeObject(*image_message_depth);
    serializer.writeObject(*image_message_rgb);
    serializer.writeObject(*camera_info_message_depth);
    serializer.writeObject(*camera_info_message_rgb);
    std::cerr << "ID";

    // ds look for ground truth data (in RGB camera frame)
    auto iterator = camera_poses_in_world.find(timestamp_rgb);
    if (iterator != camera_poses_in_world.end()) {
      TransformEventsMessagePtr transform_message(
        new TransformEventsMessage("/tf",
                                   image_message_rgb->frame_id.value(),
                                   image_message_rgb->seq.value(),
                                   image_message_rgb->timestamp.value()));
      transform_message->events.resize(1);
      TransformEvent event_gt(
        image_message_rgb->timestamp.value(), "camera_rgb", iterator->second, "world");
      transform_message->events.setValue(0, event_gt);
      serializer.writeObject(*transform_message);
      std::cerr << "G";
    } else {
      std::cerr << "_";
    }
    ++number_of_converted_messages;
  }
  std::cerr << std::endl;
  input_file.close();

  std::cerr << "# converted RGBD messages: " << number_of_converted_messages << std::endl;
  return 0;
}
