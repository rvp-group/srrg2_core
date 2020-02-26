#include <dirent.h>
#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts Malaga formatted stereo RGB files to SRRG format (json/bag)", 0};
// clang-format on

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& gps_poses_in_world_,
                                   std::vector<double>& timestamps_seconds_gt_);

void loadImageInformation(const std::string& folder_images_,
                          std::vector<std::string>& image_filenames_left_,
                          std::vector<double>& timestamps_seconds_left_,
                          std::vector<std::string>& image_filenames_right_,
                          std::vector<double>& timestamps_seconds_right_);

void loadIMUInformation(const std::string& file_imu_data_,
                        StdVectorEigenVector3d& angular_velocities_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        StdVectorEigenVector3d& orientations_,
                        std::vector<double>& timestamps_seconds_);

void loadCameraCalibration(const std::string& file_path_camera_calibration_,
                           Matrix3f& camera_calibration_matrix_left_,
                           Vector5d& distortion_coefficients_left_,
                           Isometry3f& imu_from_camera_left_,
                           Matrix3f& camera_calibration_matrix_right_,
                           Vector5d& distortion_coefficients_right_,
                           Isometry3f& imu_from_camera_right_);

void serializeCameraImageAndInfo(Serializer& serializer_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_,
                                 const Isometry3f& imu_from_camera_,
                                 const Isometry3f& transformation_right_from_left_);

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_folder_stereo_images(
    &command_line_parser, "if",  "images-folder",
    "camera image folder (contains: {img_CAMERA1_t_left.jpg, img_CAMERA1_t_right.jpg})", "");
  ArgumentString argument_file_stereo_camera_calibration(
    &command_line_parser, "cc",  "camera-calibration",
    "stereo camera calibration file", "");
  ArgumentString argument_file_imu(
    &command_line_parser, "imu",  "imu",
    "IMU messages file", "");
  ArgumentString argument_output_file(
    &command_line_parser, "o",  "output",
    "converted output file", "messages.json");
  ArgumentString argument_ground_truth_file(
     &command_line_parser, "gt",  "ground-truth",
     "ground truth file that contains IMU filtered prism poses w.r.t. world origin", "");

  // clang-format on
  command_line_parser.parse();
  if (argument_folder_stereo_images.value().empty()) {
    std::cerr << "ERROR: stereo images folder is not set (-if)" << std::endl;
    return 0;
  }
  if (argument_file_stereo_camera_calibration.value().empty()) {
    std::cerr << "ERROR: stereo camera calibration file is not set (-cc)" << std::endl;
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
  StdVectorEigenIsometry3f gps_poses_in_world;
  std::vector<double> timestamps_seconds_gt;
  loadGroundTruthPosesAndStamps(
    argument_ground_truth_file.value(), gps_poses_in_world, timestamps_seconds_gt);
  if (timestamps_seconds_gt.empty()) {
    throw std::runtime_error("ERROR: no ground truth poses loaded");
  }

  // ds save ground truth in kitti format (for benchmarking)
  std::ofstream ground_truth_kitti_format("gt.txt", std::ofstream::out);
  std::ofstream ground_truth_timestamps_kitti_format("times.txt", std::ofstream::out);
  ground_truth_kitti_format << std::fixed;
  ground_truth_kitti_format << std::setprecision(9);
  ground_truth_timestamps_kitti_format << std::fixed;
  ground_truth_timestamps_kitti_format << std::setprecision(9);
  Isometry3f gps_from_camera(Isometry3f::Identity());
  gps_from_camera.linear() << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  for (size_t i = 0; i < timestamps_seconds_gt.size(); ++i) {
    // ds move to kitti origin uagh
    const Isometry3f world_from_camera(gps_from_camera * gps_poses_in_world[i]);
    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 4; ++c) {
        ground_truth_kitti_format << world_from_camera.matrix()(r, c) << " ";
      }
    }
    ground_truth_kitti_format << std::endl;
    ground_truth_timestamps_kitti_format << timestamps_seconds_gt[i] << std::endl;
  }
  ground_truth_kitti_format.close();
  ground_truth_timestamps_kitti_format.close();
  std::cerr << "saved GT information to 'gt.txt' and 'times.txt' (KITTI style)" << std::endl;

  // ds configure serializer for parallel in and out streaming
  Serializer serializer;
  serializer.setFilePath(argument_output_file.value());

  // ds read depth and RGB images and timestamps
  std::vector<std::string> image_file_paths_camera_left;
  std::vector<double> timestamps_camera_left;
  std::vector<std::string> image_file_paths_camera_right;
  std::vector<double> timestamps_camera_right;

  // ds parse image information
  loadImageInformation(argument_folder_stereo_images.value(),
                       image_file_paths_camera_left,
                       timestamps_camera_left,
                       image_file_paths_camera_right,
                       timestamps_camera_right);
  if (timestamps_camera_left.empty()) {
    throw std::runtime_error("ERROR: no left camera image information retrieved");
  }

  // ds load camera calibration data from files
  Matrix3f camera_calibration_matrix_left(Matrix3f::Zero());
  Vector5d distortion_coefficients_left(Vector5d::Zero());
  Isometry3f imu_from_camera_left(Isometry3f::Identity());
  Matrix3f camera_calibration_matrix_right(Matrix3f::Zero());
  Vector5d distortion_coefficients_right(Vector5d::Zero());
  Isometry3f imu_from_camera_right(Isometry3f::Identity());
  loadCameraCalibration(argument_file_stereo_camera_calibration.value(),
                        camera_calibration_matrix_left,
                        distortion_coefficients_left,
                        imu_from_camera_left,
                        camera_calibration_matrix_right,
                        distortion_coefficients_right,
                        imu_from_camera_right);

  Isometry3f camera_left_to_right(imu_from_camera_right.inverse() * imu_from_camera_left);
  std::cerr << "camera LEFT w.r.t. RIGHT: " << std::endl;
  std::cerr << camera_left_to_right.matrix() << std::endl;

  // ds common parameters for all sequences
  const std::string distortion_model_name = "undistorted";

  // ds TODO load IMU information
  std::vector<double> timestamps_imu;
  StdVectorEigenVector3d angular_velocities;
  StdVectorEigenVector3d linear_accelerations;
  StdVectorEigenVector3d orientations_rpy;
  const double rate_hz = 100;
  loadIMUInformation(argument_file_imu.value(),
                     angular_velocities,
                     linear_accelerations,
                     orientations_rpy,
                     timestamps_imu);

  // ds stop before starting to allow for a quick configuration inspection
  std::cerr << "press [ENTER] to start conversion" << std::endl;
  getchar();

  // ds playback configuration UAGHS
  constexpr double timestamp_tolerance_seconds = 0.01;
  double timestamp_oldest                      = 0;
  size_t index_gt                              = 0;
  size_t index_camera_left                     = 0;
  size_t index_camera_right                    = 0;
  size_t index_imu                             = 0;

  // ds determine initial, smallest timestamp
  timestamp_oldest = timestamps_seconds_gt[0];
  if (timestamp_oldest > timestamps_camera_left[0]) {
    timestamp_oldest = timestamps_camera_left[0];
  }
  if (timestamp_oldest > timestamps_camera_right[0]) {
    timestamp_oldest = timestamps_camera_right[0];
  }
  if (!timestamps_imu.empty() && timestamp_oldest > timestamps_imu[0]) {
    timestamp_oldest = timestamps_imu[0];
  }
  std::cerr << std::setprecision(16);
  std::cerr << "initial timestamp (s): " << timestamp_oldest << std::endl;

  // ds playback all buffers
  while (index_gt < timestamps_seconds_gt.size() ||
         index_camera_left < timestamps_camera_left.size() ||
         index_camera_right < timestamps_camera_right.size() || index_imu < timestamps_imu.size()) {
    // ds if we still have ground truth information available
    if (index_gt < timestamps_seconds_gt.size()) {
      const double& timestamp_candidate = timestamps_seconds_gt[index_gt];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        // ds write message
        TransformEventsMessagePtr transform_message(
          new TransformEventsMessage("/tf", "/gps", index_gt, timestamp_candidate));
        transform_message->events.resize(1);
        TransformEvent event_gt(timestamp_candidate, "gps", gps_poses_in_world[index_gt], "world");
        transform_message->events.setValue(0, event_gt);
        serializer.writeObject(*transform_message);
        std::cerr << "G";
        ++index_gt;
      }
    }

    // ds if we have left image data available
    if (index_camera_left < timestamps_camera_left.size()) {
      const double& timestamp_candidate = timestamps_camera_left[index_camera_left];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        serializeCameraImageAndInfo(serializer,
                                    index_camera_left,
                                    timestamp_candidate,
                                    "camera_left",
                                    image_file_paths_camera_left[index_camera_left],
                                    distortion_model_name,
                                    distortion_coefficients_left,
                                    camera_calibration_matrix_left,
                                    imu_from_camera_left,
                                    camera_left_to_right);
      }
    }

    // ds if we have right image data available
    if (index_camera_right < timestamps_camera_right.size()) {
      const double& timestamp_candidate = timestamps_camera_right[index_camera_right];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        serializeCameraImageAndInfo(serializer,
                                    index_camera_right,
                                    timestamp_candidate,
                                    "camera_right",
                                    image_file_paths_camera_right[index_camera_right],
                                    distortion_model_name,
                                    distortion_coefficients_right,
                                    camera_calibration_matrix_right,
                                    imu_from_camera_right,
                                    camera_left_to_right);
      }
    }

    // ds if we have IMU data available
    if (index_imu < timestamps_imu.size()) {
      const double& timestamp_candidate = timestamps_imu[index_imu];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        IMUMessagePtr message(new IMUMessage("/imu", "/imu", index_imu, timestamp_candidate));
        message->angular_velocity.setValue(angular_velocities[index_imu].cast<float>());
        //        message->angular_velocity_covariance.setValue(Matrix3f::Identity());
        message->linear_acceleration.setValue(linear_accelerations[index_imu].cast<float>());
        //        message->linear_acceleration_covariance.setValue(Matrix3f::Identity());
        message->orientation.setValue(
          geometry3d::a2r<float>(orientations_rpy[index_imu].cast<float>()));
        //        message->orientation_covariance.setValue(Matrix3f::Identity());
        message->rate_hz.setValue(rate_hz);
        serializer.writeObject(*message);
        std::cerr << "I";
        ++index_imu;
      }
    }

    // ds update timestamp in a cascade - lowest wins
    bool timestamp_updated = false;
    if (index_gt < timestamps_seconds_gt.size()) {
      // ds always move timestamp
      timestamp_oldest  = timestamps_seconds_gt[index_gt];
      timestamp_updated = true;
    }
    if (index_camera_left < timestamps_camera_left.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_camera_left[index_camera_left] || !timestamp_updated) {
        timestamp_oldest  = timestamps_camera_left[index_camera_left];
        timestamp_updated = true;
      }
    }
    if (index_camera_right < timestamps_camera_right.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_camera_right[index_camera_right] || !timestamp_updated) {
        timestamp_oldest  = timestamps_camera_right[index_camera_right];
        timestamp_updated = true;
      }
    }
    if (index_imu < timestamps_imu.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_imu[index_imu] || !timestamp_updated) {
        timestamp_oldest  = timestamps_imu[index_imu];
        timestamp_updated = true;
      }
    }
  }
  std::cerr << std::endl;
  std::cerr << "# converted LEFT camera messages: " << index_camera_left << "/"
            << timestamps_camera_left.size() << std::endl;
  std::cerr << "# converted RIGHT camera messages: " << index_camera_right << "/"
            << timestamps_camera_right.size() << std::endl;
  std::cerr << "# converted IMU messages: " << index_imu << "/" << timestamps_imu.size()
            << std::endl;
  std::cerr << "# converted GT messages: " << index_gt << "/" << timestamps_seconds_gt.size()
            << std::endl;
  return 0;
}

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& gps_poses_in_world_,
                                   std::vector<double>& timestamps_seconds_gt_) {
  gps_poses_in_world_.clear();
  timestamps_seconds_gt_.clear();

  // ds load ground truth text file
  std::ifstream stream_ground_truth(ground_truth_file_, std::ifstream::in);

  // ds check failure
  if (!stream_ground_truth.is_open() || !stream_ground_truth.good()) {
    throw std::runtime_error("loadGroundTruthPosesAndStamps|ERROR: unable to open file: '" +
                             ground_truth_file_ + "'");
  }

  // ds reading buffers - skip the first line (header line)
  double timestamp_seconds      = 0;
  double latitude_radians       = 0;
  double longitude_radians      = 0;
  double altitude_meters        = 0;
  uint32_t fix                  = 0;
  uint32_t number_of_satellites = 0;
  double speed_knots            = 0;
  double heading_degrees        = 0;
  std::string line_buffer("");
  std::getline(stream_ground_truth, line_buffer);

  // ds start parsing all poses to the last line
  Isometry3d pose_gps_previous(Isometry3d::Identity());
  const Vector3d orientation_car(0, 0, 1);

  // ds reading buffers
  line_buffer = "";
  while (std::getline(stream_ground_truth, line_buffer) && !line_buffer.empty()) {
    std::istringstream stream(line_buffer);

    // ds parse in fixed order
    stream >> timestamp_seconds;
    stream >> latitude_radians;
    stream >> longitude_radians;
    stream >> altitude_meters;
    stream >> fix;
    stream >> number_of_satellites;
    stream >> speed_knots;
    stream >> heading_degrees;

    // ds parse pose matrix directly from file
    Isometry3d pose(Isometry3d::Identity());
    for (uint8_t u = 0; u < 3; ++u) {
      stream >> pose(u, 3);
    }

    // ds compute relative orientation (have nothing else)
    const Vector3d translation_delta = pose.translation() - pose_gps_previous.translation();
    Vector3d translation_delta_normalized(Vector3d::Zero());
    if (translation_delta.norm() > 0) {
      translation_delta_normalized = translation_delta / translation_delta.norm();
      Quaterniond orientation =
        Quaterniond::FromTwoVectors(orientation_car, translation_delta_normalized);
      pose.linear() = orientation.toRotationMatrix();

      // ds check if we can update the initial orientation
      if (gps_poses_in_world_.size() == 1) {
        gps_poses_in_world_.front().linear() = pose.linear().cast<float>();
      }
    }

    // ds set pose
    gps_poses_in_world_.push_back(pose.cast<float>());
    pose_gps_previous = pose;

    // ds sanity check
    if (!timestamps_seconds_gt_.empty() && timestamp_seconds < timestamps_seconds_gt_.back()) {
      throw std::runtime_error("loadGroundTruthPosesAndStamps|invalid timestamp (s): " +
                               std::to_string(timestamp_seconds));
    }

    // ds keep timestamp
    timestamps_seconds_gt_.push_back(timestamp_seconds);
  }
  stream_ground_truth.close();
  std::cerr << "loadGroundTruthPosesAndStamps|loaded GPS poses: " << gps_poses_in_world_.size()
            << std::endl;
}

void loadImageInformation(const std::string& folder_images_,
                          std::vector<std::string>& image_filenames_left_,
                          std::vector<double>& timestamps_seconds_left_,
                          std::vector<std::string>& image_filenames_right_,
                          std::vector<double>& timestamps_seconds_right_) {
  image_filenames_left_.clear();
  timestamps_seconds_left_.clear();
  image_filenames_right_.clear();
  timestamps_seconds_right_.clear();

  // ds obtain list of image filenames in the folder
  std::vector<std::string> image_file_names;
  image_file_names.reserve(10000);
  DIR* directory;
  struct dirent* entry;
  if ((directory = opendir(folder_images_.c_str()))) {
    while ((entry = readdir(directory))) {
      // ds ignore linux fake directories
      if (entry->d_name[0] != '.') {
        image_file_names.emplace_back(entry->d_name);
      }
    }
  }
  closedir(directory);

  // ds order input by timestamps
  std::sort(image_file_names.begin(), image_file_names.end());

  // ds parse string into tokens
  // ds parse image information by tokens
  double timestamp_previous_seconds = 0;
  for (const std::string& image_file_name : image_file_names) {
    // ds parse timestamp TODO less hardcoding hehe
    const double timestamp_seconds = std::stod(image_file_name.substr(12, 17));

    // ds sanity check
    if (timestamp_seconds < timestamp_previous_seconds) {
      throw std::runtime_error("loadImageInformation|ERROR: inconsistent timestamp (s): " +
                               std::to_string(timestamp_seconds) + " < " +
                               std::to_string(timestamp_previous_seconds));
    } else {
      timestamp_previous_seconds = timestamp_seconds;
    }

    // ds parse based on tag
    if (image_file_name.find("left") != std::string::npos) {
      image_filenames_left_.push_back(folder_images_ + "/" + image_file_name);
      timestamps_seconds_left_.push_back(timestamp_seconds);
    } else {
      image_filenames_right_.push_back(folder_images_ + "/" + image_file_name);
      timestamps_seconds_right_.push_back(timestamp_seconds);
    }
  }
  std::cerr << "loadImageInformation|# loaded entries LEFT: " << timestamps_seconds_left_.size()
            << " RIGHT: " << timestamps_seconds_right_.size() << " for folder '" << folder_images_
            << "'" << std::endl;
}

void loadCameraCalibration(const std::string& file_path_camera_calibration_,
                           Matrix3f& camera_calibration_matrix_left_,
                           Vector5d& distortion_coefficients_left_,
                           Isometry3f& imu_from_camera_left_,
                           Matrix3f& camera_calibration_matrix_right_,
                           Vector5d& distortion_coefficients_right_,
                           Isometry3f& imu_from_camera_right_) {
  std::cerr << "loadCameraCalibration|calibration file: '" << file_path_camera_calibration_ << "'"
            << std::endl;
  std::ifstream stream(file_path_camera_calibration_);
  if (!stream.good() || !stream.is_open()) {
    throw std::runtime_error("loadCameraCalibration|ERROR: unable to open file: " +
                             file_path_camera_calibration_);
  }

  // ds evil parsing - skip lines until we hit the transform 'data' line
  std::string buffer;
  for (size_t i = 0; i < 7; ++i) {
    std::getline(stream, buffer);
  }

  // ds read left camera parameters
  camera_calibration_matrix_left_(0, 2) = std::stod(buffer.substr(3)); // ds c_x
  std::getline(stream, buffer);
  camera_calibration_matrix_left_(1, 2) = std::stod(buffer.substr(3)); // ds c_y
  std::getline(stream, buffer);
  camera_calibration_matrix_left_(0, 0) = std::stod(buffer.substr(3)); // ds f_x
  std::getline(stream, buffer);
  camera_calibration_matrix_left_(1, 1) = std::stod(buffer.substr(3)); // ds f_y
  camera_calibration_matrix_left_(2, 2) = 1;
  distortion_coefficients_left_.setZero();

  // ds skip lines
  for (size_t i = 0; i < 8; ++i) {
    std::getline(stream, buffer);
  }

  // ds read right camera parameters
  camera_calibration_matrix_right_(0, 2) = std::stod(buffer.substr(3)); // ds c_x
  std::getline(stream, buffer);
  camera_calibration_matrix_right_(1, 2) = std::stod(buffer.substr(3)); // ds c_y
  std::getline(stream, buffer);
  camera_calibration_matrix_right_(0, 0) = std::stod(buffer.substr(3)); // ds f_x
  std::getline(stream, buffer);
  camera_calibration_matrix_right_(1, 1) = std::stod(buffer.substr(3)); // ds f_y
  camera_calibration_matrix_right_(2, 2) = 1;
  distortion_coefficients_right_.setZero();

  // ds skip lines
  for (size_t i = 0; i < 7; ++i) {
    std::getline(stream, buffer);
  }

  const size_t index_begin = buffer.find_last_of('[') + 1;
  const size_t index_end   = buffer.find_last_of(']');

  // ds parse transform by tokens
  Isometry3f camera_right_from_left(Isometry3d::Identity());
  std::stringstream stream_trans(buffer.substr(index_begin, index_end - index_begin));
  Vector_<float, 7> translation_orientation(Vector_<float, 7>::Zero());
  for (size_t i = 0; i < 7; ++i) {
    stream_trans >> translation_orientation(i);
  }
  camera_right_from_left.translation() = translation_orientation.head<3>();

  // ds TODO implement actual transforms
  imu_from_camera_left_.setIdentity();
  imu_from_camera_right_.setIdentity();
  imu_from_camera_left_.translation()  = -camera_right_from_left.translation() / 2.0;
  imu_from_camera_right_.translation() = camera_right_from_left.translation() / 2.0;

  stream.close();
  std::cerr << "loadCameraCalibration|camera matrix LEFT: " << std::endl;
  std::cerr << camera_calibration_matrix_left_ << std::endl;
  std::cerr << "loadCameraCalibration|camera matrix RIGHT: " << std::endl;
  std::cerr << camera_calibration_matrix_right_ << std::endl;
  std::cerr << "loadCameraCalibration|distortion coefficients LEFT: " << std::endl;
  std::cerr << distortion_coefficients_left_.transpose() << std::endl;
  std::cerr << "loadCameraCalibration|distortion coefficients RIGHT: " << std::endl;
  std::cerr << distortion_coefficients_right_.transpose() << std::endl;
  std::cerr << "loadCameraCalibration|LEFT camera w.r.t. IMU: " << std::endl;
  std::cerr << imu_from_camera_left_.matrix() << std::endl;
  std::cerr << "loadCameraCalibration|RIGHT camera w.r.t. IMU: " << std::endl;
  std::cerr << imu_from_camera_right_.matrix() << std::endl;
}

void loadIMUInformation(const std::string& file_imu_data_,
                        StdVectorEigenVector3d& angular_velocities_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        StdVectorEigenVector3d& orientations_,
                        std::vector<double>& timestamps_seconds_) {
  angular_velocities_.clear();
  linear_accelerations_.clear();
  orientations_.clear();
  std::cerr << "loadIMUInformation|file: '" << file_imu_data_ << "'" << std::endl;
  std::ifstream stream(file_imu_data_);
  if (!stream.good() || !stream.is_open()) {
    throw std::runtime_error("loadIMUInformation|ERROR: unable to open file: " + file_imu_data_);
  }

  // ds skip first line (header)
  std::string buffer;
  std::getline(stream, buffer);
  std::cerr << "loadIMUInformation|<header>" << std::endl;
  std::cerr << buffer << std::endl;
  std::cerr << "loadIMUInformation|<header>" << std::endl;

  // ds parse by tokens
  angular_velocities_.reserve(100000);
  linear_accelerations_.reserve(100000);
  orientations_.reserve(100000);
  double timestamp_seconds = 0;
  double IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC, IMU_YAW_VEL, IMU_PITCH_VEL, IMU_ROLL_VEL, IMU_X_VEL,
    IMU_Y_VEL, IMU_Z_VEL, IMU_YAW, IMU_PITCH, IMU_ROLL, IMU_X, IMU_Y, IMU_Z;
  while (stream >> timestamp_seconds >> IMU_X_ACC >> IMU_Y_ACC >> IMU_Z_ACC >> IMU_YAW_VEL >>
         IMU_PITCH_VEL >> IMU_ROLL_VEL >> IMU_X_VEL >> IMU_Y_VEL >> IMU_Z_VEL >> IMU_YAW >>
         IMU_PITCH >> IMU_ROLL >> IMU_X >> IMU_Y >> IMU_Z) {
    angular_velocities_.emplace_back(Vector3d(IMU_YAW_VEL, IMU_PITCH_VEL, IMU_ROLL_VEL));
    linear_accelerations_.emplace_back(Vector3d(IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC));
    orientations_.emplace_back(Vector3d(IMU_YAW, IMU_PITCH, IMU_ROLL));
    timestamps_seconds_.emplace_back(timestamp_seconds);
  }
  std::cerr << "loadIMUInformation|# loaded entries: " << timestamps_seconds_.size()
            << " for file '" << file_imu_data_ << "'" << std::endl;
  stream.close();
}

void serializeCameraImageAndInfo(Serializer& serializer_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_,
                                 const Isometry3f& imu_from_camera_,
                                 const Isometry3f& transformation_right_from_left_) {
  // ds create image message
  ImageMessagePtr image_message(
    new ImageMessage("/" + label_ + "/image_raw", "/" + label_, index_, timestamp_));

  // ds load RGB image from disk and convert
  cv::Mat image_opencv = cv::imread(file_path_image_, CV_LOAD_IMAGE_GRAYSCALE);
  if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
    throw std::runtime_error(
      "serializeCameraImageAndInfo|ERROR: invalid camera image: " + file_path_image_ +
      " with dimensions: " + std::to_string(image_opencv.rows) + " x " +
      std::to_string(image_opencv.cols));
  }

  // ds map to srrg
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
  camera_info_message->projection_model.setValue("pinhole");
  camera_info_message->distortion_model.setValue(distortion_model_name_);
  camera_info_message->distortion_coefficients.setValue(distortion_coefficients_);
  camera_info_message->camera_matrix.setValue(camera_matrix_);

  // ds provide relative transform between left and right camera
  TransformEventsMessagePtr transform_message(
    new TransformEventsMessage("/tf",
                               image_message->frame_id.value(),
                               image_message->seq.value(),
                               image_message->timestamp.value()));
  TransformEvent event_stereo(image_message->timestamp.value(),
                              "camera_right", // ds TODO change this to slashed version!
                              transformation_right_from_left_,
                              "camera_left"); // ds TODO change this to slashed version!
  TransformEvent event_anchor(image_message->timestamp.value(),
                              "imu", // ds TODO change this to slashed version!
                              imu_from_camera_,
                              label_); // ds TODO change this to slashed version!
  transform_message->events.resize(2);
  transform_message->events.setValue(0, event_stereo);
  transform_message->events.setValue(1, event_anchor);

  // ds write messages related to this image
  serializer_.writeObject(*camera_info_message);
  serializer_.writeObject(*transform_message);
  serializer_.writeObject(*image_message);
  if (label_ == "camera_left") {
    std::cerr << "L";
  } else if (label_ == "camera_right") {
    std::cerr << "R";
  } else {
    throw std::runtime_error("serializeCameraImageAndInfo|ERROR: unknown camera label");
  }

  // ds display converted images
  cv::imshow("processed image: " + label_, image_opencv);
  cv::waitKey(1);
  ++index_;
}
