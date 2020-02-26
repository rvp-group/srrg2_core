#pragma once
#include "slam_benchmark_suite.hpp"
#include "srrg_config/configurable_manager.h"

namespace srrg2_core {

  // ds 3D benchmark wrapper for: http://www.cvlibs.net/datasets/kitti/index.php
  // ds this dataset contains stereo vision and lidar data in SE(3)
  class SLAMBenchmarkSuiteKITTI : public SLAMBenchmarkSuite<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = Isometry3f;

    virtual ~SLAMBenchmarkSuiteKITTI() {
    }

    void loadDataset(const std::string& filepath_,
                     const size_t& number_of_message_packs_to_read_ = -1,
                     const size_t& number_of_message_pack_to_start_ = 0) override {
      _dataset_path                    = filepath_;
      _number_of_message_packs_to_read = number_of_message_packs_to_read_;
      _number_of_message_pack_to_start = number_of_message_pack_to_start_;

      // ds load AIS dataset from disk
      MessageFileSourcePtr source(new MessageFileSource());
      MessageSortedSourcePtr sorter(new MessageSortedSource());
      source->open(filepath_);
      if (!source->isOpen()) {
        throw std::runtime_error("SLAMBenchmarkSuiteKITTI::loadDataset|unable to load dataset: " +
                                 filepath_);
      }
      sorter->param_source.setValue(source);
      sorter->param_time_interval.setValue(0.1);

      // ds set up synchronizer for KITTI datasets
      _synchronizer = MessageSynchronizedSourcePtr(new MessageSynchronizedSource());
      _synchronizer->param_source.setValue(sorter);
      _synchronizer->param_topics.value().push_back("/camera_left/image_raw");
      _synchronizer->param_topics.value().push_back("/camera_right/image_raw");
      _synchronizer->param_topics.value().push_back("/camera_left/image_raw/info");
      _synchronizer->param_topics.value().push_back("/camera_right/image_raw/info");
      _synchronizer->param_topics.value().push_back("/tf");
      _synchronizer->param_time_interval.setValue(0.1);
      std::cerr << "SLAMBenchmarkSuiteKITTI::loadDataset|configured message playback for dataset: '"
                << filepath_ << "'" << std::endl;
      std::cerr << "SLAMBenchmarkSuiteKITTI::loadDataset|topics: " << std::endl;
      for (const std::string& topic : _synchronizer->param_topics.value()) {
        std::cerr << topic << std::endl;
      }
    }

    void loadGroundTruth(const std::string& filepath_,
                         const std::string& filepath_additional_ = std::string()) override {
      _ground_truth_path = filepath_;

      // ds open ground truth file in KITTI format: # R t (3x4)
      std::ifstream ground_truth_file(filepath_, std::ios::in);
      if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteKITTI::loadGroundTruth|unable to read ground truth file: " +
          filepath_);
      }

      // ds KITTI stores timestamps in a separate file
      std::ifstream timestamps_file(filepath_additional_, std::ios::in);
      if (!timestamps_file.good() || !timestamps_file.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteKITTI::loadGroundTruth|unable to read timestamps file: " +
          filepath_additional_);
      }
      _relative_ground_truth_poses.clear();

      // ds relative estimate computation
      double previous_timestamp_seconds = 0;
      Isometry3f previous_pose(Isometry3f::Identity());

      // ds read file by tokens
      double timestamp_seconds = 0;
      double r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz;
      while (ground_truth_file >> r00 >> r01 >> r02 >> tx >> r10 >> r11 >> r12 >> ty >> r20 >>
             r21 >> r22 >> tz) {
        // ds assemble isometry
        Isometry3f pose(Isometry3f::Identity());
        pose.translation() = Vector3f(tx, ty, tz);
        pose.linear() << r00, r01, r02, r10, r11, r12, r20, r21, r22;

        // ds retrieve timestamp from timestamps file
        std::string buffer;
        if (!std::getline(timestamps_file, buffer)) {
          throw std::runtime_error(
            "SLAMBenchmarkSuiteKITTI::loadGroundTruth|unable to parse timestamp");
        }
        timestamp_seconds = std::stod(buffer);

        // ds compute relative estimate to previous pose (except for the first measurement)
        if (timestamp_seconds - previous_timestamp_seconds > 0) {
          // ds insertion must succeed otherwise there are duplicates!
          _relative_ground_truth_poses.push_back(RelativeEstimateStamped(
            pose.inverse() * previous_pose, previous_timestamp_seconds, timestamp_seconds));
        }
        previous_timestamp_seconds = timestamp_seconds;
        previous_pose              = pose;
      }

      std::cerr << "SLAMBenchmarkSuiteKITTI::loadGroundTruth|loaded relative poses: "
                << _relative_ground_truth_poses.size() << std::endl;
      ground_truth_file.close();
    }

    BaseSensorMessagePtr getMessage() override {
      BaseSensorMessagePtr message = _synchronizer->getMessage();
      ++_number_of_message_packs_read;

      // ds start gobbling up messages after the specified number (default 0, i.e. first message)
      while (_number_of_message_packs_read < _number_of_message_pack_to_start) {
        std::cerr << "SLAMBenchmarkSuiteKITTI::getMessage|skipping messsage: "
                  << message->seq.value() << std::endl;
        message = _synchronizer->getMessage();
        ++_number_of_message_packs_read;
      }

      // ds finished source or early termination
      if (!message || _number_of_message_packs_read - _number_of_message_pack_to_start >
                        _number_of_message_packs_to_read) {
        return nullptr;
      }

      // ds must be a message pack
      MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
      if (!message_pack) {
        throw std::runtime_error("SLAMBenchmarkSuiteKITTI::getMessage|invalid message type: " +
                                 message->name());
      }

      // ds load color and depth image from disk (buffered)
      // ds this is needed if we want to have buffered playback, otherwise the image links break
      ImageMessagePtr image_message_left(
        std::dynamic_pointer_cast<ImageMessage>(message_pack->messages[0]));
      ImageMessagePtr image_message_right(
        std::dynamic_pointer_cast<ImageMessage>(message_pack->messages[1]));
      if (!image_message_left || !image_message_right) {
        throw std::runtime_error("SLAMBenchmarkSuiteKITTI::getMessage|wrong order of "
                                 "messages in pack check synchronizer configuration)");
      }

      // ds done
      return message_pack;
    }

    void reset() override {
      // ds TODO implement proper reset playback
      throw std::runtime_error("SLAMBenchmarkSuiteKITTI::reset|ERROR: not implemented");
      _synchronizer->resetCounters();
    }

    void writeTrajectoryToFile(const std::string& filename_ = "trajectory.txt") const override {
      std::ofstream outfile(filename_, std::ofstream::out);
      if (!outfile.good() || !outfile.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteKITTI::writeTrajectoryToFile|unable to open file: " + filename_);
      }
      outfile << std::fixed;
      outfile << std::setprecision(9);
      for (const auto& entry : _estimated_poses) {
        const EstimateType& pose_estimate(entry.estimate);
        for (size_t r = 0; r < 3; ++r) {
          for (size_t c = 0; c < 4; ++c) {
            outfile << pose_estimate.matrix()(r, c) << " ";
          }
        }
        outfile << std::endl;
      }
      outfile.clear();
    }

  protected:
    // ds we're loading messages on the spot as buffering image data consumes too much memory
    MessageSynchronizedSourcePtr _synchronizer;

    // ds live statistics/configuration
    size_t _number_of_message_packs_read    = 0;
    size_t _number_of_message_packs_to_read = 0;
    size_t _number_of_message_pack_to_start = 0;
  };

} // namespace srrg2_core
