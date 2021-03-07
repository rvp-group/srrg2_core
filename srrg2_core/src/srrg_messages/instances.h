#pragma once
//#include "messages/base_sensor_message.h"
#include "messages/camera_info_message.h"
#include "messages/image_message.h"
#include "messages/imu_message.h"
#include "messages/joints_message.h"
#include "messages/laser_message.h"
#include "messages/navsat_fix_message.h"
#include "messages/odometry_message.h"
#include "messages/point_cloud2_message.h"
#include "messages/point_stamped_message.h"
#include "messages/range_message.h"
#include "messages/ticks_message.h"
#include "messages/transform_events_message.h"
#include "messages/twist_stamped_message.h"
#include "messages/pose_message.h"
#include "messages/pose_array_message.h"
#include "messages/pose_stamped_message.h"
#include "messages/path_message.h"
#include "messages/pose_with_covariance_stamped_message.h"

#include "message_handlers/message_file_source.h"
#include "message_handlers/message_odom_subsampler_source.h"
#include "message_handlers/message_pack.h"
#include "message_handlers/message_sorted_source.h"
#include "message_handlers/message_source_platform.h"
#include "message_handlers/message_synchronized_source.h"

#include "message_handlers/message_file_sink.h"
#include "message_handlers/message_selector_sink.h"
#include "message_handlers/message_sorted_sink.h"
#include "message_handlers/message_synchronized_sink.h"
#include "message_handlers/message_odom_subsampler_sink.h"
#include "message_handlers/message_platform_listener_sink.h"

namespace srrg2_core {
  void messages_registerTypes() __attribute__((constructor));
}
