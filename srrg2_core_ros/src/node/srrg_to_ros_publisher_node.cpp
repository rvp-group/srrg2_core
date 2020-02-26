#include <csignal>
//srrg SRRG
#include <srrg_system_utils/system_utils.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include "srrg_messages_ros/instances.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

const char* banner[] = {
  "",
  "srrg_to_ros_publisher_node: publish srrg_messages on ros topics",
  "",
  "usage:",
  " ./srrg_to_ros_publisher_node [options] <boss_file>",
  " where [options]: ",
  0

};

volatile bool run = true;

static void sigIntHandler(int __attribute__((unused))) {
  std::cerr << "Got user interrupt, Terimnating" << std::endl;
  run = false;
}

void generateConfig(const std::string& config_file_,
                    const std::string& config_name_source_,
                    const std::string& config_name_sink_) {
  ConfigurableManager manager;

  auto source = manager.create<MessageFileSource>(config_name_source_);
  auto sink   = manager.create<MessageFileSink>(config_name_sink_);
  manager.write(config_file_);

}


int main(int argc, char** argv) {

  if (argc < 2) {
    srrg2_core::printBanner(banner);
    return 0;
  }

  messages_ros_registerTypes();
  ParseCommandLine cmd_line(argv, banner);

  std::string config_name = "srrg_to_ros_publisher_node";
  std::string config_file = config_name + ".conf";
  std::string config_name_source = config_name + "_source";
  std::string config_name_sink = config_name + "_sink";

  ros::init(argc, argv, config_name);

  //srrg check for options and params
  ArgumentFlag arg_generate_config(&cmd_line, "j", "generate-config", "generates a config file");
  ArgumentString arg_config_file(&cmd_line, "c", "config-file", "path to config file", config_file);
  ArgumentString arg_config_name_source(&cmd_line, "sr", "config-source", "name of the configuration", config_name_source);
  ArgumentString arg_config_name_sink(&cmd_line, "sk", "config-sink", "name of the configuration", config_name_sink);
  cmd_line.parse();

  config_file = arg_config_file.value();
  config_name_source = arg_config_name_source.value();
  config_name_sink = arg_config_name_sink.value();

  //srrg check for options and params
  if (arg_generate_config.isSet()) {
    generateConfig(config_file, config_name_source, config_name_sink);
    return 0;
  }


  if (cmd_line.lastParsedArgs().empty()) {
    std::cerr << "no input file specified, exiting" << std::endl;
    return 0;
  }
  const std::string& input_boss_file = cmd_line.lastParsedArgs()[0];

  signal(SIGINT, sigIntHandler);

  //srrg Load configurations
  ConfigurableManager config_manager;
  config_manager.read(config_file);
  MessageFileSourcePtr source = config_manager.getByName<MessageFileSource>(config_name_source);
  if (!source) {
    std::cerr << "cast fail" << std::endl;
    return -1;
  }

  std::shared_ptr<MessageROSSink> sink = config_manager.getByName<MessageROSSink>(config_name_sink);
  if (!sink) {
    std::cerr << "cast fail" << std::endl;
    return -1;
  }

  //srrg open the tubes

  source->open(input_boss_file);
  sink->open();
  BaseSensorMessagePtr msg;

  //srrg TODO handle the correct timing
  //     maybe use prev_time - curr_time, then put msg instead of sleep
  ros::Rate r(50);
  SystemUsageCounter usage_counter;
  int num_msgs_processed = 0;

  double processing_time = 0;

#warning "srrg: I am not be able to handle the interrupt.. please check it for me! "
#warning "  rosrun srrg2_core_ros srrg_to_ros_publisher_node out.boss"
  while (run && (msg = source->getMessage())) {
    usage_counter.tic();
    sink->putMessage(msg);
    std::cerr << "msg: " << num_msgs_processed++ << "\n";
    processing_time += usage_counter.toc();

    if (num_msgs_processed > 100) {
      break;
    }
    r.sleep();
  }

  config_manager.write(config_file);
  std::cerr << "Done" << std::endl;
  std::cerr << "Processed " << num_msgs_processed
            << " msg in " << processing_time
            << "sec (" << num_msgs_processed / processing_time << " frames/sec)" << std::endl;
  std::cerr << "Avg Memory: " << usage_counter.totalMemory() / 1024 << " Kb" << std::endl;


  sink->close();
  std::cerr << "sink closed... ";
  source->close();
  std::cerr << "source closed." << std::endl;

  return 0;

}
