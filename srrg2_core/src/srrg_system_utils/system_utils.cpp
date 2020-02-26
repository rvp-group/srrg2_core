#include "system_utils.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace srrg2_core {

  int srrg_argc         = 0;
  char** srrg_argv      = 0;
  std::string srrg_opts = "";
  void srrgInit(int argc, char** argv, const char* opts) {
    srrg_argc = argc;
    srrg_argv = argv;
    if (opts) {
      srrg_opts = opts;
    }
    int ret __attribute__((unused)) = putenv((char*) "LC_ALL='en_US.UTF-8'");
  }

  // ds timing
  double SystemUsageCounter::_time_seconds_last_tic = 0;

  using namespace std;

  void printBanner(const char** banner_) {
    const char** b = banner_;
    while (*b) {
      std::cerr << *b << std::endl;
      b++;
    }
  }

  double getTime() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv2sec(tv);
  }

  std::string getTimestamp() {
    // ds obtain current time as count
    timeval time_value;
    gettimeofday(&time_value, 0);

    // ds compute milliseconds
    const int milliseconds = time_value.tv_usec / 1000;

    // ds obtain structured time information (hours, minutes, seconds)
    struct tm* time_info = localtime(&time_value.tv_sec);

    // ds stream to formatted string
    char buffer[13];
    std::snprintf(buffer,
                  13,
                  "%02i:%02i:%02i.%03i",
                  time_info->tm_hour,
                  time_info->tm_min,
                  time_info->tm_sec,
                  milliseconds);
    return std::string(buffer);
  }

  std::string removeStringTokens(const std::string& input_string_, bool at_begining_only_) {
    char blacklisted_char[] = {'/', '\\', ' '};
    // std::cerr << input_string_ << std::endl;
    std::string output_string(input_string_);
    for (const char& c : blacklisted_char) {
      if (at_begining_only_) {
        if (c == input_string_[0]) {
          output_string = input_string_.substr(1);
          break;
        }
      } else {
        output_string.erase(std::remove(output_string.begin(), output_string.end(), c),
                            output_string.end());
      }
    }
    // std::cerr << output_string << std::endl;
    return output_string;
  }

  SystemUsageCounter::SystemUsageCounter() : _system_cpu(0), _user_cpu(0), _total_memory(0) {
    gettimeofday(&_last_update_time, NULL);
    getrusage(RUSAGE_SELF, &_last_usage);
  }

  void SystemUsageCounter::update() {
    struct rusage current_usage;
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    getrusage(RUSAGE_SELF, &current_usage);

    struct timeval aux;
    // compute the total time elapsed between two update calls
    timersub(&_last_update_time, &current_time, &aux);
    double time_lapse = aux.tv_sec * 1000 + aux.tv_usec * 1e-3;

    // compute the total cpu time in user space
    timersub(&_last_usage.ru_utime, &current_usage.ru_utime, &aux);
    double cpu_user_time = aux.tv_sec * 1000 + aux.tv_usec * 1e-3;
    _user_cpu            = cpu_user_time / time_lapse;

    // compute the total cpu time in system space
    timersub(&_last_usage.ru_stime, &current_usage.ru_stime, &aux);
    // double cpu_system_time = aux.tv_sec*1000+aux.tv_usec*1e-3;
    _last_update_time = current_time;
    _last_usage       = current_usage;

    std::ifstream is("/proc/self/statm");
    if (is) {
      is >> _total_memory; // >> _resident >> _share >> _text >> _lib >> _data >> _dt;
    } else {
      _total_memory = 0;
    }
    is.close();
  }

  const bool isAccessible(const std::string& file_or_directory_) {
    // ds stat handle
    struct stat info;

    // ds check if element on disk is accessible
    if (stat(file_or_directory_.c_str(), &info) == 0) {
      return true;
    } else {
      return false;
    }
  }
} // namespace srrg2_core
