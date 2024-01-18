#include <thread>

#include "../../../include/component/log/easylogging++.h"
INITIALIZE_EASYLOGGINGPP

int main(int argc, const char** argv) {
  START_EASYLOGGINGPP(argc, argv);
  // Load configuration from file
  el::Configurations conf;
  // Reconfigure single logger
  // if (!conf.parseFromFile("/home/luqk/c++/TCS/config/log.conf")) {
  conf.setGlobally(el::ConfigurationType::Format,
                   "[%levshort] %datetime %fbase:%line] %msg");
  conf.setGlobally(el::ConfigurationType::Filename,
                   "/home/luqk/c++/TCS/build/logs/tcs.log");
  conf.set(el::Level::Debug, el::ConfigurationType::Format,
           "%datetime{%d/%M} %func [%fbase:%line] %msg");
  // }
  el::Loggers::reconfigureLogger("default", conf);
  // Actually reconfigure all loggers instead
  el::Loggers::reconfigureAllLoggers(conf);
  // Now all the loggers will use configuration from file
  LOG(DEBUG) << "Log using default file";
  LOG(WARNING) << "Log using default file";
  LOG(TRACE) << "Log using default file";
  LOG(ERROR) << "Log using default file";
  LOG(INFO) << "Log using default file";
  std::thread t{[] {
    int i = 0;
    while (i < 10) {
      LOG(WARNING) << i++;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }};
  std::this_thread::sleep_for(std::chrono::seconds(3));
  t = std::thread{[] {
    int i = 10;
    while (i < 18) {
      LOG(WARNING) << i++;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }};
  t.join();
}