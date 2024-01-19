#ifndef TCS_HPP
#define TCS_HPP
#include "../component/xml/pugixml.hpp"
#include "../kernel/dispatch/dispatch.hpp"
#include "../kernel/schedule/schedule.hpp"
#ifdef VISUAL
#include "../component/visualization/visualizer.hpp"
#endif
#ifndef MY_EASYLOG_CONIFG  //(path) 按天滚动 最大10MB
#define MY_EASYLOG_CONIFG(path)                                            \
  auto data = get_date_fmt();                                              \
  el::Configurations conf;                                                 \
  conf.setGlobally(el::ConfigurationType::Format,                          \
                   "[%level] %datetime %fbase:%line] %msg");               \
  conf.setGlobally(el::ConfigurationType::Filename,                        \
                   (std::string(path) + "/tcs_" + data + ".log").c_str()); \
  conf.setGlobally(el::ConfigurationType::MillisecondsWidth, "4");         \
  conf.setGlobally(el ::ConfigurationType::MaxLogFileSize, "10485760");    \
  conf.set(el::Level::Debug, el::ConfigurationType::Format,                \
           "%datetime{%d/%M} %func [%fbase:%line] %msg");                  \
  el::Loggers::reconfigureAllLoggers(conf);
#endif
class TCS : public std::enable_shared_from_this<TCS> {
 public:
  bool init_all(const std::string& xml_path, double r = 30);
  bool init_resource(const std::string& xml_path);
  bool init_dispatcher();
  bool init_scheduler();
  bool init_orderpool();

  void run();
  ~TCS();

 public:
#ifdef VISUAL
  std::shared_ptr<visual::Visualizer> visualizer;
  bool init_visualizer(double);
#endif
 public:
  std::shared_ptr<kernel::allocate::ResourceManager> resource;
  std::shared_ptr<kernel::schedule::Scheduler> scheduler;
  std::shared_ptr<kernel::dispatch::Dispatcher> dispatcher;
  std::shared_ptr<kernel::allocate::OrderPool> orderpool;
};
#endif