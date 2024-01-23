#ifndef TCS_HPP
#define TCS_HPP
#include "../component/tools/xml/pugixml.hpp"
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

using Oper = std::tuple<
    std::string,
    data::order::DriverOrder::Destination::OpType>;  // 目标点和动作类型

class TCS : public std::enable_shared_from_this<TCS> {
 public:
  bool init_all(const std::string& xml_path, double r = 30);
  bool init_resource(const std::string& xml_path);
  bool init_dispatcher();
  bool init_scheduler();
  bool init_orderpool();
  bool init_planner();
  void add_task(
      std::vector<Oper> ops, const std::string& name,
      const std::string& v_name = "none");  // strategy<0 自动  >=0 指定执行端
  void cancel_order(const std::string& order_name);  // 取消某个订单
  void cancel_all_order();                           // 取消所有订单
  void cancel_vehicle_all_order(
      const std::string& vehicle_name);  // 取消某辆车所有订单
  void run();
  void add_vehicle(const std::string& type, const std::string& name);  // TODO
  void paused_vehicle(const std::string& name);
  void recovery_vehicle(const std::string& name);

  void stop();
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
  std::shared_ptr<kernel::planner::Planner> planner;
};
#endif