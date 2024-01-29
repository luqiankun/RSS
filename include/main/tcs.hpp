#ifndef TCS_HPP
#define TCS_HPP
#include "../component/tools/json/json.hpp"
#include "../component/tools/xml/pugixml.hpp"
#include "../kernel/dispatch/dispatch.hpp"
#include "../kernel/schedule/schedule.hpp"
#ifdef VISUAL
#include "../component/visualization/visualizer.hpp"
#endif

inline std::string get_log_path(const std::string& path) {
  auto data = get_date_fmt();
#ifdef _WIN32
  return (std::string(path) + "\\tcs_" + data + ".log");
#else
  return (std::string(path) + "/tcs_" + data + ".log");
#endif
}

#ifndef MY_EASYLOG_CONIFG  //(path) 按天滚动 最大10MB
#define MY_EASYLOG_CONIFG(path)                                         \
  el::Configurations conf;                                              \
  conf.setGlobally(el::ConfigurationType::Format,                       \
                   "[%level] %datetime %fbase:%line] %msg");            \
  conf.setGlobally(el::ConfigurationType::Filename,                     \
                   get_log_path(path).c_str());                         \
  conf.setGlobally(el::ConfigurationType::MillisecondsWidth, "4");      \
  conf.setGlobally(el ::ConfigurationType::MaxLogFileSize, "10485760"); \
  conf.set(el::Level::Debug, el::ConfigurationType::Format,             \
           "%datetime{%d/%M} %func [%fbase:%line] %msg");               \
  el::Loggers::reconfigureAllLoggers(conf);
#define MY_EASYLOG_CONIFG_COMPLEX(path, fmt, size)                 \
  el::Configurations conf;                                         \
  conf.setGlobally(el::ConfigurationType::Format, fmt);            \
  conf.setGlobally(el::ConfigurationType::Filename,                \
                   get_log_path(path).c_str());                    \
  conf.setGlobally(el::ConfigurationType::MillisecondsWidth, "4"); \
  conf.setGlobally(el ::ConfigurationType::MaxLogFileSize, size);  \
  conf.set(el::Level::Debug, el::ConfigurationType::Format,        \
           "%datetime{%d/%M} %func [%fbase:%line] %msg");          \
  el::Loggers::reconfigureAllLoggers(conf);
#endif

using json = nlohmann::json;
class TCS : public std::enable_shared_from_this<TCS> {
 public:
  // http interface

  // transportorder
  std::pair<int, std::string> get_transport_orders(
      const std::string& vehicle = "");
  std::pair<int, std::string> get_transport_order(const std::string& ord_name);
  std::pair<int, std::string> post_transport_order(const std::string& ord_name,
                                                   const std::string& body);
  std::pair<int, std::string> post_transport_order_withdrawl(
      const std::string& ord_name, bool immediate = false,
      bool disableVehicle = false);
  // ordsequence
  std::pair<int, std::string> get_ordersequences(
      const std::string& vehicle = "");
  std::pair<int, std::string> get_ordersequence(
      const std::string& sequence_name);
  std::pair<int, std::string> post_ordersequence(
      const std::string& sequence_name, const std::string& body);
  // vehicle
  std::pair<int, std::string> get_vehicles(const std::string& state = "");
  std::pair<int, std::string> get_vehicle(const std::string& vehicle);
  std::pair<int, std::string> post_vehicle_withdrawl(
      const std::string& vehicle, bool immediate = false,
      bool disableVehicle = false);
  // model
  std::pair<int, std::string> get_model();
  std::pair<int, std::string> put_model(const std::string& body);
  std::pair<int, std::string> put_path_locked(const std::string& path_name,
                                              bool new_value);
  std::pair<int, std::string> put_location_locked(const std::string& loc_name,
                                                  bool new_value);
  std::pair<int, std::string> put_model_xml(const std::string& body);

 public:
  bool init_all(const std::string& xml_path, double r = 30);
  bool init_dispatcher();
  bool init_scheduler();
  bool init_orderpool();
  bool init_planner();
  void home_order(const std::string& name,
                  std::shared_ptr<kernel::driver::Vehicle> v);
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
  bool init_visualizer(double = 80);
#endif
 public:
  bool is_run{false};
  std::shared_ptr<kernel::allocate::ResourceManager> resource;
  std::shared_ptr<kernel::schedule::Scheduler> scheduler;
  std::shared_ptr<kernel::dispatch::Dispatcher> dispatcher;
  std::shared_ptr<kernel::allocate::OrderPool> orderpool;
  std::shared_ptr<kernel::planner::Planner> planner;
};
#endif