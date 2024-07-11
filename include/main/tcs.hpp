#ifndef TCS_HPP
#define TCS_HPP
#include "../component/tools/json/json.hpp"
#include "../component/tools/xml/pugixml.hpp"
#include "../kernel/dispatch/dispatch.hpp"
#include "../kernel/schedule/schedule.hpp"
inline std::string get_log_path(const std::string& path) {
  auto data = get_time_fmt(std::chrono::system_clock::now());
#ifdef _WIN32
  return (std::string(path) + "\\" + data + ".log");
#else
  return (std::string(path) + "/" + data + ".log");
#endif
}
inline std::string get_log_name(const std::string& path) {
  auto data = "main";
#ifdef _WIN32
  return (std::string(path) + "\\tcs_" + data + ".log");
#else
  return (std::string(path) + "/tcs_" + data + ".log");
#endif
}
using json = nlohmann::json;
class TCS : public std::enable_shared_from_this<TCS> {
 public:
  TCS(const std::string& ip, int port) : ip(ip), port(port) {}
  // init
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
  std::pair<int, std::string> put_vehicle_paused(const std::string&, bool);
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
  std::pair<int, std::string> get_view();
  std::pair<int, std::string> post_reroute();
  std::pair<int, std::string> post_vehicle_reroute(const std::string&, bool);
  std::pair<int, std::string> put_vehicle_enable(const std::string&, bool);
  std::pair<int, std::string> put_vehicle_integration_level(const std::string&,
                                                            const std::string&);
  std::pair<int, std::string> post_vehicle_path_to_point(const std::string&,
                                                         const std::string&);

 public:
  bool init_dispatcher();
  bool init_scheduler();
  bool init_orderpool();
  bool init_planner();
  bool is_connect(std::shared_ptr<data::model::Point>,
                  std::shared_ptr<data::model::Point>);
  void home_order(const std::string& name,
                  std::shared_ptr<kernel::driver::Vehicle> v);
  void charge_order(const std::string& name,
                    std::shared_ptr<kernel::driver::Vehicle> v);
  void cancel_order(const std::string& order_name);  // 取消某个订单
  void cancel_all_order();                           // 取消所有订单
  void cancel_vehicle_all_order(
      const std::string& vehicle_name);  // 取消某辆车所有订单
  void run();
  void add_vehicle(const std::string& type, const std::string& name);
  void paused_vehicle(const std::string& name);
  void recovery_vehicle(const std::string& name);
  std::string get_vehicles_step();
  void reroute();
  void stop();
  ~TCS();

 public:
  bool is_run{false};
  std::string ip;
  int port;
  std::shared_ptr<kernel::allocate::ResourceManager> resource;
  std::shared_ptr<kernel::schedule::Scheduler> scheduler;
  std::shared_ptr<kernel::dispatch::Dispatcher> dispatcher;
  std::shared_ptr<kernel::allocate::OrderPool> orderpool;
  std::shared_ptr<kernel::planner::Planner> planner;
};
#endif