#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include "../../../include/component/tools/json/json.hpp"
#include "../../../include/component/tools/log/easylogging++.h"
#include "../../../include/component/util/taskpool.hpp"
#include "../../../include/component/vda5050/master.hpp"
#include "../../component/data/order/orderquence.hpp"
#include "../../component/tcsobject.hpp"
#include "../allocate/order.hpp"
#include "../planner/planner.hpp"
#include "../schedule/schedule.hpp"

namespace kernel {
namespace dispatch {
class Dispatcher;
}
namespace driver {
class Vehicle : public schedule::Client,
                public std::enable_shared_from_this<Vehicle> {
 public:
  Vehicle(const std::string& n) : schedule::Client(n) {}
  enum class State { UNKNOWN, UNAVAILABLE, ERROR, IDLE, EXECUTING, CHARGING };
  enum class ProcState { IDLE, AWAITING_ORDER, PROCESSING_ORDER };
  std::string get_state();
  std::string get_proc_state();
  static std::optional<std::string> get_proc_state(ProcState);

  void receive_task(std::shared_ptr<data::order::TransportOrder>);
  void next_command();
  void execute_move(std::shared_ptr<data::order::Step>);  // 执行移动回调
  void execute_action(
      std::shared_ptr<data::order::DriverOrder::Destination>);  // 执行移动回调
                                                                // //
  void command_done();  // 命令完成回调
  void plan_route();
  void get_next_ord();
  void run();
  void cancel_all_order();
  void close();
  virtual bool action(
      std::shared_ptr<data::order::DriverOrder::Destination>) = 0;  // 执行动作
  virtual bool move(std::shared_ptr<data::order::Step>) = 0;  // 执行移动
  virtual void update() = 0;  // 接收外部信息更新机器人状态
  virtual void init(){};
  virtual ~Vehicle();

 public:
  int priority_level{0};
  int length;  // mm
  int width;
  int max_vel{0};
  int max_reverse_vel{0};
  int energy_level_good{0};
  int energy_level_critical{0};
  int engrgy_level_recharge{0};
  int engrgy_level_full{0};
  int engerg_level{100};
  std::string integration_level;
  std::string color;
  ProcState proc_state{ProcState::IDLE};
  State state{State::UNKNOWN};
  bool paused{false};
  std::deque<std::shared_ptr<data::order::TransportOrder>> orders;
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<allocate::ResourceManager> resource;
  std::weak_ptr<allocate::OrderPool> orderpool;
  std::weak_ptr<planner::Planner> planner;
  std::shared_ptr<data::order::TransportOrder> current_order;
  std::shared_ptr<Command> current_command;
  std::shared_ptr<data::model::Point> current_point;
  std::shared_ptr<data::model::Point> init_point;
  std::thread run_th;
  std::thread update_th;
  data::Vector3i position;
  float angle{0};
  data::Vector3i layout;
  fa::taskpool_t pool{1};
  bool task_run{false};
  std::chrono::system_clock::time_point idle_time;
};
class SimVehicle : public Vehicle {
 public:
  using Vehicle::Vehicle;
  bool action(std::shared_ptr<data::order::DriverOrder::Destination>) override;
  bool move(std::shared_ptr<data::order::Step>) override;
  void update() override;

 public:
  int rate{5};  // 时间快进比例
};

class Rabbit3 : public Vehicle, public vda5050::VehicleMaster {
 public:
  Rabbit3(const std::string& name, const std::string& interface_name,
          const std::string& serial_number, const std::string& version,
          const std::string& manufacturer)
      : Vehicle(name),
        vda5050::VehicleMaster(interface_name, serial_number, version,
                               manufacturer) {}
  bool action(std::shared_ptr<data::order::DriverOrder::Destination>) override;
  bool move(std::shared_ptr<data::order::Step>) override;
  void update() override;
  void init() override;
  void onstate(mqtt::const_message_ptr);
  void onconnect(mqtt::const_message_ptr);

 public:
  vda5050::VehicleMqttStatus veh_state{vda5050::VehicleMqttStatus::OFFLINE};
  std::string broker_ip;
  int broker_port;
  nlohmann::json state_json;
  std::string map_id;
  int send_header_id{0};
  int order_id{-1};
  bool init_pos{false};
  int rece_header_id{-1};
};

// TODO 工厂
}  // namespace driver
}  // namespace kernel
#endif