#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include "../../../include/3rdparty/log/easylogging++.h"
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
  enum class State { UNKNOWN, UNAVAILABLE, ERROR, IDEL, EXECUTING, CHARGING };
  enum proState { AWAITING_ORDER, IDEL, PROCESSING_ORDER };
  enum integrationLevel {
    TO_BE_IGNORED,
    TO_BE_NOTICED,
    TO_BE_RESPECTED,
    TO_BE_UTILIZED
  };
  std::string get_state();
  std::string get_process_state();
  void receive_task(std::shared_ptr<data::order::TransportOrder>);
  void next_command();
  void execute_move(
      std::vector<std::shared_ptr<data::order::Step>>);  // 执行移动回调
  void execute_action(
      std::shared_ptr<data::order::DriverOrder::Destination>);  // 执行移动回调
                                                                // //
  void execute_instatn_action(std::shared_ptr<vda5050::instantaction::Action>);
  void command_done();  // 命令完成回调
  void plan_route();
  void reroute();
  void get_next_ord();
  void run();
  void cancel_all_order();
  void close();
  virtual bool action(
      std::shared_ptr<data::order::DriverOrder::Destination>) = 0;  // 执行动作
  virtual bool move(
      std::vector<std::shared_ptr<data::order::Step>>) = 0;  // 执行移动
  virtual void init(){};  // 初始化或者配置接收外部信息更新机器人状态
  virtual bool instant_action(
      std::shared_ptr<data::model::Actions::Action>) = 0;
  virtual ~Vehicle();

 public:
  int priority_level{0};
  int length;  // mm
  int width;
  int max_vel{0};
  int max_reverse_vel{0};
  int energy_level_good{70};      // 接收订单，无订单自动充电
  int energy_level_critical{15};  // 低于值不可移动
  int engrgy_level_recharge{35};  // 只充电不接订单
  int engrgy_level_full{90};      // 满电
  int engerg_level{100};
  bool process_chargeing{false};
  bool reroute_flag{false};
  integrationLevel integration_level{TO_BE_UTILIZED};
  std::string color;
  State state{State::UNKNOWN};
  proState process_state{proState::IDEL};
  bool paused{false};

  std::deque<std::shared_ptr<data::order::TransportOrder>> orders;
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<allocate::ResourceManager> resource;
  std::weak_ptr<allocate::OrderPool> orderpool;
  std::weak_ptr<planner::Planner> planner;
  std::shared_ptr<data::order::TransportOrder> current_order;
  std::shared_ptr<Command> current_command;
  std::shared_ptr<data::model::Point> current_point;
  std::shared_ptr<data::model::Point> last_point;
  std::shared_ptr<vda5050::instantaction::Action> current_action;
  std::shared_ptr<data::model::Point> park_point{nullptr};
  std::thread run_th;
  Eigen::Vector3i position{0, 0, 0};
  float angle{0};
  Eigen::Vector3i layout{0, 0, 0};
  fa::taskpool_t pool{1};          // 普通任务
  fa::taskpool_t instant_pool{1};  // 立即任务
  bool task_run{false};
  uint32_t send_queue_size{2};
  bool instanttask_run{false};
  std::chrono::system_clock::time_point idle_time;
  std::vector<std::string> allowed_order_type;
};
class SimVehicle : public Vehicle {
 public:
  using Vehicle::Vehicle;
  SimVehicle(int rate, std::string name) : rate(rate), Vehicle(name) {}
  bool action(std::shared_ptr<data::order::DriverOrder::Destination>) override;
  bool move(std::vector<std::shared_ptr<data::order::Step>>) override;
  bool instant_action(std::shared_ptr<data::model::Actions::Action>) override;
  void init() override;

 public:
  int rate{5};  // 时间快进比例
};

class Rabbit3 : public Vehicle {
 public:
  Rabbit3(const std::string& name, const std::string& interface_name,
          const std::string& serial_number, const std::string& version,
          const std::string& manufacturer)
      : Vehicle(name) {
    mqtt_cli = std::make_shared<vda5050::VehicleMaster>(
        interface_name, serial_number, version, manufacturer);
  }
  bool action(std::shared_ptr<data::order::DriverOrder::Destination>) override;
  bool move(std::vector<std::shared_ptr<data::order::Step>>) override;
  bool instant_action(std::shared_ptr<data::model::Actions::Action>) override;
  void init() override;
  void onstate(mqtt::const_message_ptr);
  void onconnect(mqtt::const_message_ptr);
  ~Rabbit3();

 public:
  std::shared_ptr<vda5050::VehicleMaster> mqtt_cli;
  vda5050::VehicleMqttStatus veh_state{vda5050::VehicleMqttStatus::OFFLINE};
  std::string broker_ip;
  int broker_port;
  vda5050::state ::VDA5050State vdastate;
  std::string map_id;
  int send_header_id{0};
  int order_id{-1};
  bool init_pos{false};
  int rece_header_id{-1};
};
class InvalidVehicle : public Vehicle {
 public:
  InvalidVehicle(const std::string& name) : Vehicle(name) {}
  bool action(std::shared_ptr<data::order::DriverOrder::Destination>) override {
    return false;
  }
  bool move(std::vector<std::shared_ptr<data::order::Step>>) override {
    return false;
  }
  bool instant_action(std::shared_ptr<data::model::Actions::Action>) override {
    return false;
  }
  void init() override {
    state = State::ERROR;
    process_state = proState::IDEL;
    integration_level = integrationLevel::TO_BE_IGNORED;
  }
};
}  // namespace driver
}  // namespace kernel
#endif