#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include "../../../include/3rdparty/uuid/uuid.hpp"
#include "../../../include/component/util/taskpool.hpp"
#include "../../../include/component/vda5050/master.hpp"
#include "../../../include/component/vda5050/vda5050insact.hpp"
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
  explicit Vehicle(const std::string &n) : schedule::Client(n) {}
  enum class State { UNKNOWN, UNAVAILABLE, ERROR, IDEL, EXECUTING, CHARGING };
  enum class proState { AWAITING_ORDER, IDEL, PROCESSING_ORDER };
  enum class nowOrder { BEGIN, END };
  enum integrationLevel {
    TO_BE_IGNORED,
    TO_BE_NOTICED,
    TO_BE_RESPECTED,
    TO_BE_UTILIZED
  };
  std::string get_state() const;
  std::string get_process_state() const;
  void receive_task(const std::shared_ptr<data::order::TransportOrder> &);
  void next_command();
  void execute_move(
      const std::vector<std::shared_ptr<data::order::Step>> &);  // 执行移动回调
  void execute_action(
      const std::shared_ptr<data::order::DriverOrder::Destination>
          &);  // 执行移动回调
               // //
  void execute_instatn_action(
      const std::shared_ptr<vda5050::instantaction::Action> &);
  void command_done();  // 命令完成回调
  bool plan_route(allocate::TransOrderPtr) const;
  void reroute();
  void get_next_ord();
  void run();
  void cancel_all_order();
  void close();
  virtual bool action(
      const std::shared_ptr<data::order::DriverOrder::Destination>
          &) = 0;  // 执行动作
  virtual bool move(
      const std::vector<std::shared_ptr<data::order::Step>> &) = 0;  // 执行移动
  virtual void init() {};  // 初始化或者配置接收外部信息更新机器人状态
  virtual bool instant_action(
      const std::shared_ptr<data::model::Actions::Action> &) = 0;
  ~Vehicle() override;

 public:
  int priority_level{0};
  int length{0};  // mm
  int width{0};
  int max_vel{0};
  int max_reverse_vel{0};
  int energy_level_good{70};      // 接收订单，无订单自动充电
  int energy_level_critical{15};  // 低于值不可移动
  int energy_level_recharge{35};  // 只充电不接订单
  int energy_level_full{90};      // 满电
  int energy_level{100};
  bool process_charging{false};
  bool reroute_flag{false};
  bool init_pos{false};
  integrationLevel integration_level{TO_BE_UTILIZED};
  std::string color;
  State state{State::UNKNOWN};
  proState process_state{proState::IDEL};
  bool paused{false};
  nowOrder now_order_state{nowOrder::END};
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
  // std::thread run_th;
  Eigen::Vector3i position{0, 0, 0};
  double angle{0};
  Eigen::Vector3i layout{0, 0, 0};
  tools::threadpool pool{1};       // 普通任务
  tools::threadpool instant_pool;  // 立即任务
  bool task_run{false};
  uint32_t send_queue_size{2};
  bool instant_task_run{false};
  std::chrono::system_clock::time_point idle_time;
  std::vector<std::string> allowed_order_type;
};
class SimVehicle : public Vehicle {
 public:
  using Vehicle::Vehicle;
  SimVehicle(int rate, const std::string &name) : Vehicle(name), rate(rate) {}
  bool action(
      const std::shared_ptr<data::order::DriverOrder::Destination> &) override;
  bool move(const std::vector<std::shared_ptr<data::order::Step>> &) override;
  bool instant_action(
      const std::shared_ptr<data::model::Actions::Action> &) override;
  void init() override;

 public:
  int rate{5};  // 时间快进比例
};

class Rabbit3 : public Vehicle {
 public:
  Rabbit3(const std::string &name, const std::string &interface_name,
          const std::string &serial_number, const std::string &version,
          const std::string &manufacturer)
      : Vehicle(name),
        mqtt_cli(std::make_shared<vda5050::VehicleMaster>(
            interface_name, serial_number, version, manufacturer)),
        deviation_xy(1),
        deviation_theta(0.17),
        dest_deviation_xy(0.05),
        dest_deviation_theta(0.034) {}
  bool action(
      const std::shared_ptr<data::order::DriverOrder::Destination> &) override;
  bool move(const std::vector<std::shared_ptr<data::order::Step>> &) override;
  bool instant_action(
      const std::shared_ptr<data::model::Actions::Action> &) override;
  void init() override;
  void onstate(const mqtt::const_message_ptr &);
  void onconnect(const mqtt::const_message_ptr &);
  bool run_script(const std::string &path,
                  std::map<std::string, std::string> param);
  ~Rabbit3() override;

 public:
  std::shared_ptr<vda5050::VehicleMaster> mqtt_cli;
  vda5050::VehicleMqttStatus veh_state{vda5050::VehicleMqttStatus::OFFLINE};
  std::string broker_ip{"127.0.0.1"};
  int broker_port{1883};
  vda5050::state ::VDA5050State vdastate;
  std::string map_id;
  int send_header_id{0};
  uuids::uuid order_id;
  int seq_id{0};
  int update_vda_order_id{0};
  int rece_header_id{-1};
  int last_step_count{0};
  double deviation_xy;
  double deviation_theta;
  double dest_deviation_xy;
  double dest_deviation_theta;
  uuids::uuid order_action_uuid;
  tools::threadpool python_pool;  // python任务
};
class InvalidVehicle : public Vehicle {
 public:
  explicit InvalidVehicle(const std::string &name) : Vehicle(name) {}
  bool action(
      const std::shared_ptr<data::order::DriverOrder::Destination> &) override {
    return false;
  }
  bool move(const std::vector<std::shared_ptr<data::order::Step>> &) override {
    return false;
  }
  bool instant_action(
      const std::shared_ptr<data::model::Actions::Action> &) override {
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