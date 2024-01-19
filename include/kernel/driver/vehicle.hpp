#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include <boost/asio.hpp>

#include "../../../include/component/log/easylogging++.h"
#include "../../component/data/order/orderquence.hpp"
#include "../../component/tcsobject.hpp"
#include "../schedule/schedule.hpp"
#if BOOST_VERSION >= 107001
using io_service_type = boost::asio::io_context;
#else
using io_service_type = boost::asio::io_service;
#endif
namespace kernel {
namespace dispatch {
class Dispatcher;
}
namespace driver {
class Vehicle : public schedule::Client,
                public std::enable_shared_from_this<Vehicle> {
 public:
  Vehicle(const std::string& n) : schedule::Client(n) {
    ctx = std::make_shared<io_service_type>();
    auto* st = new io_service_type::strand(*ctx);
    strand = std::shared_ptr<io_service_type::strand>(st);
  }
  enum class State { UNKNOWN, UNAVAILABLE, ERROR, IDLE, EXECUTING, CHARGING };
  enum class ProcState { IDLE, AWAITING_ORDER, PROCESSING_ORDER };
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
  void close();
  virtual bool action(
      std::shared_ptr<data::order::DriverOrder::Destination>) = 0;  // 执行动作
  virtual bool move(std::shared_ptr<data::order::Step>) = 0;  // 执行移动
  virtual void update() = 0;  // 接收外部信息更新机器人状态
  virtual ~Vehicle();

 public:
  int priority_level;
  int length;  // mm
  int width;
  int max_vel;
  int max_reverse_vel;
  int engerg_level;
  std::string color;
  ProcState proc_state{ProcState::IDLE};
  State state{State::IDLE};
  bool paused;
  std::deque<std::shared_ptr<data::order::TransportOrder>> orders;
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<dispatch::Dispatcher> dispatcher;
  std::shared_ptr<data::order::TransportOrder> current_order;
  std::shared_ptr<Command> current_command;
  std::shared_ptr<data::model::Point> current_point;
  std::shared_ptr<data::model::Point> init_point;
  std::thread run_th;
  Eigen::Vector3i position;
  float angle{0};
  Eigen::Vector3i layout;
  std::shared_ptr<io_service_type> ctx;
  std::shared_ptr<io_service_type::strand> strand;
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
// TODO 工厂
}  // namespace driver
}  // namespace kernel
#endif