#ifndef MOVECOMMAND_HPP
#define MOVECOMMAND_HPP

#include "../../component/data/order/transportorder.hpp"
namespace kernel {
namespace schedule {
class Scheduler;
}
namespace driver {
class Vehicle;
class Command : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class State {
    INIT,
    CLAIMING,
    CLAIMED,
    EXECUTING,
    EXECUTED,
    END,
    DISPOSABLE
  };
  std::shared_ptr<data::order::DriverOrder::Destination> get_dest(
      std::shared_ptr<data::order::DriverOrder>);
  std::shared_ptr<data::order::Step> get_step(
      std::shared_ptr<data::order::DriverOrder>);
  std::shared_ptr<data::order::Step> get_step_nopop(
      std::shared_ptr<data::order::DriverOrder>);
  std::vector<std::shared_ptr<TCSResource>> get_future(
      std::shared_ptr<data::order::DriverOrder>);
  void vehicle_execute_cb(bool);  // 车辆通知动作结果
  void run_once();
  ~Command() { LOG(INFO) << name << " drop"; }

 public:
  State state{State::INIT};
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<driver::Vehicle> vehicle;
  std::shared_ptr<data::order::TransportOrder> order;
  std::function<void(const std::shared_ptr<data::order::Step>)> move;
  std::function<void(
      const std::shared_ptr<data::order::DriverOrder::Destination>)>
      action;
};

}  // namespace driver
}  // namespace kernel

#endif