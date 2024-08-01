#ifndef MOVECOMMAND_HPP
#define MOVECOMMAND_HPP

#include "../../component/data/order/transportorder.hpp"
namespace kernel {
namespace schedule {
class Scheduler;
}
namespace driver {
using DestPtr = std::shared_ptr<data::order::DriverOrder::Destination>;
using DriverOrderPtr = std::shared_ptr<data::order::DriverOrder>;
using StepPtr = std::shared_ptr<data::order::Step>;
using TransOrderPtr = std::shared_ptr<data::order::TransportOrder>;
class Vehicle;
class Command : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class State {
    INIT,
    ALLOCATING,
    ALLOCATED,
    EXECUTING,
    EXECUTED,
    END,
    DISPOSABLE
  };
  DestPtr get_dest(DriverOrderPtr);
  std::vector<StepPtr> get_step(DriverOrderPtr, uint32_t);
  std::vector<StepPtr> get_step_nopop(DriverOrderPtr, uint32_t);
  std::vector<std::shared_ptr<TCSResource>> get_future(DriverOrderPtr);
  void vehicle_execute_cb(bool);  // 车辆通知动作结果
  void run_once();
  ~Command() { CLOG(INFO, driver_log) << name << " drop\n"; }

 public:
  State state{State::INIT};
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<driver::Vehicle> vehicle;
  TransOrderPtr order;
  std::function<void(std::vector<StepPtr>)> move;
  std::function<void(const DestPtr)> action;
};

}  // namespace driver
}  // namespace kernel

#endif