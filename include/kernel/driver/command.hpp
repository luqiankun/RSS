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
class Command : public RSSObject {
 public:
  using RSSObject::RSSObject;
  explicit Command(const std::string &);
  enum class State {
    INIT,
    ALLOCATING,
    ALLOCATED,
    EXECUTING,
    EXECUTED,
    END,
    DISPOSABLE,
    REDISBUTE
  };
  DestPtr get_dest(const DriverOrderPtr &);
  std::vector<StepPtr> get_step(const DriverOrderPtr &, uint32_t);
  std::vector<StepPtr> get_step_nopop(const DriverOrderPtr &, uint32_t);
  std::vector<std::shared_ptr<RSSResource>> get_future(
      const DriverOrderPtr &) const;
  std::vector<std::shared_ptr<RSSResource>> get_next_allocate_res(
      const DriverOrderPtr &, const std::shared_ptr<Vehicle> &);
  void vehicle_execute_cb(bool);  // 车辆通知动作结果
  void run_once();
  ~Command() override;

 public:
  State state{State::INIT};
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<driver::Vehicle> vehicle;
  TransOrderPtr order;
  std::function<void(std::vector<StepPtr>)> move;
  std::function<void(const DestPtr)> action;
  std::unordered_map<State, std::function<void()>> cbs;
};

}  // namespace driver
}  // namespace kernel

#endif