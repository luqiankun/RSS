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
  DestPtr get_dest(const DriverOrderPtr &);  // 获取目的地
  std::vector<StepPtr> get_step(const DriverOrderPtr &, uint32_t);  // 获取步骤
  std::vector<StepPtr> get_step_nopop(const DriverOrderPtr &,
                                      uint32_t);  // 获取步骤,但不弹出
  std::vector<std::shared_ptr<RSSResource>> get_future(
      const DriverOrderPtr &) const;  // 获取未来要锁定的资源
  std::vector<std::shared_ptr<RSSResource>> get_next_allocate_res(
      const DriverOrderPtr &,
      const std::shared_ptr<Vehicle> &);  // 获取下一步要锁定的资源
  void vehicle_execute_cb(bool);          // 车辆通知动作结果
  void run_once();                        // 处理一次
  ~Command() override;

 public:
  State state{State::INIT};
  std::weak_ptr<schedule::Scheduler> scheduler;
  std::weak_ptr<driver::Vehicle> vehicle;          // 所属车辆
  TransOrderPtr order;                             // 当前订单
  std::function<void(std::vector<StepPtr>)> move;  // 绑定车辆移动回调
  std::function<void(const DestPtr)> action;       // 绑定车辆动作回调
  std::unordered_map<int, std::function<void()>> cbs;  // 回调函数
};

}  // namespace driver
}  // namespace kernel

#endif