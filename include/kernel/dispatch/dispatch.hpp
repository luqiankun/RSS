#ifndef DISPATCH_HPP
#define DISPATCH_HPP

#include "../allocate/order.hpp"
#include "../allocate/resource.hpp"
#include "../planner/planner.hpp"
namespace kernel {
namespace dispatch {
class Dispatcher : public TCSObject {
 public:
  using TCSObject::TCSObject;

  std::shared_ptr<driver::Vehicle> select_vehicle(
      std::shared_ptr<data::model::Point>);
  std::set<std::shared_ptr<driver::Vehicle>> find_owners(
      const std::shared_ptr<driver::Vehicle>&);
  std::vector<std::shared_ptr<driver::Vehicle>>
  deadlock_loop();  // 死锁车辆环路
  void brake_deadlock(
      std::vector<std::shared_ptr<driver::Vehicle>>);  // TODO 解锁
  void dispatch_once();
  void idle_detect();
  void run();
  void stop();
  ~Dispatcher();

 public:
  std::thread dispatch_th;
  bool dispose{false};
  bool auto_select{true};  // 没有指定车辆时自动选择
  std::vector<std::shared_ptr<driver::Vehicle>> vehicles;
  /// signals
  std::function<std::pair<allocate::ResourceManager::ResType,
                          std::shared_ptr<TCSResource>>(const std::string&)>
      find_res;
  std::function<void(const std::string&, std::shared_ptr<driver::Vehicle>)>
      go_home;
  std::function<void(const std::string&, std::shared_ptr<driver::Vehicle>)>
      go_charge;
  std::function<std::shared_ptr<data::order::TransportOrder>()> get_next_ord;
};
}  // namespace dispatch
}  // namespace kernel
#endif