#ifndef DISPATCH_HPP
#define DISPATCH_HPP

#include "../allocate/order.hpp"
#include "../allocate/resource.hpp"
#include "../planner/planner.hpp"
namespace kernel {
namespace dispatch {
using VehPtr = std::shared_ptr<driver::Vehicle>;
class Dispatcher : public TCSObject {
 public:
  using TCSObject::TCSObject;
  VehPtr select_vehicle(allocate::PointPtr);
  std::set<VehPtr> find_owners(const VehPtr&);
  std::vector<VehPtr> deadlock_loop();       // 死锁车辆环路
  void brake_deadlock(std::vector<VehPtr>);  // TODO 解锁
  void dispatch_once();
  void idle_detect();
  void run();
  void stop();
  void notify() { cv.notify_all(); }
  ~Dispatcher();

 public:
  std::thread dispatch_th;
  bool dispose{false};
  bool auto_select{true};  // 没有指定车辆时自动选择
  std::vector<VehPtr> vehicles;
  std::mutex mut;
  std::condition_variable cv;
  /// signals
  std::function<std::pair<allocate::ResourceManager::ResType,
                          allocate::TCSResourcePtr>(const std::string&)>
      find_res;
  std::function<void(const std::string&, VehPtr)> go_home;
  std::function<void(const std::string&, VehPtr)> go_charge;
  std::function<allocate::TransOrderPtr()> get_next_ord;
  std::function<bool()> order_empty;
};
}  // namespace dispatch
}  // namespace kernel
#endif