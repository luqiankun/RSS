#ifndef DISPATCH_HPP
#define DISPATCH_HPP

#include <condition_variable>

#include "../allocate/order.hpp"
#include "../allocate/resource.hpp"
// #include "../planner/planner.hpp"
namespace kernel::dispatch {
using VehPtr = std::shared_ptr<driver::Vehicle>;
class Dispatcher : public RSSObject {
public:
  using RSSObject::RSSObject;
  VehPtr select_vehicle(const allocate::PointPtr &);
  std::set<VehPtr> find_depends(const VehPtr &);
  std::vector<VehPtr> deadlock_loop();               // 死锁车辆环路
  std::vector<VehPtr> block_loop();                  // 阻挡环路
  void brake_deadlock(const std::vector<VehPtr> &);  // TODO 解锁
  void brake_blocklock(const std::vector<VehPtr> &); // TODO 移动避让

  void dispatch_once();
  void idle_detect();
  void run();
  void stop();
  void notify() { cv.notify_all(); }
  ~Dispatcher() override;

public:
  std::thread dispatch_th;
  bool dispose{false};
  bool auto_select{true}; // 没有指定车辆时自动选择
  std::vector<VehPtr> vehicles;
  std::mutex mut;
  std::condition_variable cv;
  /// signals
  std::function<std::pair<allocate::ResourceManager::ResType,
                          allocate::TCSResourcePtr>(const std::string &)>
      find_res;
  std::function<void(const std::string &, VehPtr)> go_home;
  std::function<allocate::PointPtr(allocate::PointPtr)> get_park_point;
  std::function<void(const std::string &, VehPtr)> go_charge;
  std::function<void(allocate::TransOrderPtr)> pop_order;
  std::function<std::pair<std::string, std::vector<allocate::TransOrderPtr>>()>
      get_next_vec;
  std::function<bool()> order_empty;
};
}
#endif