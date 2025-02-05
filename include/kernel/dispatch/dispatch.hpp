#ifndef DISPATCH_HPP
#define DISPATCH_HPP

#include <condition_variable>

#include "../allocate/order.hpp"
#include "../allocate/resource.hpp"
class RSS;
// #include "../planner/planner.hpp"
namespace kernel::dispatch {
class ConflictPool;
using VehPtr = std::shared_ptr<driver::Vehicle>;
class Dispatcher : public RSSObject {
 public:
  using RSSObject::RSSObject;
  VehPtr select_vehicle(const allocate::PointPtr &);
  std::set<VehPtr> find_depends(const VehPtr &);
  std::vector<VehPtr> deadlock_loop();                // 死锁车辆环路
  std::vector<VehPtr> block_loop();                   // 阻挡环路
  void brake_deadlock(const std::vector<VehPtr> &);   // TODO 解锁
  void brake_blocklock(const std::vector<VehPtr> &);  // TODO 移动避让

  void dispatch_once();
  void idle_detect();
  void run();
  void stop();
  void notify() { cv.notify_all(); }
  ~Dispatcher() override;

 public:
  std::shared_ptr<ConflictPool> conflict_pool;
  std::thread dispatch_th;
  bool dispose{false};
  bool auto_select{true};  // 没有指定车辆时自动选择
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
  std::function<std::pair<std::string, allocate::TransOrderPtr>()> get_next_ord;
  std::function<bool()> order_empty;
};
const float kMaxOccupancy = 1;
class Conflict {
 public:
  enum class State {
    Raw,          // 初始化状态
    END,          // 结束
    Solved,       // 已解决
    Dispatching,  // 正在调度
    Dispatched,   // 调度完成
    SelfMove,     // 主车辆需要移动
    Err,          // 无解
  };
  float occupancy(std::shared_ptr<data::model::Alleyway>, VehPtr);  // 占用率
  float distance(allocate::PointPtr, allocate::PointPtr);  // 路径代价
  bool one_of_other(std::pair<VehPtr, allocate::PointPtr>,
                    std::pair<VehPtr, allocate::PointPtr>);
  void swap_obj(std::pair<VehPtr, allocate::PointPtr>,
                std::pair<VehPtr, allocate::PointPtr>);
  allocate::PointPtr select_point(VehPtr, std::vector<allocate::PointPtr>,
                                  bool = false);
  bool is_swap_conflict(std::vector<allocate::PointPtr> p, VehPtr v);
  void solve_once();
  void update();

 public:
  State state{Conflict::State::Raw};
  std::weak_ptr<allocate::ResourceManager> resource_manager;
  std::weak_ptr<planner::Planner> planner;
  std::weak_ptr<Dispatcher> dispatcher;
  std::weak_ptr<allocate::OrderPool> orderpool;
  std::shared_ptr<data::model::Point> obj;
  std::vector<allocate::PointPtr> path;
  allocate::TransOrderPtr order;
  std::stack<VehPtr> vehicles;
  std::set<VehPtr> rm_depends;
  std::map<VehPtr, allocate::PointPtr> dispthed;
  std::deque<std::pair<VehPtr, allocate::PointPtr>> graph;
};
class ConflictPool : public std::enable_shared_from_this<ConflictPool> {
 public:
  ConflictPool(std::shared_ptr<RSS> rss) : rss(rss) {}
  Conflict::State get_state(allocate::TransOrderPtr);
  void solve(allocate::TransOrderPtr);
  void reset_state(allocate::TransOrderPtr);

 private:
  std::weak_ptr<RSS> rss;
  std::unordered_map<allocate::TransOrderPtr, std::shared_ptr<Conflict>>
      conflicts;
  std::mutex mut;
};
}  // namespace kernel::dispatch
#endif