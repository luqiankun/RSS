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
  std::set<VehPtr> find_depends(const VehPtr &);     // 查找声明的资源所在的车辆
  std::vector<VehPtr> deadlock_loop();               // 死锁车辆环路
  std::vector<VehPtr> block_loop();                  // 阻挡环路
  void brake_deadlock(const std::vector<VehPtr> &);  // TODO 解锁
  void brake_blocklock(const std::vector<VehPtr> &);  // TODO 移动避让

  void dispatch_once();  // 调度一次
  void idle_detect();    // 检测空闲
  void run();
  void stop();
  void notify() {
    cv.notify_all();
    cv_preprocess.notify_all();  // 通知预处理线程
  }
  ~Dispatcher() override;

 public:
  std::shared_ptr<ConflictPool> conflict_pool;  // 冲入解决器
  std::thread dispatch_th;
  std::thread preprocess_th;
  bool dispose{false};
  bool auto_select{true};  // 没有指定车辆时自动选择
  std::vector<VehPtr> vehicles;
  std::mutex mut;
  std::condition_variable cv;
  std::mutex mut_preprocess;
  std::condition_variable cv_preprocess;
  /// signals
  std::function<std::pair<allocate::ResourceManager::ResType,
                          allocate::TCSResourcePtr>(const std::string &)>
      find_res;                                              // 查找资源
  std::function<void(const std::string &, VehPtr)> go_home;  // 下单回停靠点
  std::function<allocate::PointPtr(allocate::PointPtr)>
      get_park_point;                                          // 获取最近停车点
  std::function<void(const std::string &, VehPtr)> go_charge;  // 下单回充电点
  std::function<void(allocate::TransOrderPtr)> pop_order;      // 弹出
  std::function<void(allocate::TransOrderPtr)> pathc_order;    // 重新处理一次
  std::function<std::pair<std::string, allocate::TransOrderPtr>()>
      get_next_ord;                  // 获取下一个订单
  std::function<void()> preprocess;  // 预处理
  std::function<std::pair<std::string, allocate::TransOrderPtr>()>
      get_next_random_ord;                  // 获取下一个随机订单
  std::function<bool()> random_list_empty;  // 随机订单列表是否为空
  std::function<bool()> raw_list_empty;     // 原始订单列表是否为空

  std::function<bool()> order_empty;  // 订单池是否为空
};
const float kMaxOccupancy = 0.9;
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
  float distance(allocate::PointPtr, allocate::PointPtr);           // 路径代价
  bool one_of_other(std::pair<VehPtr, allocate::PointPtr>,
                    std::pair<VehPtr, allocate::PointPtr>);  // 路径是否包含
  void swap_obj(std::pair<VehPtr, allocate::PointPtr>,
                std::pair<VehPtr, allocate::PointPtr>);  // 交换目标点
  allocate::PointPtr select_point(VehPtr, std::vector<allocate::PointPtr>,
                                  bool = false);  // 选择目标点
  bool is_swap_conflict(std::vector<allocate::PointPtr> p,
                        VehPtr v);  // 是否交换冲突
  void solve_once();                // 解决一次
  int update();                     // 更新

 public:
  State state{Conflict::State::Raw};
  std::weak_ptr<allocate::ResourceManager> resource_manager;
  std::weak_ptr<planner::Planner> planner;
  std::weak_ptr<Dispatcher> dispatcher;
  std::weak_ptr<allocate::OrderPool> orderpool;
  std::shared_ptr<data::model::Point> obj;
  std::vector<allocate::PointPtr> path;
  allocate::TransOrderPtr order;
  std::stack<VehPtr> vehicles;                    // 阻挡车辆
  std::set<VehPtr> rm_depends;                    // 排除的点，视为不阻挡
  std::map<VehPtr, allocate::PointPtr> dispthed;  // 已派遣的
  std::deque<std::pair<VehPtr, allocate::PointPtr>> graph;  // 冲突解决方案
};
/**
 * @brief 冲突池
 *
 */
class ConflictPool : public std::enable_shared_from_this<ConflictPool> {
 public:
  ConflictPool(std::shared_ptr<RSS> rss) : rss(rss) {}
  Conflict::State get_state(allocate::TransOrderPtr);  // 获取冲突状态
  void solve(allocate::TransOrderPtr);                 // 解决一次
  void reset_state(allocate::TransOrderPtr);           // 重置冲突状态

 private:
  std::weak_ptr<RSS> rss;
  std::unordered_map<allocate::TransOrderPtr, std::shared_ptr<Conflict>>
      conflicts;  // 订单冲突池，
  std::mutex mut;
};
}  // namespace kernel::dispatch
#endif