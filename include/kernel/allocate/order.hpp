#ifndef ORDER_HPP
#define ORDER_HPP
#include "../../component/data/order/orderquence.hpp"
namespace kernel {
namespace planner {
class Planner;
}
namespace allocate {
using DriverOrderPtr = std::shared_ptr<data::order::DriverOrder>;
using DestPtr = std::shared_ptr<data::order::DriverOrder::Destination>;
using RoutePtr = std::shared_ptr<data::order::Route>;
using OpType = data::order::DriverOrder::Destination::OpType;
using TransOrderPtr = std::shared_ptr<data::order::TransportOrder>;
using OrderSeqPtr = std::shared_ptr<data::order::OrderSequence>;
// 每个机器人的订单派发顺序
struct OrderCmp {
  bool operator()(const TransOrderPtr &a, const TransOrderPtr &b) const {
    // assert(a->intended_vehicle.lock() == b->intended_vehicle.lock());
    if (a->priority < b->priority) {
      return true;
    } else if (a->priority > b->priority) {
      return false;
    } else {
      // 优先级相同按创建时间排序
      if (a->anytime_drop && b->anytime_drop) {
        // 都是避让订单
        return a->create_time < b->create_time;
      } else
        return a->create_time > b->create_time;
    }
  }
};
class OrderPool : public RSSObject {
 public:
  DriverOrderPtr route_to_driverorder(const RoutePtr &route,
                                      const DestPtr &dest);
  DestPtr res_to_destination(const std::shared_ptr<RSSResource> &res,
                             OpType op);
  void cancel_all_order();
  void cancel_order(size_t order_uuid);
  /**
   * @brief 弹出订单，放入ended队列中
   *
   * @param order
   */
  void pop(const TransOrderPtr &order);
  /**
   * @brief 加入订单，加入到随机队列或车辆队列池中
   *
   * @param order
   */
  void push(const TransOrderPtr &order);
  /**
   * @brief 加入订单，加入到未处理队列中
   *
   * @param order
   */
  void push_raw(const TransOrderPtr &order);
  /**
   * @brief 从订单队列中拿出，重新push
   *
   * @param order
   */
  void patch(const TransOrderPtr &order);
  /**
   * @brief 重新派发订单
   *
   * @param order
   */
  void redistribute(const TransOrderPtr &order);
  /**
   * @brief 预处理
   *
   * @param order
   */
  void preprocess();
  /**
   * @brief 获取下一个订单池里的订单
   * 
   * @return std::pair<std::string, TransOrderPtr> 
   */
  std::pair<std::string, TransOrderPtr> get_next_ord();
  /**
   * @brief 获取下一个随机订单
   *
   * @return std::pair<std::string, TransOrderPtr>
   */
  std::pair<std::string, TransOrderPtr> get_next_random_ord();
  bool random_list_empty() {
    std::shared_lock<std::shared_mutex> lock(mut);
    return random_orderpool.empty();
  }
  bool idel_orderpool(std::string);
  bool raw_list_empty() {
    std::shared_lock<std::shared_mutex> lock(mut);
    return raw_orderpool.empty();
  }

  ~OrderPool() { CLOG(INFO, allocate_log) << name << " close\n"; }
  void update_quence() const;
  bool is_empty() {
    if (!random_orderpool.empty()) return false;
    for (auto &x : orderpool) {
      if (!x.second.empty()) return false;
    }
    return true;
  }
  std::vector<TransOrderPtr> get_all_order();

 public:
  using RSSObject::RSSObject;
  std::vector<std::pair<std::string, std::deque<TransOrderPtr>>>
      orderpool;                               // 一个车辆对应一个订单队列
  std::deque<TransOrderPtr> random_orderpool;  // 随机队列
  std::deque<TransOrderPtr> ended_orderpool;   // 操作结束的订单
  std::deque<TransOrderPtr> raw_orderpool;     // 未处理的订单
  std::deque<OrderSeqPtr> orderquence;
  std::weak_ptr<planner::Planner> planner;
  std::map<std::string, std::shared_ptr<data::model::Point>>
      veh_ps;  // 车辆的某个可达位置
  mutable std::shared_mutex mut;
  int cur_index{0};
};
}  // namespace allocate
}  // namespace kernel

#endif