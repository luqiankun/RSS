#ifndef ORDER_HPP
#define ORDER_HPP
#include "../../component/data/order/orderquence.hpp"
namespace kernel {
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
  void pop(const TransOrderPtr &order);
  void push(const TransOrderPtr &order);
  void patch(const TransOrderPtr &order);
  void redistribute(const TransOrderPtr &order);
  std::pair<std::string, TransOrderPtr> get_next_ord();
  std::pair<std::string, TransOrderPtr> get_next_random_ord();
  bool random_list_empty() { return random_orderpool.empty(); }
  bool idel_orderpool(std::string);
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
  std::vector<std::pair<std::string, std::deque<TransOrderPtr>>> orderpool;
  std::deque<TransOrderPtr> random_orderpool;
  std::deque<TransOrderPtr> ended_orderpool;
  std::deque<OrderSeqPtr> orderquence;
  mutable std::shared_mutex mut;
  int cur_index{0};
};
}  // namespace allocate
}  // namespace kernel

#endif