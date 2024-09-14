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
  std::pair<std::string, std::vector<TransOrderPtr>> get_next_vec();
  ~OrderPool() { CLOG(INFO, allocate_log) << name << " close\n"; }
  void update_quence() const;
  bool is_empty() { return orderpool.empty(); }

public:
  using RSSObject::RSSObject;
  std::vector<std::pair<std::string, std::vector<TransOrderPtr>>> orderpool;
  std::deque<TransOrderPtr> ended_orderpool;
  std::deque<OrderSeqPtr> orderquence;
  int cur_index{0};
};
} // namespace allocate
} // namespace kernel

#endif