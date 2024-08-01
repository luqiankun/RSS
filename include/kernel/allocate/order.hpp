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
class OrderPool : public TCSObject {
 public:
  DriverOrderPtr route_to_driverorder(RoutePtr route, DestPtr dest);
  DestPtr res_to_destination(const std::shared_ptr<TCSResource>& res,
                             OpType op);
  void cancel_all_order();
  void cancel_order(size_t order_uuid);
  TransOrderPtr pop();
  ~OrderPool() { CLOG(INFO, allocate_log) << name << " close\n"; }
  void update_quence();
  bool is_empty() { return orderpool.empty(); }

 public:
  using TCSObject::TCSObject;
  std::deque<TransOrderPtr> orderpool;
  std::deque<TransOrderPtr> ended_orderpool;
  std::deque<OrderSeqPtr> orderquence;
};
}  // namespace allocate
}  // namespace kernel

#endif