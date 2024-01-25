#ifndef ORDER_HPP
#define ORDER_HPP
#include "../../component/data/order/orderquence.hpp"

namespace kernel {
namespace allocate {
class OrderPool : public TCSObject {
 public:
  std::shared_ptr<data::order::DriverOrder> route_to_driverorder(
      std::shared_ptr<data::order::Route> route,
      std::shared_ptr<data::order::DriverOrder::Destination> dest);
  std::shared_ptr<data::order::DriverOrder::Destination> res_to_destination(
      const std::shared_ptr<TCSResource>& res,
      data::order::DriverOrder::Destination::OpType op);
  void cancel_all_order();
  void cancel_order(size_t order_uuid);
  std::shared_ptr<data::order::TransportOrder> pop();
  ~OrderPool() { LOG(INFO) << name << " close"; }
  void update_quence();

 public:
  using TCSObject::TCSObject;
  std::deque<std::shared_ptr<data::order::TransportOrder>> orderpool;
  std::deque<std::shared_ptr<data::order::TransportOrder>> ended_orderpool;
  std::deque<std::shared_ptr<data::order::OrderSequence>> orderquence;
};
}  // namespace allocate
}  // namespace kernel

#endif