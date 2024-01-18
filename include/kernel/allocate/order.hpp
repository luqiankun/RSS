#ifndef ORDER_HPP
#define ORDER_HPP
#include "../../component/data/order/orderquence.hpp"

namespace kernel {
namespace allocate {
class OrderPool : public TCSObject {
 public:
 public:
  using TCSObject::TCSObject;
  std::deque<std::shared_ptr<data::order::TransportOrder>> orderpool;
  std::deque<std::shared_ptr<data::order::TransportOrder>> ended_orderpool;
};
}  // namespace allocate
}  // namespace kernel

#endif