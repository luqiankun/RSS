#ifndef ORDERQUENCE_HPP
#define ORDERQUENCE_HPP
#include "transportorder.hpp"
namespace data {
namespace order {
class OrderSequence : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class Type {
    TYPE_ANY,
    TYPE_NONE,
    TYPE_CHARGE,
    TYPE_PARK,
    TYPE_TRANSPORT
  };
  std::optional<TransportOrder> get_next() {
    if (finished || (finished_index + 1 >= orders.size())) {
      return std::nullopt;
    } else {
      return orders.at(finished_index + 1);
    }
  }

 public:
  std::deque<TransportOrder> orders;
  Type type;
  int finished_index{-1};
  bool complete;
  bool finished;
  bool failure_fatal;
  std::weak_ptr<kernel::driver::Vehicle> intended_vehicle;
  std::weak_ptr<kernel::driver::Vehicle> processing_vehicle;
};
}  // namespace order
}  // namespace data
#endif