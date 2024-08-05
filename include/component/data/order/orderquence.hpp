#ifndef ORDERQUENCE_HPP
#define ORDERQUENCE_HPP
#include "transportorder.hpp"
namespace data {
namespace order {
class OrderSequence : public RSSObject {
 public:
  using RSSObject::RSSObject;
  enum class Type {
    TYPE_ANY,
    TYPE_NONE,
    TYPE_CHARGE,
    TYPE_PARK,
    TYPE_TRANSPORT
  };

  std::string get_type() {
    if (type == Type::TYPE_ANY) {
      return "ANY";
    } else if (type == Type::TYPE_NONE) {
      return "NONE";
    } else if (type == Type::TYPE_CHARGE) {
      return "CHARGE";
    } else if (type == Type::TYPE_TRANSPORT) {
      return "TRANSPORT";
    } else {
      return "PARK";
    }
  }
  bool add_transport_ord(std::shared_ptr<TransportOrder> ord) {
    if (complete) {
      return false;
    } else {
      orders.push_back(ord);
      return true;
    }
  }

 public:
  std::deque<std::shared_ptr<TransportOrder>> orders;
  Type type{Type::TYPE_NONE};
  int finished_index{-1};
  bool complete{false};
  bool finished{false};
  bool failure_fatal{false};
  std::weak_ptr<kernel::driver::Vehicle> intended_vehicle;
  std::weak_ptr<kernel::driver::Vehicle> processing_vehicle;
};
}  // namespace order
}  // namespace data
#endif