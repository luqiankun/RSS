#ifndef TCSRESOURCE_HPP
#define TCSRESOURCE_HPP
#include "tcsobject.hpp"
class TCSResource : public TCSObject {
 public:
  virtual ~TCSResource() = default;
  using TCSObject::TCSObject;
  std::weak_ptr<TCSObject> owner;
  std::vector<std::weak_ptr<TCSObject>> future_owner;
  std::map<std::string, std::weak_ptr<TCSObject>> envelopes;
};
#endif