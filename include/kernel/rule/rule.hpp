#ifndef RULE_HPP
#define RULE_HPP
#include "../../component/data/order/orderquence.hpp"
namespace kernel {
namespace schedule {
class Client;
}
namespace planner {
class Planner;
}
namespace allocate {
class ResourceManager;
class RuleBase : public TCSObject {
 public:
  RuleBase(const std::string& name, std::shared_ptr<ResourceManager> r)
      : TCSObject(name), res(r){};
  using TCSObject::TCSObject;
  virtual bool pass(std::shared_ptr<schedule::Client>) = 0;

 public:
  std::string color;
  std::weak_ptr<ResourceManager> res;
  std::unordered_set<std::shared_ptr<TCSObject>> owners;
  std::unordered_set<std::shared_ptr<TCSResource>> occs;
};
class OnlyOneGatherRule : public RuleBase {
 public:
  using RuleBase::RuleBase;
  bool pass(std::shared_ptr<schedule::Client>) override;

 public:
  int32_t n{1};
};
class OnlyOneRectRule : public OnlyOneGatherRule {
 public:
  using OnlyOneGatherRule::OnlyOneGatherRule;

  void get_occupys();

 public:
  float x{0};
  float y{0};
  float height{0};
  float width{0};
};
}  // namespace allocate
}  // namespace kernel
#endif