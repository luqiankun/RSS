#ifndef RULE_HPP
#define RULE_HPP
#include <unordered_set>

#include "../../component/data/order/route.hpp"

namespace kernel {
namespace schedule {
class Client;
}
namespace planner {
class Planner;
}
namespace allocate {
class ResourceManager;
class RuleBase : public RSSObject {
 public:
  RuleBase(const std::string &name, const std::shared_ptr<ResourceManager> &r)
      : RSSObject(name), res(r) {};
  using RSSObject::RSSObject;
  virtual bool pass(std::vector<std::shared_ptr<RSSResource>> res,
                    std::shared_ptr<schedule::Client>) = 0;

 public:
  std::weak_ptr<ResourceManager> res;
};

class OwnerRule : public RuleBase {
 public:
  using RuleBase::RuleBase;
  bool pass(std::vector<std::shared_ptr<RSSResource>>,
            std::shared_ptr<schedule::Client>) override;
};

class CollisionRule : public RuleBase {
 public:
  using RuleBase::RuleBase;
  bool pass(std::vector<std::shared_ptr<RSSResource>> res,
            std::shared_ptr<schedule::Client>) override;
};

class BlockRuleBase : public RuleBase {
 public:
  using RuleBase::RuleBase;

 public:
  std::string color;
  std::unordered_set<std::shared_ptr<RSSObject>> owners;
  std::unordered_set<std::shared_ptr<RSSResource>> occs;
  std::string type;
};
class OnlyOneDirectRule : public BlockRuleBase {
 public:
  using BlockRuleBase::BlockRuleBase;
  bool pass(std::vector<std::shared_ptr<RSSResource>>,
            std::shared_ptr<schedule::Client>) override;
  void init();
  int cur_direct{0};  // 0 1 2
  std::shared_ptr<data::order::Route> route;
  std::map<std::shared_ptr<data::model::Path>, int> table;
};
class OnlyOneGatherRule : public BlockRuleBase {
 public:
  using BlockRuleBase::BlockRuleBase;
  bool pass(std::vector<std::shared_ptr<RSSResource>>,
            std::shared_ptr<schedule::Client>) override;

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