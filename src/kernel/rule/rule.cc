#include "../../../include/kernel/rule/rule.hpp"

#include "../../../include/kernel/allocate/resource.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
#include "../../../include/kernel/schedule/schedule.hpp"

namespace kernel {
namespace allocate {
void OnlyOneRectRule::get_occupys() {
  occs.clear();
  for (auto& x : res.lock()->points) {
    if (x->position.x >= this->x && x->position.y >= this->y &&
        (x->position.x <= this->x + width) &&
        (x->position.y <= this->y + height)) {
      occs.insert(x);
    }
  }
  for (auto& x : res.lock()->points) {
    if (x->position.x >= this->x && x->position.y >= this->y &&
        (x->position.x <= this->x + width) &&
        (x->position.y <= this->y + height)) {
      occs.insert(x);
    }
  }
  for (auto& x : res.lock()->locations) {
    if (x->position.x >= this->x && x->position.y >= this->y &&
        (x->position.x <= this->x + width) &&
        (x->position.y <= this->y + height)) {
      occs.insert(x);
    }
  }
}

bool OnlyOneGatherRule::pass(std::vector<std::shared_ptr<TCSResource>>,
                             std::shared_ptr<schedule::Client> client) {
  owners.clear();
  for (auto& x : occs) {
    if (x->owner.lock()) {
      owners.insert(x->owner.lock());
    }
  }
  if (n < 0) {
    return true;
  } else {
    int32_t con{static_cast<int32_t>(owners.size())};
    if (owners.find(client) != owners.end()) {
      con = owners.size() - 1;
    }
    if (con < n) {
      return true;
    } else {
      return false;
    }
  }
}

bool OwnerRule::pass(std::vector<std::shared_ptr<TCSResource>> resource,
                     std::shared_ptr<schedule::Client> client) {
  for (auto& r : resource) {
    for (auto& p : res.lock()->points) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        auto it = p->owner.lock();
        if (!it || it == client) {
        } else {
          return false;
        }
      }
    }
    for (auto& p : res.lock()->paths) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        auto it = p->owner.lock();
        if (!it || it == client) {
        } else {
          return false;
        }
      }
    }
    for (auto& p : res.lock()->locations) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        auto it = p->owner.lock();
        if (!it || it == client) {
        } else {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace allocate
}  // namespace kernel