#include "../../../include/kernel/rule/rule.hpp"

#include "../../../include/component/data/model/envelope.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
#include "../../../include/kernel/schedule/schedule.hpp"

namespace kernel::allocate {
void OnlyOneRectRule::get_occupys() {
  occs.clear();
  for (auto &x : res.lock()->points) {
    if (x->position.x() >= this->x && x->position.y() >= this->y &&
        (x->position.x() <= this->x + width) &&
        (x->position.y() <= this->y + height)) {
      occs.insert(x);
    }
  }
  for (auto &x : res.lock()->paths) {
    if ((x->source_point.lock()->position.x() >= this->x) &&
        (x->source_point.lock()->position.y() >= this->y) &&
        (x->source_point.lock()->position.x() <= this->x + width) &&
        (x->source_point.lock()->position.y() <= this->y + height) &&
        (x->destination_point.lock()->position.x() >= this->x) &&
        (x->destination_point.lock()->position.y() >= this->y) &&
        (x->destination_point.lock()->position.x() <= this->x + width) &&
        (x->destination_point.lock()->position.y() <= this->y + height)) {
      occs.insert(x);
    }
  }
  for (auto &x : res.lock()->locations) {
    if (x->position.x() >= this->x && x->position.y() >= this->y &&
        (x->position.x() <= this->x + width) &&
        (x->position.y() <= this->y + height)) {
      occs.insert(x);
    }
  }
}

bool OnlyOneGatherRule::pass(std::vector<std::shared_ptr<RSSResource>> res,
                             std::shared_ptr<schedule::Client> client) {
  bool flag = false;
  for (auto &x : res) {
    if (occs.find(x) != occs.end()) {
      flag = true;
      break;
    }
  }
  if (!flag) {
    return true;
  }
  owners.clear();
  for (auto &x : occs) {
    if (x->owner.lock() && x->owner.lock() != client) {
      owners.insert(x->owner.lock());
    }
  }
  if (n < 0) {
    return true;
  } else {
    return true;
  }
}

bool OwnerRule::pass(std::vector<std::shared_ptr<RSSResource>> resource,
                     std::shared_ptr<schedule::Client> client) {
  for (auto &r : resource) {
    for (auto &p : res.lock()->points) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        auto it = p->owner.lock();
        if (!it || it == client) {
        } else {
          return false;
        }
      }
    }
    for (auto &p : res.lock()->paths) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        auto it = p->owner.lock();
        if (!it || it == client) {
        } else {
          return false;
        }
      }
    }
    for (auto &p : res.lock()->locations) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
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

std::shared_ptr<data::model::Envelope> get_envelope(
    const std::shared_ptr<RSSResource> &res, const std::string &key) {
  if (const auto it = res->envelopes.find(key); it != res->envelopes.end()) {
    return std::dynamic_pointer_cast<data::model::Envelope>(it->second);
  }
  return nullptr;
}

bool CollisionRule::pass(std::vector<std::shared_ptr<RSSResource>> resource,
                         std::shared_ptr<schedule::Client> client) {
  for (auto &r : resource) {
    auto client_envelope = get_envelope(r, client->envelope_key);
    // LOG(INFO) << client_envelope;
    if (!client_envelope) {
      // TODO 是否需要返回失败？
      continue;
    }
    for (auto &key : r->envelope_keys) {
      if (key == client->envelope_key) {
        continue;
      }
      if (auto other_envelope = get_envelope(r, key)) {
        if (client_envelope->collide(*other_envelope)) {
          return false;
        }
      }
    }
  }
  return true;
}
bool AlleywayRule::pass(std::vector<std::shared_ptr<RSSResource>> resource,
                        std::shared_ptr<schedule::Client> client) {
  // TODO
  return true;
}

}  // namespace kernel::allocate