#include "../../../include/kernel/rule/rule.hpp"

#include "../../../include/component/data/model/envelope.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
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
  if (owners.size() > 0) {
    return false;
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
void OnlyOneDirectRule::init() {
  std::map<std::shared_ptr<RSSResource>, int> t;
  std::set<std::shared_ptr<RSSResource>> paths;
  for (auto &x : occs) {
    auto path = std::dynamic_pointer_cast<data::model::Path>(x);
    if (path) {
      t[path->source_point.lock()] += 1;
      t[path->destination_point.lock()] += 1;
      paths.insert(path);
    }
  }
  std::shared_ptr<data::model::Point> beg;
  for (auto &x : t) {
    if (x.second == 1) {
      beg = std::dynamic_pointer_cast<data::model::Point>(x.first);
      break;
    }
  }
  assert(beg);
  std::vector<std::shared_ptr<data::model::Point>> path;
  path.push_back(beg);
  while (!paths.empty()) {
    for (auto it = paths.begin(); it != paths.end();) {
      auto temp = std::dynamic_pointer_cast<data::model::Path>(*it);
      if (temp->source_point.lock() == path.back()) {
        // forward
        path.push_back(temp->destination_point.lock());
        table[temp] = 1;
        it = paths.erase(it);
      } else if (temp->destination_point.lock() == path.back()) {
        // backward
        path.push_back(temp->source_point.lock());
        table[temp] = 2;
        it = paths.erase(it);
      } else {
        ++it;
      }
    }
  }
}

bool OnlyOneDirectRule::pass(std::vector<std::shared_ptr<RSSResource>> res,
                             std::shared_ptr<schedule::Client> client) {
  bool ext = false;
  for (auto &x : res) {
    if (occs.find(x) != occs.end()) {
      ext = true;
      break;
    }
  }
  if (!ext) {
    return true;
  }

  owners.clear();
  for (auto &x : occs) {
    if (x->owner.lock() && x->owner.lock() != client) {
      auto veh = std::dynamic_pointer_cast<driver::Vehicle>(x->owner.lock());
      if (veh->state == driver::Vehicle::State::EXECUTING)
        owners.insert(x->owner.lock());
    }
  }
  // 当前dir
  int dir = 0;  // 0 无 1 同向 2 反向
  for (auto &x : owners) {
    auto veh = std::dynamic_pointer_cast<driver::Vehicle>(x);
    if (!veh->last_step.empty()) {
      for (auto &step : veh->last_step) {
        if (table.find(step->path) != table.end()) {
          if (step->vehicle_orientation ==
              data::order::Step::Orientation::FORWARD) {
            if (table[step->path] == 1) {
              dir = 1;
            } else {
              dir = 2;
            }
          } else {
            if (table[step->path] == 1) {
              dir = 2;
            } else {
              dir = 1;
            }
          }
        }
      }
    }
  }
  if (dir == 0) {
    return true;
  }
  int cur_dir = 0;
  auto veh = std::dynamic_pointer_cast<driver::Vehicle>(client);
  if (veh->future_step.empty()) {
    return true;
  } else {
    for (auto &x : veh->future_step) {
      if (table.find(x->path) != table.end()) {
        if (x->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
          if (table[x->path] == 1) {
            cur_dir = 1;
          } else {
            cur_dir = 2;
          }
        } else {
          if (table[x->path] == 1) {
            cur_dir = 2;
          } else {
            cur_dir = 1;
          }
        }
      }
    }
  }
  return dir == cur_dir;
}
}  // namespace kernel::allocate