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
  for (auto& x : res.lock()->paths) {
    if ((x->source_point.lock()->position.x >= this->x) &&
        (x->source_point.lock()->position.y >= this->y) &&
        (x->source_point.lock()->position.x <= this->x + width) &&
        (x->source_point.lock()->position.y <= this->y + height) &&
        (x->destination_point.lock()->position.x >= this->x) &&
        (x->destination_point.lock()->position.y >= this->y) &&
        (x->destination_point.lock()->position.x <= this->x + width) &&
        (x->destination_point.lock()->position.y <= this->y + height)) {
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

bool CollisionRule::pass(std::vector<std::shared_ptr<TCSResource>> resource,
                         std::shared_ptr<schedule::Client> client) {
  fcl::ContinuousCollisionRequestd req;
  fcl::ContinuousCollisionResultd rep;
  auto t1 = fcl::Transform3d::Identity();
  auto t2 = fcl::Transform3d::Identity();
  fcl::AngleAxisd rotate_vec(
      std::dynamic_pointer_cast<driver::Vehicle>(client)->angle,
      fcl::Vector3<double>(0, 0, 1));
  t1.linear() = rotate_vec.matrix();
  t2.linear() = rotate_vec.matrix();
  for (int i = 0; i < resource.size(); ++i) {
    auto tcs_res = resource.at(i);
    for (auto& x : res.lock()->points) {
      if (x->name == tcs_res->name) {
        t1.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                         x->position.y * 1.0 / 1000,
                                         x->position.z * 1.0 / 1000);
        t2.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                         x->position.y * 1.0 / 1000,
                                         x->position.z * 1.0 / 1000);
        // 在该点是否碰撞
        for (auto& e : x->envelopes) {
          if (e.second.lock() == client->envelope) {
            continue;
          }
          auto ret = fcl::continuousCollide(
              std::dynamic_pointer_cast<data::model::Envelope>(e.second.lock())
                  ->convex.get(),
              t1, t2, client->envelope->convex.get(), t1, t2, req, rep);
          if (rep.is_collide) {
            return false;
          }
        }
        // 是否和关联的操作点碰撞
        for (auto& loc : x->attached_links) {
          if (loc->envelopes.empty()) continue;
          t1.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                           x->position.y * 1.0 / 1000,
                                           x->position.z * 1.0 / 1000);
          t2.translation() = fcl::Vector3d(loc->position.x * 1.0 / 1000,
                                           loc->position.y * 1.0 / 1000,
                                           loc->position.z * 1.0 / 1000);
          fcl::Vector3d dist = t1.translation() - t2.translation();
          for (auto& e : loc->envelopes) {
            if (e.second.lock() == client->envelope) {
              continue;
            }
            auto ret = fcl::continuousCollide(
                std::dynamic_pointer_cast<data::model::Envelope>(
                    e.second.lock())
                    ->convex.get(),
                t2, t2, client->envelope->convex.get(), t1, t1, req, rep);
            if (rep.is_collide) {
              // LOG(WARNING) << "■■■■■■■■■■■■■";
              return false;
            }
          }
        }
        // 是否和相邻的点碰撞
        for (auto& path : x->incoming_paths) {
          auto p = path->destination_point.lock();
          t1.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                           x->position.y * 1.0 / 1000,
                                           x->position.z * 1.0 / 1000);
          t2.translation() = fcl::Vector3d(p->position.x * 1.0 / 1000,
                                           p->position.y * 1.0 / 1000,
                                           p->position.z * 1.0 / 1000);
          // 在该点是否碰撞
          for (auto& e : p->envelopes) {
            if (e.second.lock() == client->envelope) {
              continue;
            }
            auto ret = fcl::continuousCollide(
                std::dynamic_pointer_cast<data::model::Envelope>(
                    e.second.lock())
                    ->convex.get(),
                t2, t2, client->envelope->convex.get(), t1, t1, req, rep);
            if (rep.is_collide) {
              // LOG(WARNING) << "■■■■■■■■■■■■■";
              return false;
            }
          }
        }
        for (auto& path : x->outgoing_paths) {
          auto p = path->source_point.lock();
          t1.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                           x->position.y * 1.0 / 1000,
                                           x->position.z * 1.0 / 1000);
          t2.translation() = fcl::Vector3d(p->position.x * 1.0 / 1000,
                                           p->position.y * 1.0 / 1000,
                                           p->position.z * 1.0 / 1000);
          // 在该点是否碰撞
          for (auto& e : p->envelopes) {
            if (e.second.lock() == client->envelope) {
              continue;
            }
            auto ret = fcl::continuousCollide(
                std::dynamic_pointer_cast<data::model::Envelope>(
                    e.second.lock())
                    ->convex.get(),
                t2, t2, client->envelope->convex.get(), t1, t1, req, rep);
            if (rep.is_collide) {
              // LOG(WARNING) << "■■■■■■■■■■■■■";
              return false;
            }
          }
        }

        break;
      }
    }
    for (auto& x : res.lock()->paths) {
      if (x->name == tcs_res->name) {
        // 相向行驶是否碰撞
        t1.translation() =
            fcl::Vector3d(x->source_point.lock()->position.x * 1.0 / 1000,
                          x->source_point.lock()->position.y * 1.0 / 1000,
                          x->source_point.lock()->position.z * 1.0 / 1000);
        t2.translation() =
            fcl::Vector3d(x->destination_point.lock()->position.x * 1.0 / 1000,
                          x->destination_point.lock()->position.y * 1.0 / 1000,
                          x->destination_point.lock()->position.z * 1.0 / 1000);
        for (auto& e : x->envelopes) {
          if (e.second.lock() == client->envelope) {
            continue;
          }
          auto ret = fcl::continuousCollide(
              std::dynamic_pointer_cast<data::model::Envelope>(e.second.lock())
                  ->convex.get(),
              t1, t2, client->envelope->convex.get(), t2, t1, req, rep);
          if (rep.is_collide) {
            return false;
          }
        }
        break;
      }
    }
    // 在操作点是否碰撞
    for (auto& x : res.lock()->locations) {
      if (x->name == tcs_res->name) {
        t1.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                         x->position.y * 1.0 / 1000,
                                         x->position.z * 1.0 / 1000);
        t2.translation() = fcl::Vector3d(x->position.x * 1.0 / 1000,
                                         x->position.y * 1.0 / 1000,
                                         x->position.z * 1.0 / 1000);
        for (auto& e : x->envelopes) {
          if (e.second.lock() == client->envelope) {
            continue;
          }
          auto ret = fcl::continuousCollide(
              std::dynamic_pointer_cast<data::model::Envelope>(e.second.lock())
                  ->convex.get(),
              t1, t2, client->envelope->convex.get(), t1, t2, req, rep);
          if (rep.is_collide) {
            return false;
          }
        }
        break;
      }
    }
  }
  return true;
}

}  // namespace allocate
}  // namespace kernel