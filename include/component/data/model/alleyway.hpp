#ifndef ALLEYWAY_HPP
#define ALLEYWAY_HPP
#include "path.hpp"
namespace data::model {

class Alleyway : public RSSResource {
 public:
  Alleyway(const std::string& name) : RSSResource(name) {}
  std::deque<std::shared_ptr<RSSObject>> get_owners() {
    std::deque<std::shared_ptr<RSSObject>> ret;
    for (auto& x : vertices) {
      auto own = x->owner.lock();
      if (own) {
        ret.push_back(own);
      }
    }
    return ret;
  }

 public:
  std::deque<std::shared_ptr<Path>> alley;
  std::deque<std::shared_ptr<Point>> vertices;
};
}  // namespace data::model
#endif