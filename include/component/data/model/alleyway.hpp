#ifndef ALLEYWAY_HPP
#define ALLEYWAY_HPP
#include <algorithm>

#include "path.hpp"
namespace data::model {

class Alleyway : public RSSResource {
 public:
  Alleyway(const std::string& name, std::deque<std::shared_ptr<Path>> alley_,
           std::deque<std::shared_ptr<Point>> vertices_)
      : RSSResource(name) {
    this->alley = alley_;
    this->vertices = vertices_;
    common_point = vertices.back();
    vertices.pop_back();
    std::reverse(vertices.begin(), vertices.end());
    std::reverse(alley.begin(), alley.end());
  }
  int size() const { return vertices.size(); }
  std::pair<bool, int> is_in(std::shared_ptr<Point> p) {
    int i = 0;
    for (auto& v : vertices) {
      if (v == p) {
        return {true, i};
      }
      i++;
    }
    return {false, i};
  }

 public:
  std::shared_ptr<Point> common_point;
  std::deque<std::shared_ptr<Path>> alley;
  std::deque<std::shared_ptr<Point>> vertices;
};
}  // namespace data::model
#endif