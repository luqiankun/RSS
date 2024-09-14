#ifndef ENVELOPE_HPP
#define ENVELOPE_HPP
#include <Eigen/Eigen>

#include "../../rssresource.hpp"
namespace data::model {
class Envelope : public RSSResource {
public:
  using RSSResource ::RSSResource;
  using Vertex = Eigen::Vector2f;
  explicit Envelope(const std::string &name) : RSSResource(name) {}
  inline void add_vertex(double x, double y) { vertexs.emplace_back(x, y); }
  [[nodiscard]] bool collide(const Envelope &other) const {
    auto it = std::any_of(vertexs.begin(), vertexs.end(),
                          [&](const Vertex &v) { return other.inside(v); });
    if (it) {
      return true;
    }
    auto if_it = std::any_of(other.vertexs.begin(), other.vertexs.end(),
                             [&](const Vertex &v) { return inside(v); });
    if (if_it) {
      return true;
    }
    return false;
  }
  [[nodiscard]] bool inside(const Vertex &p) const {
    bool flag = false;
    int n = vertexs.size() - 1;
    for (int i = 0; i < n; ++i) {
      auto p1 = vertexs[i];
      auto p2 = vertexs[i + 1];
      if (p.x() == p1.x() && p.y() == p1.y()) {
        return true;
      }
      if (p.x() == p2.x() && p.y() == p2.y()) {
        return true;
      }
      if (p.y() > std::min(p1.y(), p2.y())) {
        if (p.y() <= std::max(p1.y(), p2.y())) {
          if (p.x() <= std::max(p1.x(), p2.x())) {
            if (p1.y() != p2.y()) {
              // 交点 (px,p.y())
              auto px =
                  (p.y() - p1.y()) * (p1.x() - p2.x()) / (p1.y() - p2.y()) +
                  p1.x();
              if (px == p.x()) {
                return true;
              } else if (px < p.x()) {
                flag = !flag;
              }
            }
          }
        }
      }
    }
    return flag;
  }

public:
  std::vector<Vertex> vertexs;
};
} // namespace data::model
#endif