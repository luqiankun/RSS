#ifndef ENVELOPE_HPP
#define ENVELOPE_HPP
#include <boost/geometry.hpp>

#include "../../rssresource.hpp"
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::polygon<point_t> polygon_t;
constexpr float ENVELOPE_STEP = 0.01;
namespace data::model {

class Envelope : public RSSResource {
 public:
  using RSSResource ::RSSResource;
  explicit Envelope(const std::string &name) : RSSResource(name) {}
  void set_points(double x, double y) {
    bg::append(poly.outer(), point_t(x, y));
  }
  [[nodiscard]] bool collide(const Envelope &other) const {
    if (bg::equals(poly, other.poly)) {
      return true;
    }
    if (bg::overlaps(poly, other.poly)) {
      return true;
    }
    if (bg::touches(poly, other.poly)) {
      return true;
    }
    return false;
  }

 public:
  polygon_t poly;
};
}  // namespace data::model
#endif