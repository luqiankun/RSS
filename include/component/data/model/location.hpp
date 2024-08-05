#ifndef LOCATION_HPP
#define LOCATION_HPP
#include "../../rssresource.hpp"
#include "./locationtype.hpp"
#include "point.hpp"
namespace data {
namespace model {

class Location : public RSSResource {
 public:
  struct Layout {
    Vector2i position{0, 0};
    Vector2i label_offset{0, 0};
    int layer_id;
  };

  using RSSResource::RSSResource;
  Vector3i position{0, 0, 0};
  Layout layout;
  bool locked{false};
  std::weak_ptr<Point> link;
  std::weak_ptr<LocationType> type;
};
}  // namespace model
}  // namespace data
#endif
