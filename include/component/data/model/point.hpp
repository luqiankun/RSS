#ifndef POINT_HPP
#define POINT_HPP
#include "../../rssresource.hpp"
#include "./envelope.hpp"
namespace data {
namespace model {
using namespace Eigen;
class Path;
class Location;
class Point : public RSSResource {
 public:
  using RSSResource::RSSResource;
  struct Layout {
    Vector2i position{0, 0};
    Vector2i label_offset{0, 0};
    int layer_id{0};
  };
  enum class Type {
    REPORT_POSITION,
    HALT_POSITION,
    PARK_POSITION,
    NORMAL_POSITION,
    UNKNOWN
  };
  static std::string get_type(Type t) {
    if (t == Type::HALT_POSITION) {
      return "HALT_POSITION";

    } else if (t == Type::NORMAL_POSITION) {
      return "NORMAL_POSITION";

    } else if (t == Type::PARK_POSITION) {
      return "PARK_POSITION";

    } else if (t == Type::REPORT_POSITION) {
      return "REPORT_POSITION";
    } else {
      return "UNKNOWN";
    }
  }
  static Type new_type(const std::string& name) {
    if (name == "HALT_POSITION") {
      return Type::HALT_POSITION;
    } else if (name == "NORMAL_POSITION") {
      return Type::NORMAL_POSITION;

    } else if (name == "PARK_POSITION") {
      return Type::PARK_POSITION;

    } else if (name == "REPORT_POSITION") {
      return Type::REPORT_POSITION;
    } else {
      return Type::UNKNOWN;
    }
  }

 public:
  Vector3i position{0, 0, 0};  // x y
  Layout layout;
  Type type{Type::UNKNOWN};
  double vehicle_orientation{std::nan("")};
  std::vector<std::shared_ptr<Path>> incoming_paths;
  std::vector<std::shared_ptr<Path>> outgoing_paths;
  std::vector<std::shared_ptr<Location>> attached_links;
};
}  // namespace model
}  // namespace data
#endif