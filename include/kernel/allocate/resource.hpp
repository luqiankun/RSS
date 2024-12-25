#ifndef RESOURCE_HPP
#define RESOURCE_HPP
#include "../../../include/3rdparty/log/easylogging++.h"
#include "../../component/data/model/location.hpp"
#include "../../component/data/model/visuallayout.hpp"
#include "../../component/data/order/route.hpp"
#include "../rule/rule.hpp"
namespace data::model {
class Alleyway;
}
namespace kernel {
namespace schedule {
class Client;
}
namespace allocate {
using TCSResourcePtr = std::shared_ptr<RSSResource>;
using ClientPtr = std::shared_ptr<schedule::Client>;
using PointPtr = std::shared_ptr<data::model::Point>;
using LocationTypePtr = std::shared_ptr<data::model::LocationType>;
using PathPtr = std::shared_ptr<data::model::Path>;
using LocationPtr = std::shared_ptr<data::model::Location>;
class ResourceManager : public RSSObject {
 public:
  using RSSObject::RSSObject;
  enum class ResType { Point = 0, Location = 1, Err = 2, Path = 3 };
  ~ResourceManager() { CLOG(INFO, allocate_log) << name << " close\n"; }
  bool claim(const std::vector<TCSResourcePtr>&, const ClientPtr&);
  bool allocate(std::vector<TCSResourcePtr>, const ClientPtr&);
  bool unclaim(const std::vector<TCSResourcePtr>&, const ClientPtr&);
  bool free(const std::vector<TCSResourcePtr>&, const ClientPtr&);
  PointPtr get_recent_park_point(const PointPtr&) const;
  LocationPtr get_recent_charge_loc(const PointPtr&) const;
  std::shared_ptr<data::order::Route> paths_to_route(std::vector<PointPtr> ps);
  std::pair<ResType, std::shared_ptr<RSSResource>> find(
      const std::string& name);
  std::vector<std::shared_ptr<data::model::Point>> get_all_idel_points();
  std::pair<std::shared_ptr<data::model::Alleyway>, int> get_alleyway(PointPtr);

 public:
  std::mutex mut;
  std::string model_name;
  std::vector<PathPtr> paths;
  std::vector<PointPtr> points;
  std::vector<LocationPtr> locations;
  std::vector<LocationTypePtr> location_types;
  std::vector<std::shared_ptr<data::model::Alleyway>> alleyways;
  std::shared_ptr<data::model::VisualLayout> visual_layout;
  std::function<bool(PointPtr, PointPtr)> is_connected;

 public:
  std::vector<std::shared_ptr<RuleBase>> rules;
};
}  // namespace allocate
}  // namespace kernel

#endif