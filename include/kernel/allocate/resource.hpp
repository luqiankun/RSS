#ifndef RESOURCE_HPP
#define RESOURCE_HPP
#include "../../component/data/model/location.hpp"
#include "../../component/data/model/path.hpp"
#include "../../component/data/model/visuallayout.hpp"
#include "../../component/data/order/orderquence.hpp"
#include "../../component/util/timer.hpp"
#include "../rule/rule.hpp"
namespace kernel {
namespace schedule {
class Client;
}
namespace allocate {
using TCSResourcePtr = std::shared_ptr<TCSResource>;
using ClientPtr = std::shared_ptr<schedule::Client>;
using PointPtr = std::shared_ptr<data::model::Point>;
using LocationTypePtr = std::shared_ptr<data::model::LocationType>;
using PathPtr = std::shared_ptr<data::model::Path>;
using LocationPtr = std::shared_ptr<data::model::Location>;
class ResourceManager : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class ResType { Point = 0, Location = 1, Err = 2, Path = 3 };
  ~ResourceManager() { CLOG(INFO, allocate_log) << name << " close\n"; }
  bool claim(const std::vector<TCSResourcePtr>&, const ClientPtr&);
  bool allocate(std::vector<TCSResourcePtr>, ClientPtr);
  bool unclaim(const std::vector<TCSResourcePtr>&, const ClientPtr&);
  bool free(const std::vector<TCSResourcePtr>&, const ClientPtr&);
  PointPtr get_recent_park_point(PointPtr);
  LocationPtr get_recent_charge_loc(PointPtr);
  std::shared_ptr<data::order::Route> paths_to_route(std::vector<PointPtr> ps);
  std::pair<ResType, std::shared_ptr<TCSResource>> find(
      const std::string& name);

 public:
  std::mutex mut;
  std::string model_name;
  std::vector<PathPtr> paths;
  std::vector<PointPtr> points;
  std::vector<LocationPtr> locations;
  std::vector<LocationTypePtr> location_types;
  std::shared_ptr<data::model::VisualLayout> visual_layout;
  std::function<bool(PointPtr, PointPtr)> is_connected;

 public:
  std::vector<std::shared_ptr<RuleBase>> rules;
};
}  // namespace allocate
}  // namespace kernel

#endif