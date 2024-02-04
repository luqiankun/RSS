#ifndef RESOURCE_HPP
#define RESOURCE_HPP
#include "../../component/data/model/location.hpp"
#include "../../component/data/model/path.hpp"
#include "../../component/data/order/orderquence.hpp"
#include "../../component/util/timer.hpp"
namespace kernel {
namespace schedule {
class Client;
}
namespace allocate {
class ResourceManager : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class ResType { Point = 0, Location = 1, Err = 2 };
  ~ResourceManager() { CLOG(INFO, "allocate") << name << " close"; }
  bool allocate(const std::vector<std::shared_ptr<TCSResource>>&,
                const std::shared_ptr<schedule::Client>&);
  bool claim(std::vector<std::shared_ptr<TCSResource>>,
             std::shared_ptr<schedule::Client>);
  bool free(const std::vector<std::shared_ptr<TCSResource>>&,
            const std::shared_ptr<schedule::Client>&);
  bool unclaim(const std::vector<std::shared_ptr<TCSResource>>&,
               const std::shared_ptr<schedule::Client>&);
  std::shared_ptr<data::order::Route> paths_to_route(
      std::vector<std::shared_ptr<data::model::Point>> ps);
  std::pair<ResType, std::shared_ptr<TCSResource>> find(
      const std::string& name);

 public:
  std::mutex mut;
  std::string model_name;
  std::vector<std::shared_ptr<data::model::Path>> paths;
  std::vector<std::shared_ptr<data::model::Point>> points;
  std::vector<std::shared_ptr<data::model::Location>> locations;
};
}  // namespace allocate
}  // namespace kernel

#endif