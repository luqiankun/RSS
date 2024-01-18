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
  ~ResourceManager() { LOG(INFO) << name << " close"; }
  bool allocate(const std::vector<std::shared_ptr<TCSResource>>&,
                const std::shared_ptr<schedule::Client>&);
  bool claim(const std::vector<std::shared_ptr<TCSResource>>&,
             const std::shared_ptr<schedule::Client>&);
  bool free(const std::vector<std::shared_ptr<TCSResource>>&,
            const std::shared_ptr<schedule::Client>&);
  bool unclaim(const std::vector<std::shared_ptr<TCSResource>>&,
               const std::shared_ptr<schedule::Client>&);

 public:
  std::mutex mut;
  std::vector<std::shared_ptr<data::model::Path>> paths;
  std::vector<std::shared_ptr<data::model::Point>> points;
  std::vector<std::shared_ptr<data::model::Location>> locations;
};
}  // namespace allocate
}  // namespace kernel

#endif