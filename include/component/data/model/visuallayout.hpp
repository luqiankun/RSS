#ifndef VISUALLAYOUT_HPP
#define VISUALLAYOUT_HPP
#include "../../tcsresource.hpp"
namespace data {
namespace model {
class VisualLayout : public TCSResource {
 public:
  using TCSResource::TCSResource;
  class Layer {
   public:
    int id;
    int ordinal;
    bool visible;
    std::string name;
    int group_id;
  };
  class LayerGroup {
   public:
    int id;
    std::string name;
    bool visible;
  };
  double scale_x;
  double scale_y;
  std::vector<Layer> layers;
  std::vector<LayerGroup> layer_groups;
};
}  // namespace model
}  // namespace data
#endif