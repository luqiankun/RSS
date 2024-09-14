#ifndef VISUALLAYOUT_HPP
#define VISUALLAYOUT_HPP
#include "../../rssresource.hpp"
namespace data::model {
class VisualLayout : public RSSObject {
public:
  using RSSObject::RSSObject;
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
} // namespace data::model
#endif