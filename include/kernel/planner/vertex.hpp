#ifndef VERTEX_HPP
#define VERTEX_HPP
#include "../../component/data/model/point.hpp"
namespace kernel {

namespace planner {

class Edge;
class Vertex;
using VertexPtr = std::shared_ptr<Vertex>;
using EdgePtr = std::shared_ptr<Edge>;
using VertexWeakPtr = std::weak_ptr<Vertex>;
enum class AType {
  ATYPE_UNKNOWN,
  ATYPE_CLOSED,
  ATYPE_OPENED,
  ATYPE_BARRIER  // 障碍
};
class Vertex : public ::std::enable_shared_from_this<Vertex> {
 public:
  Eigen::Vector2i location;
  Eigen::Vector2i layout;
  AType type{AType::ATYPE_UNKNOWN};
  std::vector<VertexWeakPtr> next_node;
  std::vector<EdgePtr> next_edge;
  std::shared_ptr<data::model::Point> equal_point;
  float F{0};
  float G{0};
  float H{0};
  std::vector<VertexWeakPtr> parents;
  std::string name;

 public:
  Vertex() = delete;
  explicit Vertex(const std::shared_ptr<data::model::Point> &);
  bool operator==(Vertex node);
  std::optional<float> operator[](const VertexPtr &node);
  float get_h_vlaue(const VertexPtr &node);
  std::optional<float> get_g_value();
  void set_type(AType type);
  ~Vertex() = default;
  ;
  std::string get_info() {
    std::stringstream os;
    os << "{\"x\": " << this->location.x() << " ,\"y\": " << this->location.y()
       << " ,\"name\": " << this->name << "}";
    return os.str();
  }
  void set_parent(const VertexPtr &n) {
    parents.clear();
    parents.push_back(n);
  }
  void add_parent(const VertexPtr &n) { this->parents.push_back(n); }
};
}  // namespace planner
}  // namespace kernel
#endif