#include "../../../include/kernel/planner/vertex.hpp"

#include "../../../include/kernel/planner/edge.hpp"

namespace kernel {
namespace planner {

Vertex::Vertex(const std::shared_ptr<data::model::Point> &p) {
  equal_point = p;
  location = Eigen::Vector2i(p->position.x(), p->position.y());
  this->name = p->name;
  layout = p->layout.position;
}

void Vertex::set_type(AType type) { this->type = type; }

bool Vertex::operator==(const Vertex &vertex) {
  if (vertex.location != this->location) {
    return false;
  }
  if (vertex.layout != this->layout) {
    return false;
  }
  if (vertex.type != this->type) {
    return false;
  }
  if (vertex.name != this->name) {
    return false;
  }
  if (vertex.F != this->F) {
    return false;
  }
  if (vertex.G != this->G) {
    return false;
  }
  if (vertex.H != this->H) {
    return false;
  }
  return true;
}

std::optional<float> Vertex::operator[](const VertexPtr &vertex) {
  if (vertex == this->shared_from_this()) {
    return std::optional(0);
  }
  for (auto &x : next_edge) {
    for (auto &y : vertex->next_edge) {
      if (x == y) {
        if (x->open) {
          return std::optional(x->weight);
        } else {
          return std::nullopt;
        }
      }
    }
  }
  return std::nullopt;
}
float Vertex::get_h_vlaue(const VertexPtr &vertex) {
  // todo H代价算法 为0则等同迪杰斯特拉算法。广度优先
  Eigen::Vector2f dis(this->location.x() - vertex->location.x(),
                      this->location.y() - vertex->location.y());
  // return 1 * (abs(dis.x()) + abs(dis.y()));
  // return dis.norm();
  return 0;
}

std::optional<float> Vertex::get_g_value() {
  // 获取从起点到该点代价，为0则深度优先
  //  return 0;
  if (parents.empty()) {
    return std::nullopt;
  }
  auto parent = this->parents.begin()->lock();
  if (parent) {
    std::optional<float> x = (*this)[parent];
    if (x.has_value()) {
      return std::optional(parent->G + x.value());
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}
}  // namespace planner
}  // namespace kernel