#ifndef EDGE_HPP
#define EDGE_HPP
#include "../../component/data/model/path.hpp"
#include "./vertex.hpp"
namespace kernel {
namespace planner {
class Edge : public std::enable_shared_from_this<Edge> {
 public:
  enum class Access { Front = 0, Back = 1, Both = 2 };
  int weight;  // 权重
  VertexWeakPtr head;
  VertexWeakPtr tail;
  std::string name;
  bool open{true};  // 是否可通行
  Access access{Access::Front};

 public:
  Edge(const VertexPtr& h, const VertexPtr& t, const std::string& n, int w,
       Access);
  Edge() = delete;
  bool operator==(const EdgePtr& egde);
  void set_head(const VertexPtr& node);
  void set_tail(const VertexPtr& node);
  void set_weight(int w);
  void set_access(Access);
  void remove_links();
  void rebuild_links();
  std::string get_info() {
    std::stringstream os;
    os << "{\"name\": " << this->name
       << " , \"head\": " << this->head.lock()->get_info()
       << " , \"tail\": " << this->tail.lock()->get_info() << "}";
    return os.str();
  }
};
class Console {
 public:
  Console() = delete;
  Console(data::Vector2i pos, const std::string& n);
  Console(data::Vector2i pos, data::Vector2i layout, const std::string& n);
  ~Console() = default;
  void set_link_node(const VertexPtr& n) { link = n; }
  std::string get_info() {
    std::stringstream os;
    os << "{\"x\": " << this->location.x << " ,\"y\": " << this->location.y
       << " ,\"name\": " << this->name << "}";
    return os.str();
  }

 public:
  std::string name;
  data::Vector2i location;
  data::Vector2i layout;
  VertexPtr link{nullptr};
  bool locked{false};
};
using ConsolePtr = std::shared_ptr<Console>;
}  // namespace planner
}  // namespace kernel
#endif