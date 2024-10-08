#ifndef EDGE_HPP
#define EDGE_HPP
#include "./vertex.hpp"
namespace kernel::planner {
class Edge : public std::enable_shared_from_this<Edge> {
 public:
  enum class Access { Front = 0, Back = 1, Both = 2 };
  int weight;  // 权重
  VertexWeakPtr head{};
  VertexWeakPtr tail{};
  std::string name{};
  bool open{true};  // 是否可通行
  Access access{Access::Front};
  std::shared_ptr<data::model::Path> equal_path{nullptr};

 public:
  Edge(const VertexPtr &h, const VertexPtr &t, std::string n, int w, Access);
  Edge() = delete;
  bool operator==(const EdgePtr &edge) const;
  void set_head(const VertexPtr &node);
  void set_tail(const VertexPtr &node);
  void set_weight(int w);
  void set_access(Access);
  void remove_links() const;
  void rebuild_links() const;
  std::string get_info() const {
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
  Console(const Eigen::Vector2i &pos, std::string n);
  Console(Eigen::Vector2i pos, Eigen::Vector2i layout, std::string n);
  ~Console() = default;
  void set_link_node(const VertexPtr &n) { link = n; }
  std::string get_info() {
    std::stringstream os;
    os << "{\"x\": " << this->location.x() << " ,\"y\": " << this->location.y()
       << " ,\"name\": " << this->name << "}";
    return os.str();
  }

 public:
  std::string name{};
  Eigen::Vector2i location{};
  Eigen::Vector2i layout{};
  VertexPtr link{nullptr};
  bool locked{false};
};
using ConsolePtr = std::shared_ptr<Console>;
}  // namespace kernel::planner
#endif