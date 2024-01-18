#ifndef EDGE_HPP
#define EDGE_HPP
#pragma once
#include "../../component/data/model/path.hpp"
#include "./vertex.hpp"
namespace kernel {
namespace planner {
class Edge : public std::enable_shared_from_this<Edge> {
 public:
  int weight;  // 权重
  VertexWeakPtr head;
  VertexWeakPtr tail;
  std::string name;
  bool single{false};  // 单向
  bool open{true};     // 是否可通行
 public:
  Edge(const VertexPtr& h, const VertexPtr& t, const std::string& n, int w,
       bool s);
  Edge() = delete;
  bool operator==(const EdgePtr& egde);
  void set_head(const VertexPtr& node);
  void set_tail(const VertexPtr& node);
  void set_weight(int w);
  std::string get_info() {
    std::stringstream os;
    os << "{\"name\": " << this->name
       << " , \"head\": " << this->head.lock()->get_info()
       << " , \"tail\": " << this->tail.lock()->get_info()
       << " signle: " << this->single << "}";
    return os.str();
  }
};
class Console {
 public:
  Console() = delete;
  Console(Eigen::Vector2i pos, const std::string& n);
  Console(Eigen::Vector2i pos, Eigen::Vector2i layout, const std::string& n);
  ~Console() = default;
  void set_link_node(const VertexPtr& n) { link = n; }
  std::string get_info() {
    std::stringstream os;
    os << "{\"x\": " << this->location.x() << " ,\"y\": " << this->location.y()
       << " ,\"name\": " << this->name << "}";
    return os.str();
  }

 public:
  std::string name;
  Eigen::Vector2i location;
  Eigen::Vector2i layout;
  VertexPtr link{nullptr};
};
using ConsolePtr = std::shared_ptr<Console>;
}  // namespace planner
}  // namespace kernel
#endif