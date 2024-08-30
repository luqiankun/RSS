#ifndef ASTAR_HPP
#define ASTAR_HPP
#include <vector>

#include "vertex.hpp"
namespace kernel {

namespace planner {

class cmp {
 public:
  // 小顶堆，越小优先级越高
  bool operator()(const VertexPtr& a, const VertexPtr& b) {
    return a->F > b->F;
  }
};
class Solver : public std::enable_shared_from_this<Solver> {
 public:
  void solver(const VertexPtr& begin);
  std::vector<std::vector<VertexPtr>> get_paths(const VertexPtr& end);

 private:
  VertexPtr end_node{nullptr};
  VertexPtr begin_node{nullptr};
};
using SolverPtr = std::shared_ptr<Solver>;
}  // namespace planner
}  // namespace kernel
#endif