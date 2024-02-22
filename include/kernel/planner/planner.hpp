#ifndef Planner_HPP
#define Planner_HPP
#include "../../component/tcsobject.hpp"
#include "sovler.hpp"
#define PI 3.1415926
namespace kernel {
namespace allocate {
class ResourceManager;
}
namespace planner {
class Planner {
 public:
  Planner() = delete;
  explicit Planner(const std::shared_ptr<allocate::ResourceManager> &);
  // 设置障碍点
  void set_barrier_vertex(const std::string &n_s);
  // 恢复点的初始状态
  void reset_vertex(const std::string &n_s);
  // 设置路径方向
  void set_path_direct(const std::string &e_s, Edge::Access access);
  // 设置边不可通行
  void set_barrier_edge(const std::string &e_s);
  // 恢复边可通行
  void reset_edge(const std::string &e_s);
  // 计算转弯数,大于角度阈值视为转弯,默认45°
  uint32_t calculate_turns(const std::vector<VertexPtr> &path,
                           float th = PI / 4);
  // 查找最短路径 返回vertex对象路径
  std::vector<std::vector<std::shared_ptr<data::model::Point>>> find_paths(
      const std::shared_ptr<data::model::Point> &,
      const std::shared_ptr<data::model::Point> &);

 public:
  std::vector<VertexPtr> vertexs;    // 顶点
  std::vector<EdgePtr> edges;        // 边
  std::vector<ConsolePtr> consoles;  // 工作台
 private:
  VertexPtr cur_begin{nullptr};
  std::unique_ptr<Solver> solver;
};
}  // namespace planner
}  // namespace kernel

#endif