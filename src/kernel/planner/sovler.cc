#include "../../../include/kernel/planner/sovler.hpp"

#include <algorithm>
#include <iostream>
#include <stack>
#include <utility>

#include "../../../include/component/util/timer.hpp"
namespace kernel {
namespace planner {
#if defined(_MSC_VER)
// 防止包含window.h中的宏定义冲突
#undef max
#undef min
#endif
void Solver::solver(const VertexPtr& begin) {
  cpu_timer t("solver paths");
  this->begin_node = begin;
  if (begin_node == nullptr) {
    return;
  }
  {
    std::priority_queue<VertexPtr, std::vector<VertexPtr>, cmp> open_list;
    begin_node->type = AType::ATYPE_OPENED;
    begin_node->H = 0;
    begin_node->G = 0;
    begin_node->F = begin_node->G + begin_node->H;
    open_list.push(begin_node);
    do {
      // 获取最小值的节点
      auto cur_node = open_list.top();
      open_list.pop();
      cur_node->type = AType::ATYPE_CLOSED;
      // 获取相邻的点
      for (auto& x : cur_node->next_node) {
        if (x.lock()->type == AType::ATYPE_CLOSED ||
            x.lock()->type == AType::ATYPE_BARRIER) {
          continue;
        }
        if (x.lock()->type != AType::ATYPE_OPENED) {
          x.lock()->set_parent(cur_node);
          x.lock()->H = 0;
          x.lock()->G = x.lock()->get_g_value().has_value()
                            ? x.lock()->get_g_value().value()
                            : std::numeric_limits<float>::max();
          x.lock()->F = x.lock()->G + x.lock()->H;
          x.lock()->type = AType::ATYPE_OPENED;
          open_list.push(x.lock());
        } else {
          auto p = (*x.lock())[cur_node];
          float p2 =
              p.has_value() ? p.value() : std::numeric_limits<float>::max();
          if (x.lock()->G > (cur_node->G + p2)) {
            x.lock()->set_parent(cur_node);
            x.lock()->H = 0;
            x.lock()->G = x.lock()->get_g_value().has_value()
                              ? x.lock()->get_g_value().value()
                              : std::numeric_limits<float>::max();
            x.lock()->F = x.lock()->H + x.lock()->G;
          } else if (x.lock()->G == (cur_node->G + p2)) {
            x.lock()->add_parent(cur_node);
          }
        }
      }
    } while (!open_list.empty());
    CLOG(INFO, "planner") << "solver end\n";
  }
}
std::vector<std::vector<VertexPtr>> Solver::get_paths(const VertexPtr& end) {
  cpu_timer t("find paths");
  end_node = end;
  std::stack<VertexPtr> stack;
  stack.push(end_node);
  std::vector<VertexPtr> path;
  std::vector<std::vector<VertexPtr>> paths;
  // 前序便利
  while (!stack.empty()) {
    auto& x = stack.top();
    // std::cout << "cur " << x->name << "\n";
    path.emplace_back(x);
    stack.pop();
    if (x == begin_node) {
      // std::cout << "-----------------------\n";
      if (!paths.empty()) {
        auto temp_path = paths.back();
        if (!temp_path.empty()) {
          // path 首位
          auto first = path.front();
          // 找前置点
          std::vector<VertexPtr> temp;
          for (auto& p : temp_path) {
            bool check{false};
            temp.push_back(p);
            for (auto& v : p->parents) {
              if (first == v.lock()) {
                check = true;
                break;
              }
            }
            if (check) {
              // 招到了，插入后续
              temp.insert(temp.end(), path.begin(), path.end());
              break;
            }
          }
          paths.push_back(temp);
        }
      } else {
        paths.push_back(path);
      }
      path.clear();
      continue;
    }
    for (auto& t : x->parents) {
      stack.push(t.lock());
    }
  }
  // std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n";
  // for (auto& path : paths) {
  //   std::cout << "^^^^^^^^^^^^^^^^^^^^^^\n";
  //   for (auto& v : path) {
  //     std::cout << v->name << "\n";
  //   }
  //   std::cout << "~~~~~~~~~~~~~~~~~~~~~~~\n";
  // }
  // std::cout << "cost:[" << end_node->F << " ] vertex number:[<<"
  //           << paths.front().size() << "]\n";
  for (auto& x : paths) {
    std::reverse(x.begin(), x.end());
  }
  return paths;
}
}  // namespace planner
}  // namespace kernel