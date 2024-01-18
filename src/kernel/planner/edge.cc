#include "../../../include/kernel/planner/edge.hpp"
namespace kernel {
namespace planner {

Edge::Edge(const VertexPtr &h, const VertexPtr &t, const std::string &n, int w,
           bool s) {
  head = h;
  tail = t;
  weight = w;
  this->name = n;
  this->single = s;
  bool has_n{false};
  // 头是否需要插入邻近点
  for (auto &x : h->next_node) {
    auto n = x.lock();
    if (n) {
      if (n == t) {
        has_n = true;
        break;
      }
    }
  }
  if (!has_n) {
    h->next_node.push_back(t);
  }
  if (!single) {  // 双向
    // 尾是否需要插入邻近点
    bool has_n_{false};
    for (auto &x : t->next_node) {
      auto n = x.lock();
      if (n) {
        if (n == h) {
          has_n_ = true;
          break;
        }
      }
    }
    if (!has_n_) {
      t->next_node.push_back(h);
    }
  }
}

void Edge::set_tail(const VertexPtr &vertex) {
  if (single) {
    // 单向
    auto head_ptr = head.lock();
    auto tail_ptr = tail.lock();
    // 从头节点的邻近点中先移除尾节点
    if (head_ptr && tail_ptr) {
      for (auto x = head_ptr->next_node.begin();
           x != head_ptr->next_node.end();) {
        if (x->lock() == tail_ptr) {
          head_ptr->next_node.erase(x++);
        } else {
          ++x;
        }
      }
      // 指向新尾
      tail = vertex;
      // 头邻近点钟添加新尾
      bool has_n{false};
      for (auto &x : head_ptr->next_node) {
        auto n = x.lock();
        if (n) {
          if (n == vertex) {
            has_n = true;
            break;
          }
        }
      }
      if (!has_n) {
        head_ptr->next_node.push_back(vertex);
      }
    }
  } else {
    // 双向
    auto head_ptr = head.lock();
    auto tail_ptr = tail.lock();
    if (head_ptr && tail_ptr) {
      for (auto x = tail_ptr->next_node.begin();
           x != tail_ptr->next_node.end();) {
        if (x->lock() == head_ptr) {
          tail_ptr->next_node.erase(x++);
        } else {
          ++x;
        }
      }
      tail = vertex;
      bool has_n{false};
      for (auto &x : vertex->next_node) {
        auto n = x.lock();
        if (n) {
          if (n == head_ptr) {
            has_n = true;
            break;
          }
        }
      }
      if (!has_n) {
        vertex->next_node.push_back(head_ptr);
      }
    }
  }
}
void Edge::set_head(const VertexPtr &vertex) {
  auto head_ptr = head.lock();
  auto tail_ptr = tail.lock();
  if (head_ptr && tail_ptr) {
    for (auto x = head_ptr->next_node.begin();
         x != head_ptr->next_node.end();) {
      if (x->lock() == tail_ptr) {
        head_ptr->next_node.erase(x++);
      } else {
        ++x;
      }
    }
    head = vertex;
    bool has_n{false};
    for (auto &x : vertex->next_node) {
      auto n = x.lock();
      if (n) {
        if (n == tail_ptr) {
          has_n = true;
          break;
        }
      }
    }
    if (!has_n) {
      vertex->next_node.push_back(tail_ptr);
    }
  }
};
void Edge::set_weight(int w) { this->weight = w; }
bool Edge::operator==(const EdgePtr &edge) {
  // TODO(unknown): 判断两个边是否为同一个或者等价
  if (edge->name == this->name) {
    if (edge->weight == this->weight) {
      auto head_ptr = head.lock();
      auto tail_ptr = tail.lock();
      auto e_head_ptr = edge->head.lock();
      auto e_tail_ptr = edge->tail.lock();
      if (head_ptr && tail_ptr && e_head_ptr && e_tail_ptr) {
        if (e_head_ptr == head_ptr) {
          if (e_tail_ptr == tail_ptr) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

Console::Console(Eigen::Vector2i pos, const std::string &n) {
  location = pos;
  layout = pos;
  name = n;
}
Console::Console(Eigen::Vector2i pos, Eigen::Vector2i layout,
                 const std::string &n) {
  location = pos;
  layout = layout;
  name = n;
}

}  // namespace planner
}  // namespace kernel