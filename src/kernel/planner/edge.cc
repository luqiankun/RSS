#include <utility>

#include "../../../include/kernel/planner/edge.hpp"
namespace kernel::planner {

Edge::Edge(const VertexPtr &h, const VertexPtr &t, std::string n, int w,
           const Access access)
    : weight(w), head(h), tail(t), name(std::move(n)), access(access) {
  rebuild_links();
}

void Edge::set_access(Access access) {
  this->access = access;
  remove_links();
  rebuild_links();
}
void Edge::remove_links() const {
  auto head_ptr = head.lock();
  auto tail_ptr = tail.lock();
  if (head_ptr && tail_ptr) {
    // 移除链接
    for (auto x = head_ptr->next_node.begin();
         x != head_ptr->next_node.end();) {
      if (x->lock() == tail_ptr) {
        head_ptr->next_node.erase(x++);
      } else {
        ++x;
      }
    }
    for (auto x = tail_ptr->next_node.begin();
         x != tail_ptr->next_node.end();) {
      if (x->lock() == head_ptr) {
        tail_ptr->next_node.erase(x++);
      } else {
        ++x;
      }
    }
  }
}
void Edge::rebuild_links() const {
  // 重新建立链接
  if (access == Access::Front) {
    // 头是否需要插入邻近点
    bool has_n = false;
    for (auto &x : head.lock()->next_node) {
      if (auto n = x.lock()) {
        if (n == tail.lock()) {
          has_n = true;
          break;
        }
      }
    }
    if (!has_n) {
      head.lock()->next_node.push_back(tail);
    }
  } else if (access == Access::Back) {
    // 尾是否需要插入邻近点
    bool has_n_{false};
    for (auto &x : tail.lock()->next_node) {
      if (auto n = x.lock()) {
        if (n == head.lock()) {
          has_n_ = true;
          break;
        }
      }
    }
    if (!has_n_) {
      tail.lock()->next_node.push_back(head);
    }
  } else {
    {
      // 头是否需要插入邻近点
      bool has_n = false;
      for (auto &x : head.lock()->next_node) {
        if (auto n = x.lock()) {
          if (n == tail.lock()) {
            has_n = true;
            break;
          }
        }
      }
      if (!has_n) {
        head.lock()->next_node.push_back(tail);
      }
    }
    {
      // 尾是否需要插入邻近点
      bool has_n_{false};
      for (auto &x : tail.lock()->next_node) {
        if (auto n = x.lock()) {
          if (n == head.lock()) {
            has_n_ = true;
            break;
          }
        }
      }
      if (!has_n_) {
        tail.lock()->next_node.push_back(head);
      }
    }
  }
}
void Edge::set_tail(const VertexPtr &vertex) {
  remove_links();
  tail = vertex;
  rebuild_links();
}
void Edge::set_head(const VertexPtr &vertex) {
  remove_links();
  //
  head = vertex;
  // 重新建立链接
  rebuild_links();
}
void Edge::set_weight(int w) { this->weight = w; }
bool Edge::operator==(const EdgePtr &edge) const {
  // 判断两个边是否为同一个或者等价
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

Console::Console(const Eigen::Vector2i &pos, std::string n)
    : name(std::move(n)), location(pos), layout(pos) {}
Console::Console(Eigen::Vector2i pos, Eigen::Vector2i layout,
                 std::string n)
    : name(std::move(n)), location(std::move(pos)), layout(std::move(layout)) {}

}