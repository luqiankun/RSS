#include "../../../include/kernel/planner/planner.hpp"

#include "../../../include/component/util/timer.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
namespace kernel {
namespace planner {
#if defined(_MSC_VER)
#undef max
#undef min
#endif
Planner::Planner(const std::shared_ptr<allocate::ResourceManager> &manager) {
  this->res = manager;
  rebuild();
}
Planner::~Planner() { CLOG(INFO, planner_log) << "Planner close"; }

void Planner::rebuild() {
  cpu_timer t("generate map");
  // std::cout << "begining generate map\n";
  solver = std::make_unique<Solver>();
  vertexs.clear();
  consoles.clear();
  edges.clear();
  for (auto &x : res.lock()->points) {
    auto vertex = std::make_shared<Vertex>(x);
    vertexs.push_back(vertex);
  }
  // std::cout << "generate " << vertexs.size() << " vertex\n";
  for (auto &x : res.lock()->paths) {
    bool ban{false};  // 路段是否禁止
    bool single{false};
    if (x->locked) {
      ban = true;
    }
    VertexPtr be{nullptr};
    VertexPtr en{nullptr};
    for (auto &v : vertexs) {
      if (v->equal_point == x->source_point.lock()) {
        be = v;
      }
      if (v->equal_point == x->destination_point.lock()) {
        en = v;
      }
    }
    if (be != nullptr && en != nullptr) {
      // TODO() 测试中权重写死了 应用时注意
      Edge::Access access{Edge::Access::Both};
      if (x->max_vel <= 0) {
        access = Edge::Access::Back;
      } else if (x->max_reverse_vel <= 0) {
        access = Edge::Access::Front;
      }
      auto edge = std::make_shared<Edge>(be, en, x->name, x->length, access);
      edges.push_back(edge);
      if (ban) {
        edge->open = false;
      } else {
        be->next_edge.push_back(edge);
        en->next_edge.push_back(edge);
      }
    }
  }
  // std::cout << "generate " << edges.size() << " edge\n";
  for (auto &x : res.lock()->locations) {
    auto console =
        std::make_shared<Console>(data::Vector2i(x->position.x, x->position.y),
                                  x->layout.position, x->name);
    VertexPtr link;
    for (auto &v : vertexs) {
      if (v->equal_point == x->link.lock()) {
        link = v;
      }
    }
    if (link) {
      console->set_link_node(link);
    }
    console->locked = x->locked;
    consoles.push_back(console);
  }
  // std::cout << "generate " << consoles.size() << " console\n";

  //
  // std::cout << "generate map ok\n\n";
}

void Planner::set_barrier_vertex(const std::string &n_s) {
  for (auto &x : vertexs) {
    if (x->name == n_s) {
      x->set_type(AType::ATYPE_BARRIER);
    }
  }
}

void Planner::reset_vertex(const std::string &n_s) {
  for (auto &x : vertexs) {
    if (x->name == n_s) {
      x->set_type(AType::ATYPE_UNKNOWN);
    }
  }
}

void Planner::set_path_direct(const std::string &e_s, Edge::Access access) {
  for (auto &x : edges) {
    if (x->name == e_s) {
      x->set_access(access);
    }
  }
}

void Planner::set_barrier_edge(const std::string &e) {
  for (auto &x : edges) {
    if (e == x->name) {
      x->open = false;
    }
  }
}
void Planner::reset_edge(const std::string &e) {
  for (auto &x : edges) {
    if (e == x->name) {
      x->open = true;
    }
  }
}
uint32_t Planner::calculate_turns(const std::vector<VertexPtr> &path,
                                  float th) {
  if (path.empty()) {
    return 0;
  }
  std::vector<data::Vector2f> edge_vec;
  for (auto it = path.begin(); it != path.end() - 1; it++) {
    auto from = *it;
    auto to = *(it + 1);
    edge_vec.emplace_back(data::Vector2f(to->location.x - from->location.x,
                                         to->location.y - from->location.y));
  }
  if (edge_vec.size() < 2) {
    return 0;
  }
  uint32_t con{0};
  for (auto it = edge_vec.begin(); it != edge_vec.end() - 1; it++) {
    float cos = (*it).dot(*(it + 1)) / ((*it).norm() * (*(it + 1)).norm());
    float angle = std::fabs(std::acos(cos));
    if (angle > th) {
      con++;
    }
  }
  return con;
}

std::vector<std::vector<std::shared_ptr<data::model::Point>>>
Planner::find_paths(const std::shared_ptr<data::model::Point> &begin,
                    const std::shared_ptr<data::model::Point> &end) {
  std::vector<std::vector<std::shared_ptr<data::model::Point>>> res;
  VertexPtr st{nullptr};
  VertexPtr ed{nullptr};
  for (auto &v : vertexs) {
    if (v->equal_point == begin) {
      st = v;
    }
    if (v->equal_point == end) {
      ed = v;
    }
  }
  if (st == nullptr || ed == nullptr) {
    CLOG(ERROR, planner_log) << "The start or end point does not exist\n";
    return res;
  }
  if (st == ed) {
    CLOG(WARNING, planner_log) << "The starting point and"
                                  "the ending point are the same\n ";
    res.emplace_back(
        std::vector<std::shared_ptr<data::model::Point>>{st->equal_point});
    return res;
  }
  {
    if (st == cur_begin) {
      CLOG(INFO, planner_log) << "already sovled\n";
    } else {
      solver->solver(st);
      cur_begin = st;
    }
    auto paths = solver->get_paths(ed);
    CLOG(INFO, planner_log) << "find " << paths.size() << " s paths\n";
    if (paths.empty()) {
    } else {
      // 按转弯数量排序
      std::sort(paths.begin(), paths.end(),
                [this](std::vector<VertexPtr> &a, std::vector<VertexPtr> &b) {
                  return calculate_turns(a) < calculate_turns(b);
                });
      for (auto &x : paths) {
        std::stringstream ss;
        ss << "\n.- - - - - - - - - - - - - - - - - - - - - - -.\n"
           << "| turns : " << calculate_turns(x) << " cost : " << x.back()->F
           << " vertex number : " << x.size() << "\n";
        std::vector<std::shared_ptr<data::model::Point>> temp;
        ss << "| path:[";
        temp.reserve(x.size());
        for (auto &p : x) {
          if (p == *(x.end() - 1)) {
            ss << p->name << "";
          } else {
            ss << p->name << " -> ";
          }
          temp.push_back(p->equal_point);
        }
        ss << "]\n";
        if (x.back()->F != std::numeric_limits<float>::max()) {
          res.push_back(std::move(temp));
        } else {
          ss << "this path is not open";
        }
        ss << "·- - - - - - - - - - - - - - - - - - - - - - -.\n";
        CLOG(INFO, planner_log) << ss.str();
      }
    }
  }
  // 复位顶点类型，已经设置为了障碍点则保持原样
  for (auto &x : vertexs) {
    if (x->type == AType::ATYPE_OPENED || x->type == AType::ATYPE_CLOSED) {
      x->set_type(AType::ATYPE_UNKNOWN);
    }
  }
  return res;
}

}  // namespace planner
}  // namespace kernel