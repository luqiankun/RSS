#ifndef ACTION_HGPP
#define ACTION_HGPP
#include <regex>

#include "../../tcsresource.hpp"
namespace data {
namespace model {
class Actions {
 public:
  // vda操作
  struct Action {
    std::string id;
    std::string name;
    std::string block_type{"NONE"};
    std::map<std::string, std::string> params;
    std::string when;
    bool vaild{false};
    bool check_name{true};
  };
  Actions(std::map<std::string, std::string> pro) {
    std::regex N{R"(^vda5050:action.([^.]+)$)"};
    std::regex T{R"(^vda5050:action.([^.]+).blockingType$)"};
    std::regex P{R"(^vda5050:action.([^.]+).parameter.([^.]+)$)"};
    std::regex W{R"(^vda5050:action.([^.]+).when$)"};

    for (auto& x : pro) {
      std::smatch mt;
      if (std::regex_match(x.first, N)) {
        std::regex_search(x.first, mt, N);
        auto id = (mt.end() - 1)->str();
        auto name = x.second;
        bool has{false};
        for (auto& a : actions) {
          if (a.id == id) {
            a.name = x.second;
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.check_name = false;
          act.id = id;
          act.name = name;
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, T)) {
        std::regex_search(x.first, mt, T);
        auto id = (mt.end() - 1)->str();
        bool has{false};
        for (auto& a : actions) {
          if (a.id == id) {
            a.block_type = x.second;
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.check_name = false;
          act.id = id;
          act.block_type = x.second;
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, W)) {
        std::regex_search(x.first, mt, W);
        auto id = (mt.end() - 1)->str();
        bool has{false};
        for (auto& a : actions) {
          if (a.id == id) {
            a.when = x.second;
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.check_name = false;
          act.id = id;
          act.when = x.second;
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, P)) {
        std::regex_search(x.first, mt, P);
        auto p = std::pair<std::string, std::string>((mt.end() - 1)->str(),
                                                     x.second);
        auto id = (mt.end() - 2)->str();
        bool has{false};
        for (auto& a : actions) {
          if (a.id == id) {
            a.params.insert(p);
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.check_name = false;
          act.id = id;
          act.params.insert(p);
          actions.push_back(act);
        }
      }
    }
    for (auto& x : actions) {
      if (x.id.empty()) {
        x.vaild = false;
        continue;
      }
      if (x.name.empty()) {
        x.vaild = false;
        continue;
      }
      if (x.block_type.empty()) {
        x.vaild = false;
        continue;
      }
      if (x.when.empty()) {
        x.vaild = false;
        continue;
      }
      x.vaild = true;
    }
  }
  Actions() {}
  void append(Action act) { actions.push_back(act); }

 public:
  std::vector<Action> actions;
};
}  // namespace model
}  // namespace data

#endif