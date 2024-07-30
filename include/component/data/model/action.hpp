#ifndef ACTION_HGPP
#define ACTION_HGPP
#include <regex>
#include <variant>

#include "../../tcsresource.hpp"
#include "../../tools/jsoncons/json.hpp"
// #include "../order/driverorder.hpp"
namespace data {
namespace model {
class Actions {
 public:
  enum class OpType {
    NOP,     // 啥也不干
    LOAD,    // 去某location load
    UNLOAD,  // 去某location unload
    MOVE,    // 去某point
    CHARGE,  // 充电
    CLOSE,
    OPEN,
    LIFT,
    DROP,
    PICK,
    startPause,
    stopPause
  };
  enum class ActionBlockingType { NONE = 1, SOFT = 2, HARD = 3 };
  class ActionParam {
   public:
    std::string key;
    std::variant<std::string, bool, double, std::vector<std::string>> value;
  };
  // 自定义
  enum class ActionWhen { ORDER_START = 1, ORDER_END = 2 };
  static std::optional<OpType> get_optype(const std::string& op) {
    if (op == "NOP") {
      return OpType::NOP;
    } else if (op == "pick") {
      return OpType::PICK;
    } else if (op == "LOAD") {
      return OpType::LOAD;
    } else if (op == "UNLOAD") {
      return OpType::UNLOAD;
    } else if (op == "drop") {
      return OpType::DROP;
    } else if (op == "CLOSE") {
      return OpType::CLOSE;
    } else if (op == "OPEN") {
      return OpType::OPEN;
    } else if (op == "LIFT") {
      return OpType::LIFT;
    } else if (op == "Charge") {
      return OpType::CHARGE;
    } else if (op == "MOVE") {
      return OpType::MOVE;
    } else if (op == "startPause") {
      return OpType::startPause;
    } else if (op == "stopPause") {
      return OpType::stopPause;
    } else {
      return std::nullopt;
    }
  }
  static std::string get_type(OpType t) {
    if (t == OpType::MOVE) {
      return "MOVE";
    } else if (t == OpType::PICK) {
      return "pick";
    } else if (t == OpType::DROP) {
      return "drop";
    } else if (t == OpType::LOAD) {
      return "LOAD";
    } else if (t == OpType::UNLOAD) {
      return "UNLOAD";
    } else if (t == OpType::CHARGE) {
      return "Charge";
    } else if (t == OpType::OPEN) {
      return "OPEN";
    } else if (t == OpType::CLOSE) {
      return "CLOSE";
    } else if (t == OpType::LIFT) {
      return "LIFT";
    } else if (t == OpType::startPause) {
      return "startPause";
    } else if (t == OpType::stopPause) {
      return "stopPause";
    } else {
      return "NOP";
    }
  }
  // vda操作
  class Action {
   public:
    Action() = default;
    OpType action_type{OpType::NOP};
    std::string action_id;
    std::optional<std::string> action_description;
    ActionBlockingType blocking_type{ActionBlockingType::HARD};
    std::optional<std::vector<ActionParam>> action_parameters;
    ActionWhen when;
    bool vaild{false};
    std::string name;
    virtual void init(jsoncons::json&){};
    virtual jsoncons::json to_json() { return jsoncons::json::object(); };
  };
  explicit Actions(std::map<std::string, std::string> pro) {
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
          if (a.action_id == id) {
            a.name = name;
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.name = name;
          act.action_id = id;
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, T)) {
        std::regex_search(x.first, mt, T);
        auto id = (mt.end() - 1)->str();
        bool has{false};
        for (auto& a : actions) {
          if (a.action_id == id) {
            if (x.second == "NONE") {
              a.blocking_type = ActionBlockingType::NONE;
            } else if (x.second == "SOFT") {
              a.blocking_type = ActionBlockingType::SOFT;
            } else {
              a.blocking_type = ActionBlockingType::HARD;
            }
          }
          has = true;
        }
        if (!has) {
          Action act;
          act.action_id = id;
          if (x.second == "NONE") {
            act.blocking_type = ActionBlockingType::NONE;
          } else if (x.second == "SOFT") {
            act.blocking_type = ActionBlockingType::SOFT;
          } else {
            act.blocking_type = ActionBlockingType::HARD;
          }
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, W)) {
        std::regex_search(x.first, mt, W);
        auto id = (mt.end() - 1)->str();
        bool has{false};
        for (auto& a : actions) {
          if (a.action_id == id) {
            if (x.second == "ORDER_START") {
              a.when = ActionWhen::ORDER_START;
            } else {
              a.when = ActionWhen::ORDER_END;
            }
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.action_id = id;
          if (x.second == "ORDER_START") {
            act.when = ActionWhen::ORDER_START;
          } else {
            act.when = ActionWhen::ORDER_END;
          }
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, P)) {
        std::regex_search(x.first, mt, P);
        auto p1 = std::pair<std::string, std::string>((mt.end() - 1)->str(),
                                                      x.second);
        auto p = ActionParam();
        p.key = (mt.end() - 1)->str();
        p.value = x.second;
        auto id = (mt.end() - 2)->str();
        bool has{false};
        for (auto& a : actions) {
          if (a.action_id == id) {
            if (!a.action_parameters.has_value()) {
              a.action_parameters = std::vector<ActionParam>();
            }
            a.action_parameters->push_back(p);
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.action_id = id;
          act.action_parameters->push_back(p);
          actions.push_back(act);
        }
      }
    }
    for (auto& x : actions) {
      if (x.action_id.empty()) {
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
class PeripheralActions {
 public:
  class PeripheralAction {
   public:
    std::string op_name;
    std::string location_name;
    bool completion_required{false};
    std::string execution_trigger;
  };
  std::vector<PeripheralAction> acts;
};
}  // namespace model
}  // namespace data

#endif