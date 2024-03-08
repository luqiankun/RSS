#ifndef VDA5050ORDER_HPP
#define VDA5050ORDER_HPP
#include <optional>
#include <variant>

#include "../tools/json/json.hpp"
#include "../tools/mqtt/mqtt.hpp"
namespace vda5050 {
namespace order {
enum class ActionType {
  NOP,     // 啥也不干
  LOAD,    // 去某location load
  UNLOAD,  // 去某location unload
  MOVE     // 停在某point,
};
enum class ActionBlockingType { NONE = 1, SOFT = 2, HARD = 3 };

// action

struct ActionParam {
  std::string key;
  std::variant<std::string, bool, float, std::vector<std::string>> value;
};

struct Action {
  ActionType action_type{ActionType::NOP};
  std::string action_id;
  std::optional<std::string> action_description;
  ActionBlockingType blocking_type{ActionBlockingType::HARD};
  std::optional<std::vector<ActionParam>> action_parameters;
  Action() {}
  Action(nlohmann::json& obj) {
    if (obj["actionType"].get<std::string>() == "LOAD") {
      action_type = ActionType::LOAD;
    } else if (obj["actionType"].get<std::string>() == "UNLOAD") {
      action_type = ActionType::UNLOAD;
    } else if (obj["actionType"].get<std::string>() == "MOVE") {
      action_type = ActionType::MOVE;
    } else {
      action_type = ActionType::NOP;
    }
    action_id = obj["actionId"].get<std::string>();
    if (obj.contains("actionDescription")) {
      action_description = obj["actionDescription"].get<std::string>();
    }
    if (obj["blockingType"].get<std::string>() == "NONE") {
      blocking_type = ActionBlockingType::NONE;
    } else if (obj["blockingType"].get<std::string>() == "SOFT") {
      blocking_type = ActionBlockingType::SOFT;
    } else {
      blocking_type = ActionBlockingType::HARD;
    }
    if (obj.contains("actionParameters")) {
      action_parameters = std::vector<ActionParam>();
      for (auto& x : obj["actionParameters"]) {
        ActionParam param;
        param.key = x["key"].get<std::string>();
        if (x["value"].is_boolean()) {
          param.value = x["value"].get<bool>();
        } else if (x["value"].is_string()) {
          param.value = x["value"].get<std::string>();
        } else if (x["value"].is_number()) {
          param.value = x["value"].get<float>();
        } else if (x["value"].is_array()) {
          param.value = std::vector<std::string>();
          for (auto& str : x["value"]) {
            std::get<std::vector<std::string>>(param.value)
                .push_back(str.get<std::string>());
          }
        }
        action_parameters->push_back(param);
      }
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["actionId"] = action_id;
    if (action_type == ActionType::LOAD) {
      res["actionType"] = "LOAD";
    } else if (action_type == ActionType::UNLOAD) {
      res["actionType"] = "UNLOAD";
    } else if (action_type == ActionType::MOVE) {
      res["actionType"] = "MOVE";
    } else {
      res["actionType"] = "NOP";
    }
    if (blocking_type == ActionBlockingType::HARD) {
      res["blockingType"] = "HARD";
    } else if (blocking_type == ActionBlockingType::NONE) {
      res["blockingType"] = "NONE";
    } else {
      res["blockingType"] = "SOFT";
    }
    if (action_description.has_value()) {
      res["actionDescription"] = action_description.value();
    }
    if (action_parameters.has_value()) {
      res["actionParameters"] = nlohmann::json::array();
      for (auto& x : action_parameters.value()) {
        nlohmann::json p;
        p["key"] = x.key;
        if (x.value.index() == 0) {
          p["value"] = std::get<0>(x.value);
        } else if (x.value.index() == 1) {
          p["value"] = std::get<1>(x.value);
        } else if (x.value.index() == 2) {
          p["value"] = std::get<2>(x.value);
        } else {
          p["value"] = nlohmann::json::array();
          for (auto& str : std::get<3>(x.value)) {
            p["value"].push_back(str);
          }
        }
        res["actionParameters"].push_back(p);
      }
    }
    return res;
  }
};

// edge

struct ControlPoint {
  float x;
  float y;
  std::optional<float> weight;
};

struct Trajectory {
  int degree;
  std::vector<float> knot_vector;
  std::vector<ControlPoint> control_points;
};
struct Edge {
  std::string edge_id;
  int sequence_id;
  bool released;
  std::string start_node_id;
  std::string end_node_id;
  std::vector<Action> actions;
  std::optional<std::string> edge_description;
  std::optional<float> max_speed;
  std::optional<float> max_height;
  std::optional<float> min_height;
  std::optional<float> orientation;
  std::optional<std::string> orientation_type;
  std::optional<std::string> direction;
  std::optional<bool> rotation_allowed;
  std::optional<float> max_rotation_speed;
  std::optional<float> length;
  std::optional<Trajectory> trajectory;
  Edge() {}
  Edge(nlohmann::json& obj) {
    edge_id = obj["edgeId"].get<std::string>();
    sequence_id = obj["sequenceId"].get<int>();
    released = obj["released"].get<bool>();
    start_node_id = obj["startNodeId"].get<std::string>();
    end_node_id = obj["endNodeId"].get<std::string>();
    for (auto& x : obj["actions"]) {
      Action act(x);
      actions.push_back(act);
    }
    //
    if (obj.contains("maxSpeed")) {
      max_speed = obj["maxSpeed"].get<float>();
    }
    if (obj.contains("maxHeight")) {
      max_height = obj["maxHeight"].get<float>();
    }
    if (obj.contains("minHeight")) {
      min_height = obj["minHeight"].get<float>();
    }
    if (obj.contains("orientation")) {
      orientation = obj["orientation"].get<float>();
    }
    if (obj.contains("orientationType")) {
      orientation_type = obj["orientationType"].get<std::string>();
    }
    if (obj.contains("direction")) {
      direction = obj["direction"].get<std::string>();
    }
    if (obj.contains("rotationAllowed")) {
      rotation_allowed = obj["rotationAllowed"].get<bool>();
    }
    if (obj.contains("maxRotationSpeed")) {
      max_rotation_speed = obj["maxRotationSpeed"].get<float>();
    }
    if (obj.contains("length")) {
      length = obj["length"].get<float>();
    }
    if (obj.contains("trajectory")) {
      trajectory = Trajectory();
      trajectory->degree = obj["trajectory"]["degree"].get<int>();
      for (auto& x : obj["trajectory"]["knotVector"]) {
        trajectory->knot_vector.push_back(x.get<float>());
      }
      for (auto& x : obj["trajectory"]["controlPoints"]) {
        auto p = ControlPoint();
        p.x = x["x"].get<float>();
        p.y = x["y"].get<float>();
        if (x.contains("weight")) {
          p.weight = x["weight"].get<float>();
        }
        trajectory->control_points.push_back(p);
      }
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["edgeId"] = edge_id;
    res["sequenceId"] = sequence_id;
    res["released"] = released;
    res["startNodeId"] = start_node_id;
    res["endNodeId"] = end_node_id;
    res["actions"] = nlohmann::json::array();
    for (auto& x : actions) {
      res["actions"].push_back(x.to_json());
    }
    if (edge_description.has_value()) {
      res["edgeDescription"] = edge_description.value();
    }
    if (max_speed.has_value()) {
      res["maxSpeed"] = max_speed.value();
    }
    if (max_height.has_value()) {
      res["maxHeight"] = max_height.value();
    }
    if (min_height.has_value()) {
      res["minHeight"] = min_height.value();
    }
    if (orientation.has_value()) {
      res["orientation"] = orientation.value();
    }
    if (orientation_type.has_value()) {
      res["orientationType"] = orientation_type.value();
    }
    if (direction.has_value()) {
      res["direction"] = direction.value();
    }
    if (rotation_allowed.has_value()) {
      res["rotationAllowed"] = rotation_allowed.value();
    }
    if (max_rotation_speed.has_value()) {
      res["maxRotationSpeed"] = max_rotation_speed.value();
    }
    if (length.has_value()) {
      res["length"] = length.value();
    }
    if (trajectory.has_value()) {
      res["trajectory"]["degree"] = trajectory.value().degree;
      res["knotVector"] = nlohmann::json::array();
      res["controlPoints"] = nlohmann::json::array();
      for (auto& x : trajectory.value().knot_vector) {
        res["knotVector"].push_back(x);
      }
      for (auto& x : trajectory.value().control_points) {
        nlohmann::json p;
        p["x"] = x.x;
        p["y"] = x.y;
        if (x.weight.has_value()) {
          p["weight"] = x.weight.value();
        }
        res["controlPoints"].push_back(p);
      }
    }
    return res;
  }
};

// node

struct NodePosition {
  float x;
  float y;
  std::string map_id;
  std::optional<float> theta;
  std::optional<float> allowed_deviation_xy;
  std::optional<float> allowed_deviation_theta;
  std::optional<std::string> map_description;
};

struct Node {
  std::string node_id;
  int sequence_id;
  bool released;
  std::vector<Action> actions;
  std::optional<std::string> node_description;
  std::optional<NodePosition> node_position;
  Node() {}
  Node(nlohmann::json& obj) {
    node_id = obj["nodeId"].get<std::string>();
    sequence_id = obj["sequenceId"].get<int>();
    released = obj["released"].get<bool>();
    for (auto& x : obj["actions"]) {
      Action act(x);
      actions.push_back(act);
    }
    if (obj.contains("nodeDescription")) {
      node_description = obj["nodeDescription"].get<std::string>();
    }
    if (obj.contains("nodePosition")) {
      node_position = NodePosition();
      node_position.value().x = obj["nodePosition"]["x"].get<float>();
      node_position.value().y = obj["nodePosition"]["y"].get<float>();
      node_position.value().map_id =
          obj["nodePosition"]["mapId"].get<std::string>();
      if (obj["nodePosition"].contains("theta")) {
        node_position.value().theta = obj["nodePosition"]["theta"].get<float>();
      }
      if (obj["nodePosition"].contains("allowedDeviationXy")) {
        node_position.value().allowed_deviation_xy =
            obj["nodePosition"]["allowedDeviationXy"].get<float>();
      }
      if (obj["nodePosition"].contains("allowedDeviationTheta")) {
        node_position.value().allowed_deviation_theta =
            obj["nodePosition"]["allowedDeviationTheta"].get<float>();
      }
      if (obj["nodePosition"].contains("mapDescription")) {
        node_position.value().map_description =
            obj["nodePosition"]["mapDescription"].get<std::string>();
      }
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["nodeId"] = node_id;
    res["sequenceId"] = sequence_id;
    res["released"] = released;
    res["actions"] = nlohmann::json::array();
    for (auto& x : actions) {
      res["actions"].push_back(x.to_json());
    }
    if (node_description.has_value()) {
      res["nodeDescription"] = node_description.value();
    }
    if (node_position.has_value()) {
      res["nodePosition"]["x"] = node_position.value().x;
      res["nodePosition"]["y"] = node_position.value().y;
      res["nodePosition"]["mapId"] = node_position.value().map_id;
      if (node_position.value().map_description.has_value()) {
        res["nodePosition"]["mapDescription"] =
            node_position.value().map_description.value();
      }
      if (node_position.value().theta.has_value()) {
        res["nodePosition"]["theta"] = node_position.value().theta.value();
      }
      if (node_position.value().allowed_deviation_xy.has_value()) {
        res["nodePosition"]["allowedDeviationXy"] =
            node_position.value().allowed_deviation_xy.value();
      }
      if (node_position.value().allowed_deviation_theta.has_value()) {
        res["nodePosition"]["allowedDeviationTheta"] =
            node_position.value().allowed_deviation_theta.value();
      }
    }
    return res;
  }
};

// order

struct VDA5050Order {
  int header_id;
  std::string timestamp;
  std::string version;
  std::string manufacturer;
  std::string serial_number;
  std::string order_id;
  int order_update_id;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  std::optional<std::string> zoneset_id;
  VDA5050Order() {}
  VDA5050Order(nlohmann::json& obj) {
    header_id = obj["headerId"].get<int>();
    timestamp = obj["timestamp"].get<std::string>();
    version = obj["version"].get<std::string>();
    manufacturer = obj["manufacturer"].get<std::string>();
    serial_number = obj["serialNumber"].get<std::string>();
    order_id = obj["orderId"].get<std::string>();
    order_update_id = obj["orderUpdateId"].get<int>();
    if (obj.contains("zoneSetId")) {
      zoneset_id = obj["zoneSetId"].get<std::string>();
    }
    for (auto& x : obj["nodes"]) {
      auto n = Node(x);
      nodes.push_back(n);
    }
    std::stable_sort(nodes.begin(), nodes.end(),
                     [](const Node& a, const Node& b) {
                       return a.sequence_id < b.sequence_id;
                     });
    for (auto& x : obj["edges"]) {
      auto n = Edge(x);
      edges.push_back(n);
    }
    std::stable_sort(edges.begin(), edges.end(),
                     [](const Edge& a, const Edge& b) {
                       return a.sequence_id < b.sequence_id;
                     });
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["headerId"] = header_id;
    res["timestamp"] = timestamp;
    res["version"] = version;
    res["manufacturer"] = manufacturer;
    res["serialNumber"] = serial_number;
    res["orderId"] = order_id;
    res["orderUpdateId"] = order_update_id;
    res["nodes"] = nlohmann::json::array();
    res["edges"] = nlohmann::json::array();
    for (auto& x : nodes) {
      res["nodes"].push_back(x.to_json());
    }
    for (auto& x : edges) {
      res["edges"].push_back(x.to_json());
    }
    if (zoneset_id.has_value()) {
      res["zoneSetId"] = zoneset_id.value();
    }
    return res;
  }
};

}  // namespace order
}  // namespace vda5050

#endif