#ifndef VDA5050STATE_HPP
#define VDA5050STATE_HPP
#include <optional>

#include "../tools/json/json.hpp"
#include "../tools/mqtt/mqtt.hpp"
namespace vda5050 {
enum class VehicleMqttStatus { ONLINE = 1, OFFLINE = 2, CONNECTIONBROKEN = 3 };
enum class MasterMqttStatus { ONLINE = 1, OFFLINE = 2, CONNECTIONBROKEN = 3 };
namespace state {
enum class ActionStatus {
  WAITING = 1,
  INITIALIZING = 2,
  RUNNING = 3,
  FINISHED = 4,
  FAILED = 5
};
enum class ErrorLevel { WARNING = 1, FATAL = 2 };
enum class SafetyStatus { AUTOACK = 1, MANUAL = 2, REMOTE = 3, NONE = 4 };
enum class OperMode {
  AUTOMATIC = 1,
  SEMIAUTOMATIC = 2,
  MANUAL = 3,
  SERVICE = 4,
  TEACHIN = 5
};
enum InfoLevel { INFO = 1, DEBUG = 2 };

// data struct

// safety
struct SafetyState {
  SafetyStatus estop{SafetyStatus::NONE};
  bool field_violation{false};
  SafetyState() {
    estop = SafetyStatus::NONE;
    field_violation = false;
  }
  SafetyState(nlohmann::json& obj) {
    auto es = obj["eStop"].get<std::string>();
    if (es == "AUTOACK") {
      estop = SafetyStatus::AUTOACK;
    } else if (es == "MANUAL") {
      estop = SafetyStatus::MANUAL;
    } else if (es == "REMOTE") {
      estop = SafetyStatus::REMOTE;
    } else {
      estop = SafetyStatus::NONE;
    }
    auto f = obj["fieldViolation"].get<bool>();
    field_violation = f;
  }
  SafetyState(SafetyStatus s, bool f) {
    estop = s;
    field_violation = f;
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["fieldViolation"] = field_violation;
    if (estop == SafetyStatus::AUTOACK) {
      res["eStop"] = "AUTOACK";
    } else if (estop == SafetyStatus::MANUAL) {
      res["eStop"] = "MANUAL";

    } else if (estop == SafetyStatus::REMOTE) {
      res["eStop"] = "REMOTE";
    } else {
      res["eStop"] = "NONE";
    }
    return std::move(res);
  }
};

// information
struct InforRef {
  std::string reference_key;
  std::string reference_value;
};

struct InforMation {
  std::string info_type;
  InfoLevel info_level{InfoLevel::INFO};
  std::optional<std::string> info_description{std::nullopt};
  std::optional<std::vector<InforRef>> info_references{std::nullopt};
  InforMation(){};
  InforMation(nlohmann::json& obj) {
    info_type = obj["infoType"].get<std::string>();
    auto level = obj["infoLevel"].get<std::string>();
    if (level == "INFO") {
      info_level = InfoLevel::INFO;
    } else {
      info_level = InfoLevel::DEBUG;
    }
    if (obj.contains("infoReferences")) {
      info_references = std::vector<InforRef>();
      for (auto& x : obj["infoReferences"]) {
        auto t = InforRef();
        t.reference_key = x["referenceKey"].get<std::string>();
        t.reference_key = x["referenceValue"].get<std::string>();
        info_references->push_back(t);
      }
    }
    if (obj.contains("infoDescription")) {
      info_description = obj["infoDescription"].get<std::string>();
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["infoType"] = info_type;
    if (info_level == InfoLevel::INFO) {
      res["infoLevel"] = "INFO";
    } else {
      res["infoLevel"] = "DEBUG";
    }
    if (info_description.has_value()) {
      res["infoDescription"] = info_description.value();
    }
    if (info_references.has_value()) {
      res["infoReferences"] = nlohmann::json::array();
      for (auto& x : info_references.value()) {
        nlohmann::json ref;
        ref["referenceKey"] = x.reference_key;
        ref["referenceValue"] = x.reference_value;
        res["infoReferences"].push_back(ref);
      }
    }
    return res;
  }
};

// errors

struct ErrorRef {
  std::string reference_key;
  std::string reference_value;
};
struct Error {
  ErrorLevel error_level;
  std::string error_type;
  std::optional<std::string> error_description{std::nullopt};
  std::optional<std::vector<ErrorRef>> error_references{std::nullopt};
  Error() {}
  Error(nlohmann::json& obj) {
    error_type = obj["errorType"].get<std::string>();
    auto level = obj["errorLevel"].get<std::string>();
    if (level == "WARNING") {
      error_level = ErrorLevel::WARNING;
    } else {
      error_level = ErrorLevel::FATAL;
    }
    if (obj.contains("errorDescription")) {
      error_description = obj["errorDescription"].get<std::string>();
    }
    if (obj.contains("errorReferences")) {
      error_references = std::vector<ErrorRef>();
      for (auto& x : obj["errorReferences"]) {
        ErrorRef ref;
        ref.reference_key = x["referenceKey"].get<std::string>();
        ref.reference_value = x["referenceValue"].get<std::string>();
        error_references->push_back(ref);
      }
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["errorType"] = error_type;
    if (error_level == ErrorLevel::WARNING) {
      res["errorLevel"] = "WARNING";
    } else {
      res["errorLevel"] = "FATAL";
    }
    if (error_description.has_value()) {
      res["errorDescription"] = error_description.value();
    }
    if (error_references.has_value()) {
      res["errorReferences"] = nlohmann::json::array();
      for (auto& x : error_references.value()) {
        nlohmann::json ref;
        ref["referenceKey"] = x.reference_key;
        ref["referenceValue"] = x.reference_value;
        res["errorReferences"].push_back(ref);
      }
    }
    return res;
  }
};

// battery

struct BatteryState {
  float battery_charge;
  bool charging;
  std::optional<float> battery_voltage{std::nullopt};
  std::optional<float> battery_health{std::nullopt};
  std::optional<float> reach{std::nullopt};
  BatteryState() {}
  BatteryState(nlohmann::json& obj) {
    battery_charge = obj["batteryCharge"].get<float>();
    charging = obj["charging"].get<bool>();
    if (obj.contains("batteryHealth")) {
      battery_health = obj["batteryHealth"].get<float>();
    }
    if (obj.contains("batteryVoltage")) {
      battery_voltage = obj["batteryVoltage"].get<float>();
    }
    if (obj.contains("reach")) {
      reach = obj["reach"].get<float>();
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["batteryCharge"] = battery_charge;
    res["charging"] = charging;
    if (battery_voltage.has_value()) {
      res["batteryVoltage"] = battery_voltage.value();
    }
    if (battery_health.has_value()) {
      res["batteryHealth"] = battery_health.value();
    }
    if (reach.has_value()) {
      res["reach"] = reach.value();
    }
    return res;
  }
};

// actionstate

struct ActionState {
  std::string action_id;
  std::optional<std::string> action_type{std::nullopt};
  std::optional<std::string> action_description{std::nullopt};
  std::optional<std::string> result_description{std::nullopt};
  ActionStatus action_status;
  ActionState(){};
  ActionState(nlohmann::json& obj) {
    action_id = obj["actionId"].get<std::string>();
    auto status = obj["actionStatus"].get<std::string>();
    if (status == "WAITING") {
      action_status = ActionStatus::WAITING;
    } else if (status == "INITIALIZING") {
      action_status = ActionStatus::INITIALIZING;
    } else if (status == "RUNNING") {
      action_status = ActionStatus::RUNNING;
    } else if (status == "FINISHED") {
      action_status = ActionStatus::FINISHED;
    } else {
      action_status = ActionStatus::FAILED;
    }
    if (obj.contains("actionType")) {
      action_type = obj["actionType"].get<std::string>();
    }
    if (obj.contains("actionDescription")) {
      action_description = obj["actionDescription"].get<std::string>();
    }
    if (obj.contains("resultDescription")) {
      result_description = obj["resultDescription"].get<std::string>();
    }
  };
  nlohmann::json to_json() {
    nlohmann::json res;
    res["actionId"] = action_id;
    if (action_status == ActionStatus::WAITING) {
      res["actionStatus"] = "WAITING";
    } else if (action_status == ActionStatus::INITIALIZING) {
      res["actionStatus"] = "INITIALIZING";
    } else if (action_status == ActionStatus::RUNNING) {
      res["actionStatus"] = "RUNNING";
    } else if (action_status == ActionStatus::FINISHED) {
      res["actionStatus"] = "FINISHED";
    } else {
      res["actionStatus"] = "FAILED";
    }
    if (action_type.has_value()) {
      res["actionType"] = action_type.value();
    }
    if (action_description.has_value()) {
      res["actionDescription"] = action_description.value();
    }
    if (result_description.has_value()) {
      res["resultDescription"] = result_description.value();
    }
    return res;
  }
};

// loads
struct BoundingBox {
  float x;
  float y;
  float z;
  std::optional<float> theta;
};
struct LoadDimensions {
  float length;
  float width;
  std::optional<float> height;
};

struct Load {
  std::optional<std::string> load_id{std::nullopt};
  std::optional<std::string> load_type{std::nullopt};
  std::optional<std::string> load_position{std::nullopt};
  std::optional<float> weight{std::nullopt};
  std::optional<BoundingBox> bounding_boxReference{std::nullopt};
  std::optional<LoadDimensions> Load_dimensions{std::nullopt};
  Load(){};
  Load(nlohmann::json& obj) {
    if (obj.contains("loadId")) {
      load_id = obj["loadId"].get<std::string>();
    }
    if (obj.contains("loadType")) {
      load_type = obj["loadType"].get<std::string>();
    }
    if (obj.contains("loadPosition")) {
      load_position = obj["loadPosition"].get<std::string>();
    }
    if (obj.contains("weight")) {
      weight = obj["weight"].get<float>();
    }
    if (obj.contains("boundingBoxReference")) {
      BoundingBox box;
      box.x = obj["boundingBoxReference"]["x"].get<float>();
      box.y = obj["boundingBoxReference"]["y"].get<float>();
      box.z = obj["boundingBoxReference"]["z"].get<float>();
      if (obj["boundingBoxReference"].contains("theta")) {
        box.theta = obj["boundingBoxReference"]["theta"].get<float>();
      }
      bounding_boxReference = box;
    }
    if (obj.contains("loadDimensions")) {
      LoadDimensions box;
      box.width = obj["loadDimensions"]["width"].get<float>();
      box.length = obj["loadDimensions"]["length"].get<float>();
      if (obj["loadDimensions"].contains("height")) {
        box.height = obj["loadDimensions"]["height"].get<float>();
      }
      Load_dimensions = box;
    }
  };
  nlohmann::json to_json() {
    nlohmann::json res;
    if (load_id.has_value()) {
      res["loadId"] = load_id.value();
    }
    if (load_type.has_value()) {
      res["loadType"] = load_type.value();
    }
    if (load_position.has_value()) {
      res["loadPosition"] = load_position.value();
    }
    if (weight.has_value()) {
      res["weight"] = weight.value();
    }
    if (bounding_boxReference.has_value()) {
      res["boundingBoxReference"]["x"] = bounding_boxReference.value().x;
      res["boundingBoxReference"]["y"] = bounding_boxReference.value().y;
      res["boundingBoxReference"]["z"] = bounding_boxReference.value().z;
      if (bounding_boxReference.value().theta.has_value()) {
        res["boundingBoxReference"]["theta"] =
            bounding_boxReference.value().theta.value();
      }
    }
    if (Load_dimensions.has_value()) {
      res["loadDimensions"]["length"] = Load_dimensions.value().length;
      res["loadDimensions"]["width"] = Load_dimensions.value().width;
      if (Load_dimensions->height.has_value()) {
        res["loadDimensions"]["height"] =
            Load_dimensions.value().height.value();
      }
    }
    return res;
  }
};

// velocity

struct Velocity {
  std::optional<float> vx{std::nullopt};
  std::optional<float> vy{std::nullopt};
  std::optional<float> omega{std::nullopt};
  Velocity(){};
  Velocity(nlohmann::json& obj) {
    if (obj.contains("vx")) {
      vx = obj["vx"].get<float>();
    }
    if (obj.contains("vy")) {
      vy = obj["vy"].get<float>();
    }
    if (obj.contains("omega")) {
      omega = obj["omega"].get<float>();
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    if (vx.has_value()) {
      res["vx"] = vx.value();
    }
    if (vy.has_value()) {
      res["vy"] = vy.value();
    }
    if (omega.has_value()) {
      res["omega"] = omega.value();
    }
    return res;
  }
};

// agvpositon

struct AgvPosition {
  float x;
  float y;
  float theta;
  std::string map_id;
  bool position_initialized;
  std::optional<std::string> map_description;
  std::optional<float> deviation_range;
  std::optional<float> localization_score;
  AgvPosition(){};
  AgvPosition(nlohmann::json obj) {
    x = obj["x"].get<float>();
    y = obj["y"].get<float>();
    theta = obj["theta"].get<float>();
    map_id = obj["mapId"].get<std::string>();
    position_initialized = obj["positionInitialized"].get<bool>();
    if (obj.contains("mapDescription")) {
      map_description = obj["mapDescription"].get<std::string>();
    }
    if (obj.contains("localizationScore")) {
      localization_score = obj["localizationScore"].get<float>();
    }
    if (obj.contains("deviationRange")) {
      deviation_range = obj["deviationRange"].get<float>();
    }
  }

  nlohmann::json to_json() {
    nlohmann::json res;
    res["x"] = x;
    res["y"] = y;
    res["theta"] = theta;
    res["mapId"] = map_id;
    res["positionInitialized"] = position_initialized;
    if (map_description.has_value()) {
      res["mapDescription"] = map_description.value();
    }
    if (deviation_range.has_value()) {
      res["deviationRange"] = deviation_range.value();
    }
    if (localization_score.has_value()) {
      res["localizationScore"] = localization_score.value();
    }
    return res;
  }
};

// edgeStates

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

struct EdgeState {
  std::string edge_id;
  int sequence_id;
  bool released;
  std::optional<std::string> edge_description;
  std::optional<Trajectory> trajectory;
  EdgeState() {}
  EdgeState(nlohmann::json& obj) {
    edge_id = obj["edgeId"].get<std::string>();
    sequence_id = obj["sequenceId"].get<int>();
    released = obj["released"].get<bool>();
    if (obj.contains("edgeDescription")) {
      edge_description = obj["edgeDescription"].get<std::string>();
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
    if (edge_description.has_value()) {
      res["edgeDescription"] = edge_description.value();
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

// nodestates

struct NodePosition {
  float x;
  float y;
  float theta;
  std::string map_id;
};

struct NodeState {
  std::string node_id;
  int sequence_id;
  bool released;
  std::optional<std::string> node_description;
  std::optional<NodePosition> node_position;
  NodeState(){};
  NodeState(nlohmann::json& obj) {
    node_id = obj["nodeId"].get<std::string>();
    sequence_id = obj["sequenceId"].get<int>();
    released = obj["released"].get<bool>();
    if (obj.contains("nodeDescription")) {
      node_description = obj["nodeDescription"].get<std::string>();
    }
    if (obj.contains("nodePosition")) {
      node_position = NodePosition();
      node_position->x = obj["nodePosition"]["x"].get<float>();
      node_position->y = obj["nodePosition"]["y"].get<float>();
      node_position->theta = obj["nodePosition"]["theta"].get<float>();
      node_position->map_id = obj["nodePosition"]["mapId"].get<std::string>();
    }
  }
  nlohmann::json to_json() {
    nlohmann::json res;
    res["nodeId"] = node_id;
    res["released"] = released;
    res["sequenceId"] = sequence_id;
    if (node_description.has_value()) {
      res["nodeDescription"] = node_description.value();
    }
    if (node_position.has_value()) {
      nlohmann::json p;
      p["x"] = node_position.value().x;
      p["y"] = node_position.value().y;
      p["theta"] = node_position.value().theta;
      p["mapId"] = node_position.value().map_id;
      res["nodePosition"] = p;
    }
    return res;
  }
};

//

struct VDA5050State {
  int header_id;
  std::string timestamp;
  std::string version;
  std::string manufacturer;
  std::string serial_number;
  std::string order_id;
  int order_update_id;
  std::string last_node_id;
  int last_node_seq_id;
  std::vector<NodeState> nodestates;
  std::vector<EdgeState> edgestates;
  bool driving;
  std::vector<ActionState> actionstates;
  BatteryState battery_state;
  OperMode operating_mode;
  std::vector<Error> errors;
  SafetyState safetystate;
  // optional
  std::optional<bool> paused;
  std::optional<bool> newbase_request;
  std::optional<float> distance_since_last_node;
  std::optional<std::string> zoneset_id;
  std::optional<AgvPosition> agv_position;
  std::optional<Velocity> velocity;
  std::optional<std::vector<Load>> loads;
  std::optional<InforMation> information;
  VDA5050State(){};
  VDA5050State(nlohmann::json& obj) {
    header_id = obj["headerId"].get<int>();
    timestamp = obj["timestamp"].get<std::string>();
    version = obj["version"].get<std::string>();
    manufacturer = obj["manufacturer"].get<std::string>();
    serial_number = obj["serialNumber"].get<std::string>();
    order_id = obj["orderId"].get<std::string>();
    order_update_id = obj["orderUpdateId"].get<int>();
    last_node_id = obj["lastNodeId"].get<std::string>();
    last_node_seq_id = obj["lastNodeSequenceId"].get<int>();
    for (auto& x : obj["nodeStates"]) {
      auto s = NodeState(x);
      nodestates.push_back(s);
    }

    for (auto& x : obj["edgeStates"]) {
      auto e = EdgeState(x);
      edgestates.push_back(e);
    }

    driving = obj["driving"].get<bool>();
    for (auto& x : obj["actionStates"]) {
      auto s = ActionState(x);
      actionstates.push_back(s);
    }

    battery_state = BatteryState(obj["batteryState"]);
    if (obj["operatingMode"].get<std::string>() == "AUTOMATIC") {
      operating_mode = OperMode::AUTOMATIC;
    } else if (obj["operatingMode"].get<std::string>() == "SEMIAUTOMATIC") {
      operating_mode = OperMode::SEMIAUTOMATIC;
    } else if (obj["operatingMode"].get<std::string>() == "MANUAL") {
      operating_mode = OperMode::MANUAL;
    } else if (obj["operatingMode"].get<std::string>() == "SERVICE") {
      operating_mode = OperMode::SERVICE;
    } else {
      operating_mode = OperMode::TEACHIN;
    }

    for (auto& x : obj["errors"]) {
      auto e = Error(x);
      errors.push_back(e);
    }

    safetystate = SafetyState(obj["safetyState"]);
    // optional

    if (obj.contains("paused")) {
      paused = obj["paused"].get<bool>();
    }
    if (obj.contains("newBaseRequest")) {
      newbase_request = obj["newBaseRequest"].get<bool>();
    }
    if (obj.contains("distanceSinceLastNode")) {
      distance_since_last_node = obj["distanceSinceLastNode"].get<bool>();
    }
    if (obj.contains("zoneSetId")) {
      zoneset_id = obj["zoneSetId"].get<std::string>();
    }
    if (obj.contains("agvPosition")) {
      agv_position = AgvPosition(obj["agvPosition"]);
    }
    if (obj.contains("velocity")) {
      velocity = Velocity(obj["velocity"]);
    }
    if (obj.contains("loads")) {
      loads = std::vector<Load>();
      for (auto& x : obj["loads"]) {
        auto l = Load(x);
        loads.value().push_back(l);
      }
    }
    if (obj.contains("information")) {
      information = InforMation(obj["information"]);
    }
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
    res["lastNodeId"] = last_node_id;
    res["lastNodeSequenceId"] = last_node_seq_id;
    res["nodeStates"] = nlohmann::json::array();
    for (auto& x : nodestates) {
      res["nodeStates"].push_back(x.to_json());
    }
    res["edgeStates"] = nlohmann::json::array();
    for (auto& x : edgestates) {
      res["edgeStates"].push_back(x.to_json());
    }
    res["driving"] = driving;
    res["actionStates"] = nlohmann::json::array();
    for (auto& x : actionstates) {
      res["actionStates"].push_back(x.to_json());
    }
    res["batteryState"] = battery_state.to_json();
    if (operating_mode == OperMode::AUTOMATIC) {
      res["operatingMode"] = "AUTOMATIC";
    } else if (operating_mode == OperMode::SEMIAUTOMATIC) {
      res["operatingMode"] = "SEMIAUTOMATIC";
    } else if (operating_mode == OperMode::MANUAL) {
      res["operatingMode"] = "MANUAL";
    } else if (operating_mode == OperMode::SERVICE) {
      res["operatingMode"] = "SERVICE";
    } else {
      res["operatingMode"] = "TEACHIN";
    }
    res["errors"] = nlohmann::json::array();
    for (auto& x : errors) {
      res["errors"].push_back(x.to_json());
    }
    res["safetyState"] = safetystate.to_json();

    // optinal
    if (paused.has_value()) {
      res["paused"] = paused.value();
    }
    if (newbase_request.has_value()) {
      res["newBaseRequest"] = newbase_request.value();
    }
    if (distance_since_last_node.has_value()) {
      res["distanceSinceLastNode"] = distance_since_last_node.value();
    }
    if (zoneset_id.has_value()) {
      res["zoneSetId"] = zoneset_id.value();
    }
    if (agv_position.has_value()) {
      res["agvPosition"] = agv_position.value().to_json();
    }
    if (velocity.has_value()) {
      res["velocity"] = velocity.value().to_json();
    }
    if (loads.has_value()) {
      res["loads"] = nlohmann::json::array();
      for (auto& x : loads.value()) {
        res["loads"].push_back(x.to_json());
      }
    }
    if (information.has_value()) {
      res["information"] = information.value().to_json();
    }
    return res;
  }
};
}  // namespace state

}  // namespace vda5050
#endif