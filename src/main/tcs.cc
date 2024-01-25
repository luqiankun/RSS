#include "../../include/main/tcs.hpp"

#include "../../include/kernel/driver/vehicle.hpp"

bool TCS::init_resource(const std::string &xml_path) {
  this->resource.reset();
  this->resource =
      std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
  pugi::xml_document doc;
  auto ret = doc.load_file(xml_path.c_str());
  if (ret.status != pugi::status_ok) {
    return false;
  }
  //
  auto root = doc.first_child();  // <model>
  if (std::string(root.name()) != "model") {
    return false;
  }
  {  // point
    auto point = root.find_child([](pugi::xml_node node) {
      return std::string(node.name()) == "point";
    });
    while (point.type() != pugi::node_null) {
      if (std::string(point.name()) != "point") {
        break;
      }
      std::string name = point.attribute("name").as_string();
      std::string type = point.attribute("type").as_string();
      int xPosition = point.attribute("xPosition").as_int();
      int yPosition = point.attribute("yPosition").as_int();
      int zPosition = point.attribute("zPosition").as_int();
      int xPosition_layout =
          point.child("pointLayout").attribute("xPosition").as_int();
      int yPosition_layout =
          point.child("pointLayout").attribute("yPosition").as_int();
      int xLabelOffset =
          point.child("pointLayout").attribute("xLabelOffset").as_int();
      int yLabelOffset =
          point.child("pointLayout").attribute("yLabelOffset").as_int();
      int layerId = point.child("pointLayout").attribute("layerId").as_int();
      data::model::Point::Layout layout;
      layout.layer_id = layerId;
      layout.label_offset = Eigen::Vector2i(xLabelOffset, yLabelOffset);
      layout.position = Eigen::Vector2i(xPosition_layout, yPosition_layout);
      auto p = std::make_shared<data::model::Point>(name);
      p->position.x() = xPosition;
      p->position.y() = yPosition;
      p->layout = layout;
      if (type == "HALT_POSITION") {
        p->type = data::model::Point::Type::HALT_POSITION;
      } else if (type == "REPORT_POSITION") {
        p->type = data::model::Point::Type::REPORT_POSITION;
      } else if (type == "PARK_POSITION") {
        p->type = data::model::Point::Type::PARK_POSITION;
      } else if (type == "NORMAL_POSITION") {
        p->type = data::model::Point::Type::NORMAL_POSITION;
      } else {
        p->type = data::model::Point::Type::UNKNOWN;
      }
      resource->points.push_back(p);
      point = point.next_sibling();
    }
    LOG(INFO) << "init point size " << resource->points.size();
  }

  {  // path
    auto path = root.find_child(
        [](pugi::xml_node node) { return std::string(node.name()) == "path"; });
    while (path.type() != pugi::node_null) {
      if (std::string(path.name()) != "path") {
        break;
      }
      std::string name = path.attribute("name").as_string();
      std::string sourcePoint = path.attribute("sourcePoint").as_string();
      std::string destinationPoint =
          path.attribute("destinationPoint").as_string();
      int length = path.attribute("length").as_int();
      int maxVelocity = path.attribute("maxVelocity").as_int();
      int maxReverseVelocity = path.attribute("maxReverseVelocity").as_int();
      bool locked = path.attribute("locked").as_bool();
      std::string connectionType =
          path.child("pathLayout").attribute("connectionType").as_string();
      int layerId = path.child("pointLayout").attribute("layerId").as_int();
      std::weak_ptr<data::model::Point> source_point;
      std::weak_ptr<data::model::Point> destination_point;
      for (auto &p : resource->points) {
        if (p->name == sourcePoint) {
          source_point = p;
        }
        if (p->name == destinationPoint) {
          destination_point = p;
        }
      }
      data::model::Path::PathLayout layout;
      layout.layer_id = layerId;
      data::model::Path::new_connect_type(connectionType);
      auto p = std::make_shared<data::model::Path>(name);
      p->layout = layout;
      p->source_point = source_point;
      p->destination_point = destination_point;
      p->length = length;
      p->max_vel = maxVelocity;
      p->max_reverse_vel = maxReverseVelocity;
      p->locked = locked;
      p->source_point.lock()->incoming_paths.push_back(p);
      p->destination_point.lock()->outgoing_paths.push_back(p);
      resource->paths.push_back(p);
      path = path.next_sibling();
    }
    LOG(INFO) << "init path size " << resource->paths.size();
  }

  {  // location
    auto location = root.find_child([](pugi::xml_node node) {
      return std::string(node.name()) == "location";
    });
    while (location.type() != pugi::node_null) {
      if (std::string(location.name()) != "location") {
        break;
      }
      std::string name = location.attribute("name").as_string();
      std::string type = location.attribute("type").as_string();
      std::string link_point =
          location.child("link").attribute("point").as_string();
      int xPosition = location.attribute("xPosition").as_int();
      int yPosition = location.attribute("yPosition").as_int();
      int zPosition = location.attribute("zPosition").as_int();
      bool locked = location.attribute("locked").as_bool();
      int xPosition_layout =
          location.child("locationLayout").attribute("xPosition").as_int();
      int yPosition_layout =
          location.child("locationLayout").attribute("yPosition").as_int();
      int xLabelOffset =
          location.child("locationLayout").attribute("xLabelOffset").as_int();
      int yLabelOffset =
          location.child("locationLayout").attribute("yLabelOffset").as_int();
      int layerId =
          location.child("locationLayout").attribute("layerId").as_int();
      std::string locationRepresentation =
          location.child("locationLayout")
              .attribute("locationRepresentation")
              .as_string();
      data::model::Location::Layout layout;
      layout.position = Eigen::Vector2i(xPosition_layout, yPosition_layout);
      layout.label_offset = Eigen::Vector2i(xLabelOffset, yLabelOffset);
      layout.layer_id = layerId;
      // data::model::Location::LocationTypeLayout location_type_layout;
      // if (locationRepresentation == "NONE") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::NONE;
      // } else if (locationRepresentation == "DEFAULT") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::DEFAULT;
      // } else if (locationRepresentation == "LOAD_TRANSFER_GENERIC") {
      //   location_type_layout.location_representation =
      //   data::model::Location::
      //       LocationRepresentation::LOAD_TRANSFER_GENERIC;
      // } else if (locationRepresentation == "LOAD_TRANSFER_ALT_1") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::LOAD_TRANSFER_ALT_1;
      // } else if (locationRepresentation == "LOAD_TRANSFER_ALT_2") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::LOAD_TRANSFER_ALT_2;
      // } else if (locationRepresentation == "LOAD_TRANSFER_ALT_3") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::LOAD_TRANSFER_ALT_3;
      // } else if (locationRepresentation == "LOAD_TRANSFER_ALT_4") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::LOAD_TRANSFER_ALT_4;
      // } else if (locationRepresentation == "LOAD_TRANSFER_ALT_5") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::LOAD_TRANSFER_ALT_5;
      // } else if (locationRepresentation == "WORKING_GENERIC") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::WORKING_GENERIC;
      // } else if (locationRepresentation == "WORKING_ALT_1") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::WORKING_ALT_1;
      // } else if (locationRepresentation == "WORKING_ALT_2") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::WORKING_ALT_2;
      // } else if (locationRepresentation == "RECHARGE_GENERIC") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::RECHARGE_GENERIC;
      // } else if (locationRepresentation == "RECHARGE_ALT_1") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::RECHARGE_ALT_1;
      // } else if (locationRepresentation == "RECHARGE_ALT_2") {
      //   location_type_layout.location_representation =
      //       data::model::Location::LocationRepresentation::RECHARGE_ALT_2;
      // }
      auto loc = std::make_shared<data::model::Location>(name);
      std::weak_ptr<data::model::Point> link;
      for (auto &p : resource->points) {
        if (p->name == link_point) {
          link = p;
          p->attached_links.push_back(loc);
        }
      }
      loc->position = Eigen::Vector3i(xPosition, yPosition, zPosition);
      loc->layout = layout;
      loc->link = link;
      loc->locked = locked;
      resource->locations.push_back(loc);
      location = location.next_sibling();
    }
    LOG(INFO) << "init location size " << resource->locations.size();
  }

  //
  auto vehicle = root.find_child([](pugi::xml_node node) {
    return std::string(node.name()) == "vehicle";
  });
  while (vehicle.type() != pugi::node_null) {
    if (std::string(vehicle.name()) != "vehicle") {
      break;
    }
    std::string name = vehicle.attribute("name").as_string();
    int length = vehicle.attribute("length").as_int();
    int maxVelocity = vehicle.attribute("maxVelocity").as_int();
    int maxReverseVelocity = vehicle.attribute("maxReverseVelocity").as_int();
    std::string color =
        vehicle.child("vehicleLayout").attribute("color").as_string();
    int energyLevelCritical = vehicle.attribute("energyLevelCritical").as_int();
    int energyLevelGood = vehicle.attribute("energyLevelGood").as_int();
    int energyLevelFullyRecharged =
        vehicle.attribute("energyLevelFullyRecharged").as_int();
    int energyLevelSufficientlyRecharged =
        vehicle.attribute("energyLevelSufficientlyRecharged").as_int();
    ///////////////////
    /// // 使用仿真车辆
    //////////////////
    auto veh = std::make_shared<kernel::driver::SimVehicle>(name);
    veh->length = length;
    veh->width = 2.0 * length / 4;
    veh->max_reverse_vel = maxReverseVelocity;
    veh->max_vel = maxVelocity;
    veh->color = color;
    veh->energy_level_critical = energyLevelCritical;
    veh->energy_level_good = energyLevelGood;
    veh->engrgy_level_full = energyLevelFullyRecharged;
    veh->engrgy_level_recharge = energyLevelSufficientlyRecharged;
    veh->position.x() = resource->points.front()->position.x();
    veh->position.y() = resource->points.front()->position.y();
    veh->layout = veh->layout;
    veh->current_point = resource->points.front();  // 当前点
    veh->init_point = resource->points.front();     // 初始点
    // veh->run();
    dispatcher->vehicles.push_back(veh);
    ///////////////////////
    vehicle = vehicle.next_sibling();
  }
  LOG(INFO) << "init vehicle size " << dispatcher->vehicles.size();

  ///
  LOG(INFO) << "init resource ok";
  return true;
}

void TCS::paused_vehicle(const std::string &name) {
  std::hash<std::string> hash_fn;
  for (auto &v : dispatcher->vehicles) {
    if (v->name_hash == hash_fn(name)) {
      v->paused = true;
    }
  }
}
void TCS::recovery_vehicle(const std::string &name) {
  std::hash<std::string> hash_fn;
  for (auto &v : dispatcher->vehicles) {
    if (v->name_hash == hash_fn(name)) {
      v->paused = false;
    }
  }
}

void TCS::stop() {
  if (dispatcher) {
    dispatcher->stop();
    dispatcher.reset();
  }
  if (scheduler) {
    scheduler.reset();
  }
  if (orderpool) {
    orderpool.reset();
  }
  if (resource) {
    resource.reset();
  }
}
void TCS::home_order(const std::string &name,
                     std::shared_ptr<kernel::driver::Vehicle> v) {
  auto time = std::chrono::system_clock::now();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>(name);
  std::hash<std::string> hash_fn;
  ord->create_time = time;
  ord->dead_time = time + std::chrono::minutes(60);
  ord->state = data::order::TransportOrder::State::RAW;
  LOG(INFO) << "new ord " << ord->name << " name_hash " << ord->name_hash;
  auto destination = orderpool->res_to_destination(
      v->init_point, data::order::DriverOrder::Destination::OpType::MOVE);
  auto dr = std::make_shared<data::order::DriverOrder>("driverorder_" +
                                                       v->init_point->name);
  dr->destination = destination;
  dr->transport_order = ord;
  ord->driverorders.push_back(dr);
  ord->intended_vehicle = v;
  orderpool->orderpool.push_back(ord);
}

TCS::~TCS() {
#ifdef VISUAL
  if (visualizer) {
    visualizer->stop();
    visualizer.reset();
  }
#endif
  LOG(INFO) << "TCS  stop";
  stop();
}

bool TCS::init_dispatcher() {
  dispatcher = std::make_shared<kernel::dispatch::Dispatcher>("Dispatcher");
  LOG(INFO) << "init dispatcher ok";
  return true;
}
bool TCS::init_orderpool() {
  orderpool = std::make_shared<kernel::allocate::OrderPool>("OrderPool");
  LOG(INFO) << "init orderpool ok";
  return true;
}
bool TCS::init_scheduler() {
  this->scheduler = std::make_shared<kernel::schedule::Scheduler>("Scheduler");
  LOG(INFO) << "init scheduler ok";
  return true;
}
bool TCS::init_planner() {
  if (!resource) {
    return false;
  }
  this->planner = std::make_shared<kernel::planner::Planner>(this->resource);
  LOG(INFO) << "init planner ok";
  return true;
}
#ifdef VISUAL
bool TCS::init_visualizer(double resolution) {
  visualizer = std::make_shared<visual::Visualizer>();
  auto ret = visualizer->init(resolution, shared_from_this());
  if (ret) {
    LOG(INFO) << "init visualizer ok";
  }
  return ret;
}
#endif
bool TCS::init_all(const std::string &xml_path, double r) {
  if (!init_orderpool()) {
    return false;
  }
  if (!init_scheduler()) {
    return false;
  }
  if (!init_dispatcher()) {
    return false;
  }
  if (!init_resource(xml_path)) {
    return false;
  }
  if (!init_planner()) {
    return false;
  }
#ifdef VISUAL
  if (!init_visualizer(r)) {
    return false;
  }
#endif
  //
  scheduler->resource = resource;
  // connect signals
  dispatcher->find_res.connect(
      std::bind(&kernel::allocate::ResourceManager::find, resource,
                std::placeholders::_1));
  dispatcher->go_home.connect(std::bind(
      &TCS::home_order, this, std::placeholders::_1, std::placeholders::_2));
  dispatcher->get_next_ord.connect(
      std::bind(&kernel::allocate::OrderPool::pop, orderpool));
  for (auto &v : dispatcher->vehicles) {
    v->planner = planner;
    v->scheduler = scheduler;
    v->orderpool = orderpool;
    v->resource = resource;
  }

  return true;
}

void TCS::cancel_all_order() { orderpool->cancel_all_order(); }

void TCS::cancel_order(const std::string &order_name) {
  std::hash<std::string> hash_fn;
  orderpool->cancel_order(hash_fn(order_name));
}
void TCS::cancel_vehicle_all_order(const std::string &vehicle_name) {
  std::hash<std::string> hash_fn;
  for (auto &v : dispatcher->vehicles) {
    if (v->name_hash == hash_fn(vehicle_name)) {
      v->cancel_all_order();
    }
  }
}
void TCS::run() {
  if (dispatcher) {
    for (auto &v : dispatcher->vehicles) {
      v->run();
    }
    dispatcher->run();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (scheduler) {
    scheduler->run();
  }
#ifdef VISUAL
  if (visualizer) {
    visualizer->run();
  }
#endif
}

json order_to_json(std::shared_ptr<data::order::TransportOrder> v) {
  json value;
  if (v) {
    value["dispensable"] = v->dispensable;
    value["name"] = v->name;
    value["type"] = v->type;
    value["state"] = v->get_state();
    value["intendedVehicle"] =
        v->intended_vehicle.lock() ? v->intended_vehicle.lock()->name : "";
    value["processingVehicle"] =
        v->processing_vehicle.lock() ? v->processing_vehicle.lock()->name : "";
    value["peripheralReservationToken"] = "";
    value["wrappingSequence"] =
        v->ordersequence.lock() ? v->ordersequence.lock()->name : "";
    value["destinations"] = json::array();
    for (auto &dest : v->driverorders) {
      json dest_v;
      auto dest_ = dest->destination->destination.lock();
      dest_v["destinations"] = dest_ ? dest_->name : "";
      dest_v["operation"] = dest->destination->get_type();
      dest_v["state"] = dest->get_state();
      dest_v["properties"] = json::array();
      for (auto &property : dest->properties) {
        json pro;
        pro["key"] = property.first;
        pro["value"] = property.second;
        dest_v["properties"].push_back(pro);
      }
      value["destinations"].push_back(dest_v);
    }
  }
  return value;
}

std::pair<int, std::string> TCS::get_transport_orders(
    const std::string &vehicle) {
  if (vehicle.empty()) {
    json res = json::array();
    for (auto &v : orderpool->orderpool) {
      json value = order_to_json(v);
      res.push_back(value);
    }
    for (auto &v : orderpool->ended_orderpool) {
      json value = order_to_json(v);
      res.push_back(value);
    }
    return std::pair<int, std::string>(200, res.dump());
  } else {
    for (auto &v : dispatcher->vehicles) {
      if (v->name == vehicle) {
        json res = json::array();
        for (auto &v : v->orders) {
          json value = order_to_json(v);
          res.push_back(value);
        }
        return std::pair<int, std::string>(200, res.dump());
      }
    }
    json res = json::array();
    auto msg = "Could not find the intended vehicle '" + vehicle + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
}

std::pair<int, std::string> TCS::get_transport_order(
    const std::string &ord_name) {
  json res = json::array();
  for (auto &ord : orderpool->orderpool) {
    if (ord->name == ord_name) {
      res.push_back(order_to_json(ord));
      return std::pair<int, std::string>(200, res.dump());
    }
  }
  for (auto &ord : orderpool->ended_orderpool) {
    if (ord->name == ord_name) {
      res.push_back(order_to_json(ord));
      return std::pair<int, std::string>(200, res.dump());
    }
  }
  auto msg = "Could not find transport order '" + ord_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}
std::pair<int, std::string> TCS::post_transport_order(
    const std::string &ord_name, const std::string &body) {
  for (auto &o : orderpool->orderpool) {
    if (o->name == ord_name) {
      json res = json::array();
      auto msg = "Transport order '" + ord_name + "' already exists.";
      res.push_back(msg);
      LOG(ERROR) << msg;
      return std::pair<int, std::string>(409, res.dump());
    }
  }
  for (auto &o : orderpool->ended_orderpool) {
    if (o->name == ord_name) {
      json res = json::array();
      auto msg = "Transport order '" + ord_name + "' already exists.";
      res.push_back(msg);
      LOG(ERROR) << msg;
      return std::pair<int, std::string>(409, res.dump());
    }
  }
  try {
    auto req = json::parse(body);
    // TODO字段检查
    auto ord = std::make_shared<data::order::TransportOrder>(ord_name);
    ord->create_time = std::chrono::system_clock::now();
    auto dt = get_time_from_str(req["deadline"].get<std::string>());
    ord->dead_time = dt.value_or(ord->create_time + std::chrono::minutes(60));
    for (auto &v : dispatcher->vehicles) {
      if (v->name == req["intendedVehicle"].get<std::string>()) {
        ord->intended_vehicle = v;
      }
    }
    bool quence{false};
    for (auto &o : orderpool->orderquence) {
      if (o->name == req["wrappingSequence"].get<std::string>()) {
        o->add_transport_ord(ord);
        quence = true;
        break;
      }
    }
    if (!quence) {
      // TODO new orderquence
      auto new_orderquence = std::make_shared<data::order::OrderSequence>(
          req["wrappingSequence"].get<std::string>());
      new_orderquence->add_transport_ord(ord);
      orderpool->orderquence.push_back(new_orderquence);
    }
    ord->type = req["type"].get<std::string>();
    auto destinations = req["destinations"];
    if (destinations.empty()) {
      LOG(WARNING) << ord->name << " op is null";
      ord->state = data::order::TransportOrder::State::FAILED;
      orderpool->ended_orderpool.push_back(ord);
    } else {
      for (auto &d : destinations) {
        // TODO
        auto loc = d["locationName"].get<std::string>();
        auto check = resource->find(loc);
        if (check.first == kernel::allocate::ResourceManager::ResType::Err) {
          json res = json::array();
          auto msg = "Could not find location '" + loc + "'.";
          res.push_back(msg);
          LOG(ERROR) << msg;
          ord->state = data::order::TransportOrder::State::UNROUTABLE;
          orderpool->ended_orderpool.push_back(ord);
          return std::pair<int, std::string>(404, res.dump());
        } else {
          auto op = d["operation"].get<std::string>();
          auto op_ = data::order::DriverOrder::Destination::get_optype(op);
          if (op_.has_value()) {
            auto destination =
                orderpool->res_to_destination(check.second, op_.value());
            auto dr = std::make_shared<data::order::DriverOrder>(
                "driverorder_" + loc);
            dr->destination = destination;
            dr->transport_order = ord;
            ord->driverorders.push_back(dr);
          } else {
            ord->state = data::order::TransportOrder::State::FAILED;
            orderpool->ended_orderpool.push_back(ord);
            break;
          }
        }
      }
    }
    auto properties = req["properties"];
    for (auto &pro : properties) {
      ord->properties.insert(std::pair<std::string, std::string>(
          pro["key"].get<std::string>(), pro["value"].get<std::string>()));
    }
    for (auto &dep : req["dependencies"]) {
      for (auto &o : orderpool->orderpool) {
        if (o->name == dep.get<std::string>()) {
          ord->dependencies.push_back(o);
        }
      }
      for (auto &o : orderpool->ended_orderpool) {
        if (o->name == dep.get<std::string>()) {
          ord->dependencies.push_back(o);
        }
      }
    }
    ord->state = data::order::TransportOrder::State::RAW;
    orderpool->orderpool.push_back(ord);
    // return
    json res;
    res["dispensable"] = ord->dispensable;
    res["name"] = ord->name;
    res["type"] = ord->type;
    res["state"] = ord->get_state();
    res["intendedVehicle"] =
        ord->intended_vehicle.lock() ? ord->intended_vehicle.lock()->name : "";
    res["processingVehicle"] = ord->processing_vehicle.lock()
                                   ? ord->processing_vehicle.lock()->name
                                   : "";
    res["peripheralReservationToken"] = req["peripheralReservationToken"];
    res["wrappingSequence"] =
        ord->ordersequence.lock() ? ord->ordersequence.lock()->name : "";
    res["destinations"] = json::array();
    for (auto &dest : ord->driverorders) {
      json value;
      value["locationName"] = dest->destination->destination.lock()->name;
      value["operation"] = dest->destination->get_type();
      value["state"] = dest->get_state();
      value["properties"] = json::array();
      for (auto &pro : dest->properties) {
        json p;
        p["key"] = pro.first;
        p["value"] = pro.second;
        value["properties"].push_back(p);
      }
      res["destinations"].push_back(value);
    }
    return std::pair<int, std::string>(200, res.dump());
  } catch (json::parse_error ec) {
    LOG(ERROR) << "parse failed :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::type_error ec) {
    LOG(ERROR) << "type error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::other_error ec) {
    LOG(ERROR) << "other error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  }
}

std::pair<int, std::string> TCS::post_transport_order_withdrawl(
    const std::string &ord_name, bool immediate, bool) {
  for (auto &o : orderpool->orderpool) {
    if (o->name == ord_name) {
      o->state = data::order::TransportOrder::State::WITHDRAWL;
      return std::pair<int, std::string>(200, "");
    }
  }
  for (auto &o : orderpool->ended_orderpool) {
    if (o->name == ord_name) {
      o->state = data::order::TransportOrder::State::WITHDRAWL;
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find transport order '" + ord_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

json orderquence_to_json(std::shared_ptr<data::order::OrderSequence> quence) {
  json res;
  res["name"] = quence->name;
  res["type"] = quence->get_type();
  res["orders"] = json::array();
  for (auto &ord : quence->orders) {
    res["orders"].push_back(ord->name);
  }
  res["finishedIndex"] = quence->finished_index;
  res["complete"] = quence->complete;
  res["finished"] = quence->finished;
  res["failureFatal"] = quence->failure_fatal;
  res["intendedVehicle"] = quence->intended_vehicle.lock()
                               ? quence->intended_vehicle.lock()->name
                               : "";
  res["processingVehicle"] = quence->processing_vehicle.lock()
                                 ? quence->processing_vehicle.lock()->name
                                 : "";
  res["properties"] = json::array();
  for (auto &pro : quence->properties) {
    json p;
    p["key"] = pro.first;
    p["value"] = pro.second;
    res["properties"].push_back(p);
  }
  return res;
}
std::pair<int, std::string> TCS::get_ordersequences(
    const std::string &vehicle) {
  if (!vehicle.empty()) {
    json res = json::array();
    for (auto &o : orderpool->orderquence) {
      if (o->intended_vehicle.lock()) {
        if (o->intended_vehicle.lock()->name == vehicle) {
          res.push_back(orderquence_to_json(o));
        }
      }
    }
    if (res.empty()) {
      auto msg = "Could not find the intended vehicle '" + vehicle + "'.";
      res.push_back(msg);
      return std::pair<int, std::string>(404, res.dump());
    } else {
      return std::pair<int, std::string>(200, res.dump());
    }
  } else {
    json res = json::array();
    for (auto &q : orderpool->orderquence) {
      res.push_back(orderquence_to_json(q));
    }
    return std::pair<int, std::string>(200, res.dump());
  }
}

std::pair<int, std::string> TCS::get_ordersequence(
    const std::string &sequence_name) {
  for (auto &q : orderpool->orderquence) {
    if (q->name == sequence_name) {
      auto res = orderquence_to_json(q);
      return std::pair<int, std::string>(200, res.dump());
    }
  }
  json res = json::array();
  auto msg = "Could not find order sequence '" + sequence_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

std::pair<int, std::string> TCS::post_ordersequence(
    const std::string &sequence_name, const std::string &body) {
  for (auto &s : orderpool->orderquence) {
    if (s->name == sequence_name) {
      json res = json::array();
      auto msg = "Order sequence '" + sequence_name + "' already exists.";
      res.push_back(msg);
      return std::pair<int, std::string>(409, res.dump());
    }
  }
  try {
    auto req = json::parse(body);
    auto veh = req["intendedVehicle"].get<std::string>();
    for (auto &v : dispatcher->vehicles) {
      if (v->name == veh) {
        auto new_orderquence = std::make_shared<data::order::OrderSequence>(
            req["name"].get<std::string>());
        new_orderquence->intended_vehicle = v;
        new_orderquence->failure_fatal = req["failureFatal"].get<bool>();
        new_orderquence->complete = req["incompleteName"].get<bool>();
        for (auto &pro : req["properties"]) {
          new_orderquence->properties[pro["key"].get<std::string>()] =
              pro["value"].get<std::string>();
        }
        // return
        auto res = orderquence_to_json(new_orderquence);
        orderpool->orderquence.push_back(new_orderquence);
        return std::pair<int, std::string>(200, res.dump());
      }
    }
    json res = json::array();
    auto msg = "Could not find Vehicle '" + veh + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  } catch (json::parse_error ec) {
    LOG(ERROR) << "parse error:" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::type_error ec) {
    LOG(ERROR) << "type error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::other_error ec) {
    LOG(ERROR) << "other error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  }
}

json vehicle_to_json(std::shared_ptr<kernel::driver::Vehicle> v) {
  json res;
  res["name"] = v->name;
  res["length"] = v->length;
  for (auto &pro : v->properties) {
    json t;
    t[pro.first] = pro.second;
    res["properties"].push_back(t);
  }
  res["energyLevel"] = v->engerg_level;
  res["energyLevelGood"] = v->energy_level_good;
  res["integrationLevel"] = v->integration_level;
  res["paused"] = v->paused;
  res["procState"] = v->get_proc_state();
  res["transportOrder"] = v->current_order ? v->current_order->name : "";
  res["currentPosition"] = v->current_point ? v->current_point->name : "";
  res["precisePosition"]["x"] = v->position.x();
  res["precisePosition"]["y"] = v->position.y();
  res["precisePosition"]["z"] = v->position.z();
  res["state"] = v->get_state();
  res["allocatedResources"] = json::array();
  for (auto &r : v->allocate_resources) {
    res["allocatedResources"].push_back(r->name);
  }
  res["claimedResources"] = json::array();
  for (auto &r : v->claim_resources) {
    res["claimedResources"].push_back(r->name);
  }
  // todo
  res["allowedOrderTypes"] = json::array();
  res["envelopeKey"] = "";
  return res;
}

std::pair<int, std::string> TCS::get_vehicles(const std::string &state) {
  if (state.empty()) {
    json res = json::array();
    for (auto &v : dispatcher->vehicles) {
      res.push_back(vehicle_to_json(v));
    }
    return std::pair<int, std::string>(200, res.dump());
  } else {
    if (state == kernel::driver::Vehicle::get_proc_state(
                     kernel::driver::Vehicle::ProcState::IDLE)) {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->proc_state == kernel::driver::Vehicle::ProcState::IDLE) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state ==
               kernel::driver::Vehicle::get_proc_state(
                   kernel::driver::Vehicle::ProcState::AWAITING_ORDER)) {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->proc_state ==
            kernel::driver::Vehicle::ProcState::AWAITING_ORDER) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state ==
               kernel::driver::Vehicle::get_proc_state(
                   kernel::driver::Vehicle::ProcState::PROCESSING_ORDER)) {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->proc_state ==
            kernel::driver::Vehicle::ProcState::PROCESSING_ORDER) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else {
      json res = json::array();
      auto msg = "Could not parse input.";
      res.push_back(msg);
      return std::pair<int, std::string>(400, res.dump());
    }
  }
}

std::pair<int, std::string> TCS::get_vehicle(const std::string &vehicle) {
  for (auto &v : dispatcher->vehicles) {
    if (v->name == vehicle) {
      auto res = vehicle_to_json(v);
      return std::pair<int, std::string>(200, res.dump());
    }
  }
  json res = json::array();
  auto msg = "Could not find vehicle '" + vehicle + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

std::pair<int, std::string> TCS::post_vehicle_withdrawl(
    const std::string &vehicle, bool, bool) {
  for (auto &v : dispatcher->vehicles) {
    if (v->name == vehicle) {
      for (auto &o : v->orders) {
        o->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find vehicle '" + vehicle + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}
std::pair<int, std::string> TCS::get_model() {
  json res;
  res["name"] = resource->model_name;
  // point
  res["points"] = json::array();
  for (auto &p : resource->points) {
    json point;
    point["name"] = p->name;
    point["position"]["x"] = p->position.x();
    point["position"]["y"] = p->position.y();
    point["position"]["z"] = p->position.z();
    point["vehicleOrientationAngle"] = p->client_angle;
    point["type"] = data::model::Point::get_type(p->type);
    json layout;
    layout["position"]["x"] = p->layout.position.x();
    layout["position"]["y"] = p->layout.position.y();
    layout["labelOffset"]["x"] = p->layout.label_offset.x();
    layout["labelOffset"]["y"] = p->layout.label_offset.y();
    layout["layerId"]["y"] = p->layout.layer_id;
    point["layout"] = layout;
    point["properties"] = json::array();
    for (auto &pro : p->properties) {
      json t;
      t["name"] = pro.first;
      t["value"] = pro.second;
      point["properties"].push_back(t);
    }
    res["points"].push_back(point);
  }
  // path
  res["paths"] = json::array();
  for (auto &p : resource->paths) {
    json path;
    path["name"] = p->name;
    path["srcPointName"] =
        p->source_point.lock() ? p->source_point.lock()->name : "";
    path["destPointName"] =
        p->destination_point.lock() ? p->destination_point.lock()->name : "";
    path["maxVelocity"] = p->max_vel;
    path["maxReverseVelocity"] = p->max_reverse_vel;
    path["locked"] = p->locked;
    path["layout"]["connectionType"] =
        data::model::Path::get_connect_type(p->layout.connect_type);
    path["layout"]["layerId"] = p->layout.layer_id;
    path["properties"] = json::array();
    for (auto &pro : p->properties) {
      json t;
      t["name"] = pro.first;
      t["value"] = pro.second;
      path["properties"].push_back(t);
    }
    res["paths"].push_back(path);
  }
  // location
  res["locations"] = json::array();
  for (auto &loc : resource->locations) {
    json location;
    location["name"] = loc->name;
    location["typeNmae"] = "";  // TODO
    location["position"]["x"] = loc->position.x();
    location["position"]["y"] = loc->position.y();
    location["position"]["z"] = loc->position.z();
    location["links"] = json::array();
    location["links"].push_back(loc->link.lock() ? loc->link.lock()->name : "");
    location["locked"] = loc->locked;
    location["layout"]["position"]["x"] = loc->layout.position.x();
    location["layout"]["position"]["y"] = loc->layout.position.y();
    location["layout"]["labelOffset"]["x"] = loc->layout.label_offset.x();
    location["layout"]["labelOffset"]["y"] = loc->layout.label_offset.y();
    location["layout"]["locationRepresentation"] =
        data::model::Location::get_Representation(
            loc->type.layout.location_representation);
    location["layout"]["layerId"] = loc->layout.layer_id;
    res["locations"].push_back(location);
  }

  // vehicles
  res["vehicles"] = json::array();
  for (auto &v : dispatcher->vehicles) {
    json vehicle;
    vehicle["name"] = v->name;
    vehicle["length"] = v->length;
    vehicle["energyLevelCritical"] = v->energy_level_critical;
    vehicle["energyLevelGood"] = v->energy_level_good;
    vehicle["energyLevelFullyRecharged"] = v->engrgy_level_full;
    vehicle["energyLevelSufficientlyRecharged"] = v->engrgy_level_recharge;
    vehicle["maxVelocity"] = v->max_vel;
    vehicle["maxReverseVelocity"] = v->max_reverse_vel;
    vehicle["layout"]["routeColor"] = v->color;
    res["vehicles"].push_back(vehicle);
  }
  return std::pair<int, std::string>(200, res.dump());
}
std::pair<int, std::string> TCS::put_model(const std::string &body) {
  stop();
  init_orderpool();
  init_planner();
  init_scheduler();
  init_dispatcher();
  try {
    json model = json::parse(body);
    resource =
        std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
    resource->model_name = model["name"].get<std::string>();
    // point
    for (auto &p : model["points"]) {
      auto point =
          std::make_shared<data::model::Point>(p["name"].get<std::string>());
      point->position.x() = p["position"]["x"].get<int>();
      point->position.y() = p["position"]["y"].get<int>();
      point->position.z() = p["position"]["z"].get<int>();
      if (p.contains("vehicleOrientationAngle")) {
        point->client_angle = p["vehicleOrientationAngle"].get<int>();
      }
      point->type = data::model::Point::new_type(p["type"].get<std::string>());
      point->layout.position.x() = p["layout"]["position"]["x"].get<int>();
      point->layout.position.y() = p["layout"]["position"]["y"].get<int>();
      point->layout.label_offset.x() =
          p["layout"]["labelOffset"]["x"].get<int>();
      point->layout.label_offset.y() =
          p["layout"]["labelOffset"]["y"].get<int>();
      point->layout.layer_id = p["layout"]["layerId"].get<int>();
      if (p.contains("properties")) {
        for (auto &pro : p["properties"]) {
          point->properties.insert(std::pair<std::string, std::string>(
              pro["name"].get<std::string>(), pro["value"].get<std::string>()));
        }
      }
      resource->points.push_back(point);
    }
    // path
    for (auto &p : model["paths"]) {
      auto path = std::make_shared<data::model::Path>(p["name"]);
      path->max_vel = p["maxVelocity"].get<int>();
      path->max_reverse_vel = p["maxReverseVelocity"].get<int>();
      if (p.contains("locked")) {
        path->locked = p["locked"].get<bool>();
      }
      for (auto &x : resource->points) {
        if (x->name == p["srcPointName"].get<std::string>()) {
          path->source_point = x;
          x->incoming_paths.push_back(path);
        }
        if (x->name == p["destPointName"].get<std::string>()) {
          path->destination_point = x;
          x->outgoing_paths.push_back(path);
        }
      }
      path->layout.layer_id = p["layout"]["layerId"].get<int>();
      path->layout.connect_type =
          data::model::Path::new_connect_type(p["layout"]["connectionType"]);
      if (p.contains("properties")) {
        for (auto &pro : p["properties"]) {
          path->properties.insert(std::pair<std::string, std::string>(
              pro["name"].get<std::string>(), pro["value"].get<std::string>()));
        }
      }
      if (!p.contains("length")) {
        path->length = (path->source_point.lock()->position -
                        path->destination_point.lock()->position)
                           .norm();
      }
      resource->paths.push_back(path);
    }
    // location
    for (auto &l : model["locations"]) {
      auto loc =
          std::make_shared<data::model::Location>(l["name"].get<std::string>());
      loc->position.x() = l["position"]["x"].get<int>();
      loc->position.y() = l["position"]["y"].get<int>();
      loc->position.z() = l["position"]["z"].get<int>();
      if (!l["links"].empty()) {
        auto p_name = l["links"].front()["pointName"].get<std::string>();
        for (auto &x : resource->points) {
          if (x->name == p_name) {
            loc->link = x;
          }
        }
      }
      loc->locked = l["locked"].get<bool>();
      loc->layout.position.x() = l["layout"]["position"]["x"].get<int>();
      loc->layout.position.y() = l["layout"]["position"]["y"].get<int>();
      loc->layout.label_offset.x() = l["layout"]["labelOffset"]["x"].get<int>();
      loc->layout.label_offset.y() = l["layout"]["labelOffset"]["y"].get<int>();
      loc->layout.layer_id = l["layout"]["layerId"].get<int>();
      if (l["layout"].contains("locationRepresentation")) {
        loc->type.layout.location_representation =
            data::model::Location::new_location_type(
                l["layout"]["locationRepresentation"].get<std::string>());
      }
      resource->locations.push_back(loc);
    }
    // vehicle
    for (auto &v : model["vehicles"]) {
      // TODO 工厂
      auto vehicle = std::make_shared<kernel::driver::SimVehicle>(
          v["name"].get<std::string>());
      vehicle->length = v["length"].get<int>();
      vehicle->energy_level_critical = v["energyLevelCritical"].get<int>();
      vehicle->energy_level_good = v["energyLevelGood"].get<int>();
      vehicle->engrgy_level_full = v["energyLevelFullyRecharged"].get<int>();
      vehicle->engrgy_level_recharge =
          v["energyLevelSufficientlyRecharged"].get<int>();
      vehicle->max_vel = v["maxVelocity"].get<int>();
      vehicle->max_reverse_vel = v["maxReverseVelocity"].get<int>();
      vehicle->color = v["layout"]["routeColor"].get<std::string>();
      vehicle->width = 2.0 * vehicle->length / 4;
      // vehicle->run();
      dispatcher->vehicles.push_back(vehicle);
    }
    // TODO blocks

    // for (auto &v : model["blocks"]) {
    // }
    //
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res.connect(
        std::bind(&kernel::allocate::ResourceManager::find, resource,
                  std::placeholders::_1));
    dispatcher->go_home.connect(std::bind(
        &TCS::home_order, this, std::placeholders::_1, std::placeholders::_2));
    dispatcher->get_next_ord.connect(
        std::bind(&kernel::allocate::OrderPool::pop, orderpool));
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
#ifdef VISUAL
    if (!visualizer) {
      init_visualizer();
    } else {
      visualizer->init(visualizer->resolution, shared_from_this());
    }
#endif
    run();
    return std::pair<int, std::string>(200, "");
  } catch (json::parse_error ec) {
    LOG(ERROR) << "parse error: " << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::type_error ec) {
    LOG(ERROR) << "type error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::other_error ec) {
    LOG(ERROR) << "other error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  }
}

std::pair<int, std::string> TCS::put_path_locked(const std::string &path_name,
                                                 bool new_value) {
  for (auto &x : resource->paths) {
    if (x->name == path_name) {
      x->locked = new_value;
      if (new_value) {
        planner->set_barrier_edge(x->name);
      } else {
        planner->reset_edge(x->name);
      }
#ifdef VISUAL
      visualizer->init(visualizer->resolution, shared_from_this());
#endif
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find path '" + path_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

std::pair<int, std::string> TCS::put_location_locked(
    const std::string &loc_name, bool new_value) {
  for (auto &x : resource->locations) {
    if (x->name == loc_name) {
      x->locked = new_value;
#ifdef VISUAL
      visualizer->init(visualizer->resolution, shared_from_this());
#endif
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find location  '" + loc_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}