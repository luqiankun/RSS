#include "../../include/main/tcs.hpp"

#include "../../include/kernel/driver/vehicle.hpp"

bool TCS::init_resource(const std::string &xml_path) {
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
      if (connectionType == "DIRECT") {
        layout.connect_type = data::model::Path::ConnectType::DIRECT;
      } else if (connectionType == "ELBOW") {
        layout.connect_type = data::model::Path::ConnectType::ELBOW;

      } else if (connectionType == "SLANTED") {
        layout.connect_type = data::model::Path::ConnectType::SLANTED;

      } else if (connectionType == "POLYPATH") {
        layout.connect_type = data::model::Path::ConnectType::POLYPATH;

      } else if (connectionType == "BEZIER") {
        layout.connect_type = data::model::Path::ConnectType::BEZIER;

      } else if (connectionType == "BEZIER_3") {
        layout.connect_type = data::model::Path::ConnectType::BEZIER_3;
      }
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
    ///////////////////
    /// // 使用仿真车辆
    //////////////////
    auto veh = std::make_shared<kernel::driver::SimVehicle>(name);
    veh->length = length;
    veh->width = 2.0 * length / 4;
    veh->max_reverse_vel = maxReverseVelocity;
    veh->max_vel = maxVelocity;
    veh->color = color;
    veh->position.x() = resource->points.front()->position.x();
    veh->position.y() = resource->points.front()->position.y();
    veh->layout = veh->layout;
    veh->current_point = resource->points.front();  // 当前点
    veh->init_point = resource->points.front();     // 初始点
    std::hash<std::string> hash_fn;
    veh->name_hash = hash_fn(name);
    veh->run();
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
#ifdef VISUAL
  if (visualizer) {
    visualizer->stop();
    visualizer.reset();
  }
#endif
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
void TCS::add_task(std::vector<Oper> ops, const std::string &name,
                   const std::string &v_name) {
  auto time = std::chrono::system_clock::now();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>(name);
  std::hash<std::string> hash_fn;
  ord->name_hash = hash_fn(name);
  ord->create_time = time;
  ord->state = data::order::TransportOrder::State::RAW;
  LOG(INFO) << "new ord " << ord->name << " name_hash " << ord->name_hash;
  if (ops.empty()) {
    LOG(WARNING) << ord->name << " op is null";
    ord->state = data::order::TransportOrder::State::FAILED;
    orderpool->ended_orderpool.push_back(ord);
  } else {
    for (auto &op : ops) {
      auto dest = std::get<0>(op);
      auto op_type = std::get<1>(op);
      auto dest_check = resource->find(dest);
      if (dest_check.first == kernel::allocate::ResourceManager::ResType::Err) {
        ord->state = data::order::TransportOrder::State::UNROUTABLE;
        orderpool->ended_orderpool.push_back(ord);
        return;
      }
      auto destination =
          orderpool->res_to_destination(dest_check.second, op_type);
      auto dr =
          std::make_shared<data::order::DriverOrder>("driverorder_" + dest);
      dr->destination = destination;
      dr->transport_order = ord;
      ord->driverorders.push_back(dr);
    }
    if (v_name != "none") {
      for (auto &v : dispatcher->vehicles) {
        if (v->name_hash == hash_fn(v_name)) {
          ord->intended_vehicle = v;
        }
      }
    }
    orderpool->orderpool.push_back(ord);
  }
}

TCS::~TCS() {
  LOG(INFO) << "TCS  stop";
#ifdef VISUAL
  visualizer->stop();
#endif
  dispatcher->stop();
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
  dispatcher->add_task.connect(
      std::bind(&TCS::add_task, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
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
  dispatcher->run();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  scheduler->run();
#ifdef VISUAL
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  visualizer->run();
#endif
}