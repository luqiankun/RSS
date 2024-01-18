#include "../../include/main/tcs.hpp"

#include "../../include/kernel/driver/vehicle.hpp"

bool TCS::init_resource(const std::string &xml_path) {
  this->resource =
      std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
  scheduler->res = resource;  // scheduler 要先初始化
  dispatcher->resource = resource;
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
      auto p = std::make_shared<data::model::Point>(name);
      p->pose.x() = xPosition;
      p->pose.y() = yPosition;
      p->layout.x() = xPosition_layout;
      p->layout.y() = yPosition_layout;
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
      auto p = std::make_shared<data::model::Path>(name);
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
      auto loc = std::make_shared<data::model::Location>(name);
      std::weak_ptr<data::model::Point> link;
      for (auto &p : resource->points) {
        if (p->name == link_point) {
          link = p;
          p->attached_links.push_back(loc);
        }
      }
      loc->position = Eigen::Vector3i(xPosition, yPosition, zPosition);
      loc->layout = Eigen::Vector2i(xPosition_layout, yPosition_layout);
      loc->link = link;
      loc->locked = locked;
      resource->locations.push_back(loc);
      location = location.next_sibling();
    }
    LOG(INFO) << "init location size " << resource->locations.size();
  }

  //
  dispatcher->planner = std::make_shared<kernel::planner::Planner>(resource);
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
    auto veh = std::make_shared<kernel::driver::SimVehicle>(name);
    veh->scheduler = scheduler;
    veh->length = length;
    veh->width = 2.0 * length / 4;
    veh->max_reverse_vel = maxReverseVelocity;
    veh->max_vel = maxVelocity;
    veh->color = color;
    veh->position.x() = resource->points.front()->pose.x();
    veh->position.y() = resource->points.front()->pose.y();
    veh->layout = veh->layout;
    veh->current_point = resource->points.front();
    std::hash<std::string> hash_fn;
    veh->uuid = hash_fn(name);
    veh->run();
    dispatcher->vehicles.push_back(veh);
    vehicle = vehicle.next_sibling();
  }
  LOG(INFO) << "init vehicle size " << dispatcher->vehicles.size();

  ///
  LOG(INFO) << "init resource ok";
  return true;
}

bool TCS::init_dispatcher() {
  dispatcher = std::make_shared<kernel::dispatch::Dispatcher>("Dispatcher");
  dispatcher->orderpool =
      std::make_shared<kernel::allocate::OrderPool>("OrderPool");
  LOG(INFO) << "init dispatcher ok";
  return true;
}
bool TCS::init_scheduler() {
  this->scheduler = std::make_shared<kernel::schedule::Scheduler>("Scheduler");
  LOG(INFO) << "init scheduler ok";
  return true;
}
#ifdef VISUAL
bool TCS::init_visualizer(double resolution) {
  visualizer = std::make_shared<visual::Visualizer>();
  return visualizer->init(resolution, shared_from_this());
}
#endif
bool TCS::init_all(const std::string &xml_path, double r) {
  if (!init_dispatcher()) {
    return false;
  }
  if (!init_scheduler()) {
    return false;
  }
  if (!init_resource(xml_path)) {
    return false;
  }
#ifdef VISUAL
  if (!init_visualizer(r)) {
    return false;
  }
#endif
  return true;
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