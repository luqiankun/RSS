#include "../../include/main/tcs.hpp"

#include "../../include/kernel/driver/vehicle.hpp"

std::pair<int, std::string> TCS::put_model_xml(const std::string &body) {
  stop();
  init_orderpool();
  init_planner();
  init_scheduler();
  init_dispatcher();
  this->resource =
      std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
  pugi::xml_document doc;
  try {
    auto ret = doc.load_string(body.c_str());
    if (ret.status != pugi::status_ok) {
      CLOG(ERROR, tcs_log) << "parse error: " << ret.description();
      json res = json::array();
      auto msg =
          "Could not parse XML input '" + std::string(ret.description()) + "'.";
      res.push_back(msg);
      return std::pair<int, std::string>(400, res.dump());
    }
    //
    auto root = doc.first_child();  // <model>
    if (std::string(root.name()) != "model") {
      CLOG(ERROR, tcs_log) << "parse error: "
                           << "'don't has model'";
      json res = json::array();
      auto msg =
          "Could not parse XML input '" + std::string("'don't has model'.");
      res.push_back(msg);
      return std::pair<int, std::string>(400, res.dump());
    }
    { resource->model_name = root.attribute("name").as_string(); }
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
        layout.label_offset = data::Vector2i(xLabelOffset, yLabelOffset);
        layout.position = data::Vector2i(xPosition_layout, yPosition_layout);
        auto p = std::make_shared<data::model::Point>(name);
        p->position.x = xPosition;
        p->position.y = yPosition;
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
      CLOG(INFO, tcs_log) << "init point size " << resource->points.size();
    }

    {  // path
      auto path = root.find_child([](pugi::xml_node node) {
        return std::string(node.name()) == "path";
      });
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
        // pro
        auto property = path.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "property ";
        });
        while (property.type() != pugi::node_null) {
          if (std::string(property.name()) != "property ") {
            break;
          }
          auto name_ = property.attribute("name").as_string();
          auto vlaue_ = property.attribute("value").as_string();
          p->properties.insert(
              std::pair<std::string, std::string>(name_, vlaue_));
          property = property.next_sibling();
        }
        //
        resource->paths.push_back(p);
        path = path.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init path size " << resource->paths.size();
    }
    {
      // locatintype
      auto loc_type = root.find_child([](pugi::xml_node node) {
        return std::string(node.name()) == "locationType";
      });
      while (loc_type.type() != pugi::node_null) {
        if (std::string(loc_type.name()) != "locationType") {
          break;
        }
        auto name = loc_type.attribute("name").as_string();
        auto lt = std::make_shared<data::model::LocationType>(name);
        // allow
        auto allow_ops = loc_type.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "allowedOperation";
        });
        while (allow_ops.type() != pugi::node_null) {
          if (std::string(allow_ops.name()) != "property") {
            break;
          }
          auto name_ = allow_ops.attribute("name").as_string();
          lt->allowrd_ops.push_back(name_);
          allow_ops = allow_ops.next_sibling();
        }
        auto allow_per_ops = loc_type.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "allowedOperation";
        });
        while (allow_per_ops.type() != pugi::node_null) {
          if (std::string(allow_per_ops.name()) != "property") {
            break;
          }
          auto name_ = allow_per_ops.attribute("name").as_string();
          lt->allowrd_per_ops.push_back(name_);
          allow_per_ops = allow_per_ops.next_sibling();
        }
        // pro
        auto property = loc_type.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "property";
        });
        while (property.type() != pugi::node_null) {
          if (std::string(property.name()) != "property") {
            break;
          }
          auto name_ = property.attribute("name").as_string();
          auto vlaue_ = property.attribute("value").as_string();
          lt->properties.insert(
              std::pair<std::string, std::string>(name_, vlaue_));
          property = property.next_sibling();
        }
        //
        auto loc_layout = loc_type.child("locationTypeLayout")
                              .attribute("locationRepresentation")
                              .as_string();
        lt->layout.location_representation =
            data::model::new_location_type(loc_layout);

        //
        resource->location_types.push_back(lt);
        loc_type = loc_type.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init loc_type size "
                          << resource->location_types.size();
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
        layout.position = data::Vector2i(xPosition_layout, yPosition_layout);
        layout.label_offset = data::Vector2i(xLabelOffset, yLabelOffset);
        layout.layer_id = layerId;

        auto loc = std::make_shared<data::model::Location>(name);
        std::weak_ptr<data::model::Point> link;
        for (auto &p : resource->points) {
          if (p->name == link_point) {
            link = p;
            p->attached_links.push_back(loc);
          }
        }

        for (auto &t : resource->location_types) {
          if (t->name == type) {
            loc->type = t;
          }
        }

        loc->position = data::Vector3i(xPosition, yPosition, zPosition);
        loc->layout = layout;
        loc->link = link;
        loc->locked = locked;
        auto property = location.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "property";
        });
        while (property.type() != pugi::node_null) {
          if (std::string(property.name()) != "property") {
            break;
          }
          auto name_ = property.attribute("name").as_string();
          auto vlaue_ = property.attribute("value").as_string();
          loc->properties.insert(
              std::pair<std::string, std::string>(name_, vlaue_));
          property = property.next_sibling();
        }
        resource->locations.push_back(loc);
        location = location.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init location size "
                          << resource->locations.size();
    }

    {
      // block
      auto block = root.find_child([](pugi::xml_node node) -> bool {
        return std::string(node.name()) == "block";
      });
      while (block.type() != pugi::node_null) {
        if (std::string(block.name()) != "block") {
          break;
        }
        auto color = block.child("blockLayout").attribute("color").as_string();
        // member
        std::unordered_set<std::shared_ptr<TCSResource>> rs;
        auto member = block.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "member";
        });
        while (member.type() != pugi::node_null) {
          if (std::string(member.name()) != "member") {
            break;
          }
          for (auto &x : resource->points) {
            if (x->name == member.attribute("name").as_string()) {
              rs.insert(x);
            }
          }
          for (auto &x : resource->paths) {
            if (x->name == member.attribute("name").as_string()) {
              rs.insert(x);
            }
          }
          for (auto &x : resource->locations) {
            if (x->name == member.attribute("name").as_string()) {
              rs.insert(x);
            }
          }
          member = member.next_sibling();
        }
        if (block.attribute("type").as_string() ==
            std::string("SINGLE_VEHICLE_ONLY")) {
          auto rule = std::make_shared<kernel::allocate::OnlyOneGatherRule>(
              block.attribute("name").as_string(), resource);
          rule->occs = rs;
          rule->color = color;
          resource->rules.push_back(rule);
        } else if (block.attribute("type").as_string() ==
                   std::string("SAME_DIRECTION_ONLY")) {
          std::string direction =
              block.child("direct").attribute("value").as_string();
          for (auto &x : resource->paths) {
            auto p = rs.find(x);
            if (p != rs.end()) {
              auto path = std::dynamic_pointer_cast<data::model::Path>(*p);
              if (direction == "FRONT") {
                path->max_reverse_vel = 0;
              } else if (direction == "BACK") {
                path->max_vel = 0;
              }
            }
          }
        }
        block = block.next_sibling();
      }
    }

    //
    {
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
        int maxReverseVelocity =
            vehicle.attribute("maxReverseVelocity").as_int();
        std::string color =
            vehicle.child("vehicleLayout").attribute("color").as_string();
        int energyLevelCritical =
            vehicle.attribute("energyLevelCritical").as_int();
        int energyLevelGood = vehicle.attribute("energyLevelGood").as_int();
        int energyLevelFullyRecharged =
            vehicle.attribute("energyLevelFullyRecharged").as_int();
        int energyLevelSufficientlyRecharged =
            vehicle.attribute("energyLevelSufficientlyRecharged").as_int();
        std::string init_p_name =
            vehicle.child("initial_point").attribute("point").as_string();

        auto interfaceName =
            vehicle
                .find_child([](pugi::xml_node node) {
                  return (std::string(node.name()) == "property" &&
                          node.attribute("name").as_string() ==
                              std::string("vda5050:interfaceName"));
                })
                .attribute("value")
                .as_string();
        auto manufacturer =
            vehicle
                .find_child([](pugi::xml_node node) {
                  return (std::string(node.name()) == "property" &&
                          node.attribute("name").as_string() ==
                              std::string("vda5050:manufacturer"));
                })
                .attribute("value")
                .as_string();
        auto serialNumber =
            vehicle
                .find_child([](pugi::xml_node node) {
                  return (std::string(node.name()) == "property" &&
                          node.attribute("name").as_string() ==
                              std::string("vda5050:serialNumber"));
                })
                .attribute("value")
                .as_string();
        auto version = vehicle
                           .find_child([](pugi::xml_node node) {
                             return (std::string(node.name()) == "property" &&
                                     node.attribute("name").as_string() ==
                                         std::string("vda5050:version"));
                           })
                           .attribute("value")
                           .as_string();
        auto ip = vehicle
                      .find_child([](pugi::xml_node node) {
                        return (std::string(node.name()) == "property" &&
                                node.attribute("name").as_string() ==
                                    std::string("vda5050:ip"));
                      })
                      .attribute("value")
                      .as_string();
        auto port = vehicle
                        .find_child([](pugi::xml_node node) {
                          return (std::string(node.name()) == "property" &&
                                  node.attribute("name").as_string() ==
                                      std::string("vda5050:port"));
                        })
                        .attribute("value")
                        .as_string();
        ///////////////////
        /// // 使用mqtt车辆
        //////////////////
        auto veh = std::make_shared<kernel::driver::Rabbit3>(
            name, interfaceName, serialNumber, version, manufacturer);
        veh->length = length;
        veh->width = 2.0 * length / 4;
        veh->max_reverse_vel = maxReverseVelocity;
        veh->max_vel = maxVelocity;
        veh->color = color;
        veh->energy_level_critical = energyLevelCritical;
        veh->energy_level_good = energyLevelGood;
        veh->engrgy_level_full = energyLevelFullyRecharged;
        veh->engrgy_level_recharge = energyLevelSufficientlyRecharged;
        veh->map_id = root.attribute("name").as_string();
        veh->broker_ip = ip;
        veh->broker_port = std::stoi(port);
        ///////////////////////////
        dispatcher->vehicles.push_back(veh);
        ///////////////////////
        vehicle = vehicle.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init vehicle size "
                          << dispatcher->vehicles.size();
    }
    ///
    CLOG(INFO, tcs_log) << "init resource ok";
    init_planner();
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res = std::bind(&kernel::allocate::ResourceManager::find,
                                     resource, std::placeholders::_1);
    dispatcher->go_home = std::bind(
        &TCS::home_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->get_next_ord =
        std::bind(&kernel::allocate::OrderPool::pop, orderpool);
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
    run();
    return std::pair<int, std::string>(200, "");
  } catch (pugi::xpath_exception ec) {
    CLOG(ERROR, tcs_log) << "parse error: " << ec.what();
    json res = json::array();
    auto msg = "Could not parse XML input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  }
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
  is_run = false;
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
  CLOG(INFO, tcs_log) << "new ord " << ord->name << " name_hash "
                      << ord->name_hash << "\n";
  auto destination = orderpool->res_to_destination(
      resource->get_recent_park_point(v->current_point),
      data::order::DriverOrder::Destination::OpType::MOVE);
  auto dr =
      std::make_shared<data::order::DriverOrder>("driverorder_home_" + v->name);
  dr->destination = destination;
  dr->transport_order = ord;
  ord->driverorders.push_back(dr);
  ord->intended_vehicle = v;
  orderpool->orderpool.push_back(ord);
}

TCS::~TCS() {
  CLOG(INFO, tcs_log) << "TCS  stop";
  stop();
}

bool TCS::init_dispatcher() {
  dispatcher = std::make_shared<kernel::dispatch::Dispatcher>("Dispatcher");
  CLOG(INFO, tcs_log) << "init dispatcher ok";
  return true;
}
bool TCS::init_orderpool() {
  orderpool = std::make_shared<kernel::allocate::OrderPool>("OrderPool");
  CLOG(INFO, tcs_log) << "init orderpool ok";
  return true;
}
bool TCS::init_scheduler() {
  this->scheduler = std::make_shared<kernel::schedule::Scheduler>("Scheduler");
  CLOG(INFO, tcs_log) << "init scheduler ok";
  return true;
}
bool TCS::init_planner() {
  if (!resource) {
    return false;
  }
  this->planner = std::make_shared<kernel::planner::Planner>(this->resource);
  CLOG(INFO, tcs_log) << "init planner ok";
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
  is_run = true;
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &o : orderpool->orderpool) {
    if (o->name == ord_name) {
      json res = json::array();
      auto msg = "Transport order '" + ord_name + "' already exists.";
      res.push_back(msg);
      CLOG(ERROR, tcs_log) << msg;
      return std::pair<int, std::string>(409, res.dump());
    }
  }
  for (auto &o : orderpool->ended_orderpool) {
    if (o->name == ord_name) {
      json res = json::array();
      auto msg = "Transport order '" + ord_name + "' already exists.";
      res.push_back(msg);
      CLOG(ERROR, tcs_log) << msg;
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
          CLOG(ERROR, tcs_log) << msg;
          ord->state = data::order::TransportOrder::State::UNROUTABLE;
          orderpool->ended_orderpool.push_back(ord);
          return std::pair<int, std::string>(404, res.dump());
        } else {
          auto op = d["operation"].get<std::string>();
          auto op_ = data::order::DriverOrder::Destination::get_optype(op);
          if (op_.has_value()) {
            if (op_.value() ==
                    data::order::DriverOrder::Destination::OpType::LOAD ||
                op_.value() ==
                    data::order::DriverOrder::Destination::OpType::UNLOAD) {
              if (check.first !=
                  kernel::allocate::ResourceManager::ResType::Location) {
                CLOG(ERROR, tcs_log) << "op type '" + std::string(op) +
                                            "' is not support,need a location";
                ord->state = data::order::TransportOrder::State::FAILED;
                orderpool->ended_orderpool.push_back(ord);
                break;
              }
            }
            auto destination =
                orderpool->res_to_destination(check.second, op_.value());
            auto dr = std::make_shared<data::order::DriverOrder>(
                "driverorder_" + loc);
            dr->destination = destination;
            dr->transport_order = ord;
            ord->driverorders.push_back(dr);
          } else {
            CLOG(ERROR, tcs_log)
                << "op type '" + std::string(op) + "' is not support";
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
    CLOG(ERROR, tcs_log) << "parse failed :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::type_error ec) {
    CLOG(ERROR, tcs_log) << "type error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::other_error ec) {
    CLOG(ERROR, tcs_log) << "other error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  }
}

std::pair<int, std::string> TCS::post_transport_order_withdrawl(
    const std::string &ord_name, bool immediate, bool) {
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
        auto new_orderquence =
            std::make_shared<data::order::OrderSequence>(sequence_name);
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
    CLOG(ERROR, tcs_log) << "parse error:" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::type_error ec) {
    CLOG(ERROR, tcs_log) << "type error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::other_error ec) {
    CLOG(ERROR, tcs_log) << "other error :" << ec.what();
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
  res["transportOrder"] = v->current_order ? v->current_order->name : "";
  res["currentPosition"] = v->current_point ? v->current_point->name : "";
  res["precisePosition"]["x"] = v->position.x;
  res["precisePosition"]["y"] = v->position.y;
  res["precisePosition"]["z"] = v->position.z;
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  if (state.empty()) {
    json res = json::array();
    for (auto &v : dispatcher->vehicles) {
      res.push_back(vehicle_to_json(v));
    }
    return std::pair<int, std::string>(200, res.dump());
  } else {
    if (state == "IDLE") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::IDLE) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state == "ERROR") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::ERROR) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state == "EXECUTING") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::EXECUTING) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state == "UNKNOWN") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::UNKNOWN) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state == "UNAVAILABLE") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::UNAVAILABLE) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    } else if (state == "CHARGING") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::CHARGING) {
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == vehicle) {
      if (v->current_order) {
        v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      }
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

std::pair<int, std::string> TCS::put_vehicle_paused(const std::string &name,
                                                    bool p) {
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &x : dispatcher->vehicles) {
    if (x->name == name) {
      //
      // x->paused = p;
      auto paused_task = std::make_shared<vda5050::instantaction::Action>();
      paused_task->action_description = "set paused action";
      if (p) {
        paused_task->action_type =
            vda5050::instantaction::ActionType::startPause;
      } else {
        paused_task->action_type =
            vda5050::instantaction::ActionType::stopPause;
      }
      paused_task->blocking_type =
          vda5050::instantaction::ActionBlockingType::HARD;
      paused_task->action_id = name + "_set_pasued_<" + std::to_string(p) + ">";
      x->execute_instatn_action(paused_task);
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "no vehicle named '" + name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

std::pair<int, std::string> TCS::get_model() {
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  json res;
  res["name"] = resource->model_name;
  // point
  res["points"] = json::array();
  for (auto &p : resource->points) {
    json point;
    point["name"] = p->name;
    point["position"]["x"] = p->position.x;
    point["position"]["y"] = p->position.y;
    point["position"]["z"] = p->position.z;
    point["vehicleOrientationAngle"] = p->client_angle;
    point["type"] = data::model::Point::get_type(p->type);
    json layout;
    layout["position"]["x"] = p->layout.position.x;
    layout["position"]["y"] = p->layout.position.y;
    layout["labelOffset"]["x"] = p->layout.label_offset.x;
    layout["labelOffset"]["y"] = p->layout.label_offset.y;
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
  // location type
  res["locationTypes"] = json::array();
  for (auto &type : resource->location_types) {
    json loc_type;
    loc_type["typeName"] = type->name;
    loc_type["allowedOperation"] = json::array();
    loc_type["allowedPeripheralOperation"] = json::array();
    loc_type["properties"] = json::array();
    for (auto &x : type->allowrd_ops) {
      loc_type["allowedOperation"].push_back(x);
    }
    for (auto &x : type->allowrd_ops) {
      loc_type["allowedPeripheralOperation"].push_back(x);
    }
    for (auto &pro : type->properties) {
      json t;
      t["name"] = pro.first;
      t["value"] = pro.second;
      loc_type["properties"].push_back(t);
    }
    loc_type["layout"]["locationRepresentation"] =
        data::model::get_Representation(type->layout.location_representation);
    res["locationTypes"].push_back(loc_type);
  }
  // location
  res["locations"] = json::array();
  for (auto &loc : resource->locations) {
    json location;
    location["name"] = loc->name;
    location["typeName"] = loc->type.lock()->name;  // TODO
    location["position"]["x"] = loc->position.x;
    location["position"]["y"] = loc->position.y;
    location["position"]["z"] = loc->position.z;
    location["links"] = json::array();
    location["links"].push_back(loc->link.lock() ? loc->link.lock()->name : "");
    location["locked"] = loc->locked;
    location["layout"]["position"]["x"] = loc->layout.position.x;
    location["layout"]["position"]["y"] = loc->layout.position.y;
    location["layout"]["labelOffset"]["x"] = loc->layout.label_offset.x;
    location["layout"]["labelOffset"]["y"] = loc->layout.label_offset.y;
    location["layout"]["locationRepresentation"] =
        data::model::get_Representation(
            loc->type.lock()->layout.location_representation);
    location["layout"]["layerId"] = loc->layout.layer_id;
    location["properties"] = json::array();
    for (auto &pro : loc->properties) {
      json t;
      t["name"] = pro.first;
      t["value"] = pro.second;
      location["properties"].push_back(t);
    }
    res["locations"].push_back(location);
  }
  // block
  res["blocks"] = json::array();

  for (auto &x : resource->rules) {
    json block;
    block["member"] = json::array();
    for (auto &p : x->occs) {
      json v;
      v["name"] = p->name;
      block["member"].push_back(v);
    }
    block["name"] = x->name;
    block["type"] = "SINGLE_VEHICLE_ONLY";
    block["blockLayout"]["color"] = x->color;
    res["blocks"].push_back(block);
  }
  //  vehicles
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
  init_scheduler();
  init_dispatcher();
  try {
    json model = json::parse(body);
    resource =
        std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
    if (model.contains("name")) {
      resource->model_name = model["name"].get<std::string>();
    }
    // point
    for (auto &p : model["points"]) {
      auto point =
          std::make_shared<data::model::Point>(p["name"].get<std::string>());
      point->position.x = p["position"]["x"].get<int>();
      point->position.y = p["position"]["y"].get<int>();
      point->position.z = p["position"]["z"].get<int>();
      if (p.contains("vehicleOrientationAngle")) {
        point->client_angle = p["vehicleOrientationAngle"].get<int>();
      }
      point->type = data::model::Point::new_type(p["type"].get<std::string>());
      point->layout.position.x = p["layout"]["position"]["x"].get<int>();
      point->layout.position.y = p["layout"]["position"]["y"].get<int>();
      point->layout.label_offset.x = p["layout"]["labelOffset"]["x"].get<int>();
      point->layout.label_offset.y = p["layout"]["labelOffset"]["y"].get<int>();
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

    for (auto &x : model["locationTypes"]) {
      auto type = std::make_shared<data::model::LocationType>(
          x["name"].get<std::string>());
      type->layout.location_representation = data::model::new_location_type(
          x["layout"]["locationRepresentation"].get<std::string>());
      for (auto &pro : x["property "]) {
        auto key = pro["name"].get<std::string>();
        auto value = pro["value"].get<std::string>();
        type->properties.insert(
            std::pair<std::string, std::string>(key, value));
      }
      for (auto &allow : x["allowedOperation"]) {
        type->allowrd_ops.push_back(allow);
      }
      for (auto &allow : x["allowedPeripheralOperation"]) {
        type->allowrd_per_ops.push_back(allow);
      }
      resource->location_types.push_back(type);
    }

    // location
    for (auto &l : model["locations"]) {
      auto loc =
          std::make_shared<data::model::Location>(l["name"].get<std::string>());
      loc->position.x = l["position"]["x"].get<int>();
      loc->position.y = l["position"]["y"].get<int>();
      loc->position.z = l["position"]["z"].get<int>();
      if (!l["links"].empty()) {
        auto p_name = l["links"].front()["pointName"].get<std::string>();
        for (auto &x : resource->points) {
          if (x->name == p_name) {
            loc->link = x;
          }
        }
      }
      loc->locked = l["locked"].get<bool>();
      loc->layout.position.x = l["layout"]["position"]["x"].get<int>();
      loc->layout.position.y = l["layout"]["position"]["y"].get<int>();
      loc->layout.label_offset.x = l["layout"]["labelOffset"]["x"].get<int>();
      loc->layout.label_offset.y = l["layout"]["labelOffset"]["y"].get<int>();
      loc->layout.layer_id = l["layout"]["layerId"].get<int>();
      for (auto &x : resource->location_types) {
        if (x->name == l["typeName"].get<std::string>()) {
          loc->type = x;
        }
      }
      // if (l["layout"].contains("locationRepresentation")) {
      //   loc->type.layout.location_representation =
      //       data::model::Location::new_location_type(
      //           l["layout"]["locationRepresentation"].get<std::string>());
      // }
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
      //////////仿真 假定在第一个点
      vehicle->position.x = resource->points.front()->position.x;
      vehicle->position.y = resource->points.front()->position.y;
      vehicle->current_point = resource->points.front();  // 当前点
      ///////////////////////////
      dispatcher->vehicles.push_back(vehicle);
    }
    // TODO blocks

    // for (auto &v : model["blocks"]) {
    // }
    //
    CLOG(INFO, tcs_log) << "init resource ok";
    init_planner();
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res = std::bind(&kernel::allocate::ResourceManager::find,
                                     resource, std::placeholders::_1);
    dispatcher->go_home = std::bind(
        &TCS::home_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->get_next_ord =
        std::bind(&kernel::allocate::OrderPool::pop, orderpool);
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
    run();
    return std::pair<int, std::string>(200, "");
  } catch (json::parse_error ec) {
    CLOG(ERROR, tcs_log) << "parse error: " << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::type_error ec) {
    CLOG(ERROR, tcs_log) << "type error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  } catch (json::other_error ec) {
    CLOG(ERROR, tcs_log) << "other error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(400, res.dump());
  }
}

std::pair<int, std::string> TCS::get_view() {
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  auto r = get_vehicles_step();
  return std::pair<int, std::string>(200, r);
}

std::string TCS::get_vehicles_step() {
  using json = nlohmann::json;
  json value = json::array();
  for (auto &v : dispatcher->vehicles) {
    json veh;
    veh["name"] = v->name;
    veh["color"] = v->color;
    veh["step"] = json::array();
    veh["position"]["x"] = v->position.x;
    veh["position"]["y"] = v->position.y;
    if (v->current_order) {
      if (v->current_order->state ==
          data::order::TransportOrder::State::BEING_PROCESSED) {
        for (auto &dr : v->current_order->driverorders) {
          if (dr->state == data::order::DriverOrder::State::TRAVELLING ||
              dr->state == data::order::DriverOrder::State::OPERATING) {
            // current
            for (auto &step_ : dr->route->current_steps) {
              auto beg = step_->path->source_point.lock();
              auto end = step_->path->destination_point.lock();
              json step;
              step["src"] = beg->name;
              step["dest"] = end->name;
              if (step_->vehicle_orientation ==
                  data::order::Step::Orientation::FORWARD) {
                step["orientation"] = "FORWARD";
              } else if (step_->vehicle_orientation ==
                         data::order::Step::Orientation::BACKWARD) {
                step["orientation"] = "BACKWARD";
              } else {
                step["orientation"] = "UNDEFINED";
              }
              veh["step"].push_back(step);
            }
            // other
            for (auto &path : dr->route->steps) {
              auto beg = path->path->source_point.lock();
              auto end = path->path->destination_point.lock();
              json step;
              step["src"] = beg->name;
              step["dest"] = end->name;
              if (path->vehicle_orientation ==
                  data::order::Step::Orientation::FORWARD) {
                step["orientation"] = "FORWARD";
              } else if (path->vehicle_orientation ==
                         data::order::Step::Orientation::BACKWARD) {
                step["orientation"] = "BACKWARD";
              } else {
                step["orientation"] = "UNDEFINED";
              }
              veh["step"].push_back(step);
            }
            // location
            if (dr->destination->operation ==
                data::order::DriverOrder::Destination::OpType::NOP) {
              // TODO
            } else if (dr->destination->operation ==
                       data::order::DriverOrder::Destination::OpType::LOAD) {
              veh["destination"]["op"] = "LOAD";
              auto loction = std::dynamic_pointer_cast<data::model::Location>(
                  dr->destination->destination.lock());
              veh["destination"]["dest"] = loction->name;
            } else if (dr->destination->operation ==
                       data::order::DriverOrder::Destination::OpType::UNLOAD) {
              veh["destination"]["op"] = "UNLOAD";
              auto loction = std::dynamic_pointer_cast<data::model::Location>(
                  dr->destination->destination.lock());
              veh["destination"]["dest"] = loction->name;
            }
          }
        }
      }
    }
    value.push_back(veh);
  }
  return value.dump();
}

std::pair<int, std::string> TCS::put_path_locked(const std::string &path_name,
                                                 bool new_value) {
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &x : resource->paths) {
    if (x->name == path_name) {
      x->locked = new_value;
      if (new_value) {
        planner->set_barrier_edge(x->name);
      } else {
        planner->reset_edge(x->name);
      }
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
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &x : resource->locations) {
    if (x->name == loc_name) {
      x->locked = new_value;
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find location  '" + loc_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}