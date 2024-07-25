#include "../../include/main/tcs.hpp"

#include "../../include/kernel/driver/vehicle.hpp"

std::pair<int, std::string> TCS::put_model_xml(const std::string &body) {
  std::unique_lock<std::shared_mutex> lock(mutex);
  // TODO 判断是否有正在执行的订单
  stop();
  init_orderpool();
  init_planner();
  init_scheduler();
  init_dispatcher();
  this->resource =
      std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
  this->resource->is_connected = std::bind(
      &TCS::is_connect, this, std::placeholders::_1, std::placeholders::_2);
  auto control_rule =
      std::make_shared<kernel::allocate::OwnerRule>("control_rule", resource);
  auto envelope_rule = std::make_shared<kernel::allocate::CollisionRule>(
      "collision_rule", resource);
  resource->rules.push_back(control_rule);
  resource->rules.push_back(envelope_rule);
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
    {
      // visulize
      auto visual_layout = root.find_child([](pugi::xml_node node) {
        return std::string(node.name()) == "visualLayout";
      });
      while (visual_layout.type() != pugi::node_null) {
        if (std::string(visual_layout.name()) != "visualLayout") {
          break;
        }
        auto name = visual_layout.attribute("name").as_string();
        double scale_x = visual_layout.attribute("scaleX").as_double();
        double scale_y = visual_layout.attribute("scaleY").as_double();
        auto visuallayout = std::make_shared<data::model::VisualLayout>(name);
        visuallayout->scale_x = scale_x;
        visuallayout->scale_y = scale_y;
        auto layer = visual_layout.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "layer";
        });
        while (layer.type() != pugi::node_null) {
          if (std::string(layer.name()) != "layer") {
            break;
          }
          auto layer_id = layer.attribute("id").as_int();
          auto ordinal = layer.attribute("ordinal").as_int();
          auto name = layer.attribute("name").as_string();
          auto group_id = layer.attribute("groupId").as_int();
          auto layer_visible = layer.attribute("visible").as_bool();
          auto ly = data::model::VisualLayout::Layer();
          ly.id = layer_id;
          ly.ordinal = ordinal;
          ly.name = name;
          ly.group_id = group_id;
          ly.visible = layer_visible;
          visuallayout->layers.push_back(ly);
          layer = layer.next_sibling();
        }

        auto layer_group = visual_layout.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "layerGroup";
        });
        while (layer_group.type() != pugi::node_null) {
          if (std::string(layer_group.name()) != "layerGroup") {
            break;
          }
          auto id = layer_group.attribute("id").as_int();
          auto name = layer_group.attribute("name").as_string();
          auto visible = layer_group.attribute("visible").as_bool();
          auto layer_group_ = data::model::VisualLayout::LayerGroup();
          layer_group_.id = id;
          layer_group_.name = name;
          layer_group_.visible = visible;
          visuallayout->layer_groups.push_back(layer_group_);
          layer_group = layer_group.next_sibling();
        }
        resource->visual_layout = visuallayout;
        visual_layout = visual_layout.next_sibling();
      }
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
        auto vehicle_orientation =
            point.attribute("vehicleOrientation").as_double();
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
        p->type = data::model::Point::new_type(type);
        p->vehicle_orientation = vehicle_orientation;
        // envelope
        auto envelope = point.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "vehicleEnvelope";
        });
        while (envelope.type() != pugi::node_null) {
          if (std::string(envelope.name()) != "vehicleEnvelope") {
            break;
          }
          auto name = envelope.attribute("key").as_string();
          auto envl = std::make_shared<data::model::Envelope>(name);
          auto vertex = envelope.find_child([](pugi::xml_node node) {
            return std::string(node.name()) == "vertex";
          });
          while (vertex.type() != pugi::node_null) {
            if (std::string(vertex.name()) != "vertex") {
              break;
            }
            auto x = vertex.attribute("x").as_double();
            auto y = vertex.attribute("y").as_double();
            envl->add_vertex(x, y);
            vertex = vertex.next_sibling();
          }
          p->envelopes.insert(
              std::pair<std::string, std::shared_ptr<data::model::Envelope>>(
                  name, envl));
          envelope = envelope.next_sibling();
        }
        // pro
        auto property = point.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "property";
        });
        while (property.type() != pugi::node_null) {
          if (std::string(property.name()) != "property") {
            break;
          }
          auto name_ = property.attribute("name").as_string();
          auto vlaue_ = property.attribute("value").as_string();
          p->properties.insert(
              std::pair<std::string, std::string>(name_, vlaue_));
          property = property.next_sibling();
        }
        resource->points.push_back(p);
        point = point.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init point size " << resource->points.size();
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
          if (std::string(allow_ops.name()) != "allowedOperation") {
            break;
          }
          auto name_ = allow_ops.attribute("name").as_string();
          auto name_t = lt->allowed_ops[name_] =
              std::map<std::string, std::string>();
          allow_ops = allow_ops.next_sibling();
        }
        auto allow_per_ops = loc_type.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "allowedPeripheralOperation";
        });
        while (allow_per_ops.type() != pugi::node_null) {
          if (std::string(allow_per_ops.name()) !=
              "allowedPeripheralOperation") {
            break;
          }
          auto name_ = allow_per_ops.attribute("name").as_string();
          lt->allowrd_per_ops[name_] = std::map<std::string, std::string>();
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
        lt->get_param();
        // for (auto &p : lt->allowed_ops) {
        //   LOG(INFO) << p.first << " " << p.second.size();
        // }
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
        for (auto &p : resource->points) {
          if (p->name == link_point) {
            link = p;
            p->attached_links.push_back(loc);
          }
        }

        for (auto &t : resource->location_types) {
          if (t->name == type) {
            loc->type = t;
            for (auto &a_op : t->allowed_ops) {
              for (auto &per : loc->properties) {
                a_op.second.insert(per);
              }
            }
            for (auto &a_op : t->allowrd_per_ops) {
              for (auto &per : loc->properties) {
                a_op.second.insert(per);
              }
            }
          }
        }

        loc->position = data::Vector3i(xPosition, yPosition, zPosition);
        loc->layout = layout;
        loc->link = link;
        loc->locked = locked;

        resource->locations.push_back(loc);
        location = location.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init location size "
                          << resource->locations.size();
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
        //  xmllength不准
        p->length = Eigen::Vector2d(source_point.lock()->position.x -
                                        destination_point.lock()->position.x,
                                    source_point.lock()->position.y -
                                        destination_point.lock()->position.y)
                        .norm();
        p->max_vel = maxVelocity;
        p->max_reverse_vel = maxReverseVelocity;
        p->locked = locked;
        p->source_point.lock()->incoming_paths.push_back(p);
        p->destination_point.lock()->outgoing_paths.push_back(p);
        // envelope
        auto envelope = path.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "vehicleEnvelope";
        });
        while (envelope.type() != pugi::node_null) {
          if (std::string(envelope.name()) != "vehicleEnvelope") {
            break;
          }
          auto name = envelope.attribute("key").as_string();
          auto envl = std::make_shared<data::model::Envelope>(name);
          auto vertex = envelope.find_child([](pugi::xml_node node) {
            return std::string(node.name()) == "vertex";
          });
          while (vertex.type() != pugi::node_null) {
            if (std::string(vertex.name()) != "vertex") {
              break;
            }
            auto x = vertex.attribute("x").as_double();
            auto y = vertex.attribute("y").as_double();
            envl->add_vertex(x, y);
            vertex = vertex.next_sibling();
          }
          p->envelopes.insert(
              std::pair<std::string, std::shared_ptr<data::model::Envelope>>(
                  name, envl));
          envelope = envelope.next_sibling();
        }
        // pro
        auto property = path.find_child([](pugi::xml_node node) {
          return std::string(node.name()) == "property";
        });
        while (property.type() != pugi::node_null) {
          if (std::string(property.name()) != "property") {
            break;
          }
          auto name_ = property.attribute("name").as_string();
          auto vlaue_ = property.attribute("value").as_string();
          p->properties.insert(
              std::pair<std::string, std::string>(name_, vlaue_));
          property = property.next_sibling();
        }
        //
        data::model::Actions acts(p->properties);
        {
          //  perop
          auto peripher_op = path.find_child([](pugi::xml_node node) {
            return std::string(node.name()) == "peripheralOperation";
          });

          while (peripher_op.type() != pugi::node_null) {
            if (std::string(peripher_op.name()) != "peripheralOperation") {
              break;
            }
            auto per_op_name = peripher_op.attribute("name").as_string();
            auto wait = peripher_op.attribute("completionRequired").as_bool();
            auto when = peripher_op.attribute("executionTrigger").as_string();
            auto link_loc_name =
                peripher_op.attribute("locationName").as_string();
            for (auto &loc : resource->locations) {
              if (loc->name == link_loc_name) {
                if (loc->type.lock()->allowrd_per_ops.find(per_op_name) !=
                    loc->type.lock()->allowrd_per_ops.end()) {
                  // exist
                  data::model::PeripheralActions::PeripheralAction per_act;
                  per_act.completion_required = wait;
                  per_act.execution_trigger = when;
                  per_act.location_name = link_loc_name;
                  per_act.op_name = per_op_name;
                  p->per_acts.acts.push_back(per_act);
                  break;
                }
              }
            }
            peripher_op = peripher_op.next_sibling();
          }
        }
        p->acts = (acts);
        //
        resource->paths.push_back(p);
        path = path.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init path size " << resource->paths.size();
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
        std::string envelope = vehicle.attribute("envelopeKey").as_string();
        //
        auto init_pos = vehicle.find_child([](pugi::xml_node node) {
          return (std::string(node.name()) == "property" &&
                  node.attribute("name").as_string() ==
                      std::string("loopback:initialPosition"));
        });
        auto park_pos = vehicle.find_child([](pugi::xml_node node) {
          return (std::string(node.name()) == "property" &&
                  node.attribute("name").as_string() ==
                      std::string("tcs:preferredParkingPosition"));
        });
        auto adapter = vehicle.find_child([](pugi::xml_node node) {
          return (std::string(node.name()) == "property" &&
                  node.attribute("name").as_string() ==
                      std::string("tcs:preferredAdapterClass"));
        });
        if (adapter.type() == pugi::node_null) {
          vehicle = vehicle.next_sibling();
          continue;
        }
        std::string adapter_class = adapter.attribute("value").as_string();
        if (adapter_class.find("virtual") != std::string::npos) {
          ///////////////////
          /// // 使用虚拟车辆
          //////////////////
          auto veh = std::make_shared<kernel::driver::SimVehicle>(5, name);
          if (init_pos.type() != pugi::node_null) {
            auto init_pos_ = init_pos.attribute("value").as_string();
            auto p = resource->find(init_pos_);
            if (p.first == kernel::allocate::ResourceManager::ResType::Point) {
              veh->last_point =
                  std::dynamic_pointer_cast<data::model::Point>(p.second);
            }
          } else {
            CLOG(ERROR, tcs_log) << "vehicle " << name << " no init_pos";
            vehicle = vehicle.next_sibling();
            continue;
          }
          veh->length = length;
          veh->width = 2.0 * length / 4;
          veh->max_reverse_vel = maxReverseVelocity;
          veh->max_vel = maxVelocity;
          veh->color = color;
          veh->energy_level_critical = energyLevelCritical;
          veh->energy_level_good = energyLevelGood;
          veh->engrgy_level_full = energyLevelSufficientlyRecharged;
          veh->engrgy_level_recharge = energyLevelFullyRecharged;
          veh->send_queue_size = 2;
          veh->envelope_key = envelope;
          // pro
          auto property = vehicle.find_child([](pugi::xml_node node) {
            return std::string(node.name()) == "property";
          });
          while (property.type() != pugi::node_null) {
            if (std::string(property.name()) != "property") {
              break;
            }
            auto name_ = property.attribute("name").as_string();
            auto vlaue_ = property.attribute("value").as_string();
            veh->properties.insert(
                std::pair<std::string, std::string>(name_, vlaue_));
            property = property.next_sibling();
          }
          ///////////////////////////

          if (park_pos.type() != pugi::node_null) {
            auto park_pos_ = park_pos.attribute("value").as_string();
            auto p = resource->find(park_pos_);
            if (p.first == kernel::allocate::ResourceManager::ResType::Point) {
              veh->park_point =
                  std::dynamic_pointer_cast<data::model::Point>(p.second);
            }
          }
          dispatcher->vehicles.push_back(veh);

        } else if (adapter_class.find("vda") != std::string::npos) {
          //
          auto interfaceName = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:interfaceName"));
          });
          auto manufacturer = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:manufacturer"));
          });

          auto serialNumber = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:serialNumber"));
          });
          auto version = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:version"));
          });

          auto orderquence = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:orderQueueSize"));
          });

          ///////////////////
          /// // 使用mqtt车辆
          //////////////////
          auto vda_interfaceName =
              interfaceName.type() == pugi::node_null
                  ? "virtual"
                  : interfaceName.attribute("value").as_string();
          auto vda_serialNumber =
              serialNumber.type() == pugi::node_null
                  ? "virtual"
                  : serialNumber.attribute("value").as_string();
          auto vda_version = version.type() == pugi::node_null
                                 ? "1.0"
                                 : version.attribute("value").as_string();
          auto vda_manufacturer =
              manufacturer.type() == pugi::node_null
                  ? "virtual"
                  : manufacturer.attribute("value").as_string();
          auto veh = std::make_shared<kernel::driver::Rabbit3>(
              name, vda_interfaceName, vda_serialNumber, vda_version,
              vda_manufacturer);
          if (init_pos.type() != pugi::node_null) {
            auto init_pos_ = init_pos.attribute("value").as_string();
            auto p = resource->find(init_pos_);
            if (p.first == kernel::allocate::ResourceManager::ResType::Point) {
              veh->init_pos = true;
              veh->last_point =
                  std::dynamic_pointer_cast<data::model::Point>(p.second);
            }
          }
          if (park_pos.type() != pugi::node_null) {
            auto park_pos_ = park_pos.attribute("value").as_string();
            auto p = resource->find(park_pos_);
            if (p.first == kernel::allocate::ResourceManager::ResType::Point) {
              veh->park_point =
                  std::dynamic_pointer_cast<data::model::Point>(p.second);
            }
          }
          veh->length = length;
          veh->width = 2.0 * length / 4;
          veh->max_reverse_vel = maxReverseVelocity;
          veh->max_vel = maxVelocity;
          veh->color = color;
          veh->energy_level_critical = energyLevelCritical;
          veh->energy_level_good = energyLevelGood;
          veh->engrgy_level_full = energyLevelSufficientlyRecharged;
          veh->engrgy_level_recharge = energyLevelFullyRecharged;
          veh->map_id = root.attribute("name").as_string();
          veh->broker_ip = ip;
          veh->broker_port = port;
          veh->send_queue_size = orderquence.type() == pugi::node_null
                                     ? 2
                                     : orderquence.attribute("value").as_int();
          veh->envelope_key = envelope;

          // pro
          auto property = vehicle.find_child([](pugi::xml_node node) {
            return std::string(node.name()) == "property";
          });
          while (property.type() != pugi::node_null) {
            if (std::string(property.name()) != "property") {
              break;
            }
            auto name_ = property.attribute("name").as_string();
            auto vlaue_ = property.attribute("value").as_string();
            veh->properties.insert(
                std::pair<std::string, std::string>(name_, vlaue_));
            property = property.next_sibling();
          }
          ///////////////////////////
          dispatcher->vehicles.push_back(veh);
        } else {
          auto veh = std::make_shared<kernel::driver::InvalidVehicle>(name);
          veh->length = length;
          veh->width = 2.0 * length / 4;
          veh->max_reverse_vel = maxReverseVelocity;
          veh->max_vel = maxVelocity;
          veh->color = color;
          veh->energy_level_critical = energyLevelCritical;
          veh->energy_level_good = energyLevelGood;
          veh->engrgy_level_full = energyLevelSufficientlyRecharged;
          veh->engrgy_level_recharge = energyLevelFullyRecharged;
          veh->send_queue_size = 2;
          veh->envelope_key = envelope;
          // pro
          auto property = vehicle.find_child([](pugi::xml_node node) {
            return std::string(node.name()) == "property";
          });
          while (property.type() != pugi::node_null) {
            if (std::string(property.name()) != "property") {
              break;
            }
            auto name_ = property.attribute("name").as_string();
            auto vlaue_ = property.attribute("value").as_string();
            veh->properties.insert(
                std::pair<std::string, std::string>(name_, vlaue_));
            property = property.next_sibling();
          }
          dispatcher->vehicles.push_back(veh);
        }

        ///////////////////////
        vehicle = vehicle.next_sibling();
      }
      CLOG(INFO, tcs_log) << "init vehicle size "
                          << dispatcher->vehicles.size();
    }
    ///
    CLOG(INFO, tcs_log) << "init resource ok\n";
    init_planner();
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res = std::bind(&kernel::allocate::ResourceManager::find,
                                     resource, std::placeholders::_1);
    dispatcher->go_home = std::bind(
        &TCS::home_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->order_empty =
        std::bind(&kernel::allocate::OrderPool::is_empty, orderpool);
    dispatcher->go_charge = std::bind(
        &TCS::charge_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->get_next_ord =
        std::bind(&kernel::allocate::OrderPool::pop, orderpool);
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
    assert(dispatcher);
    CLOG(INFO, tcs_log) << "run all ...\n";
    run();
    CLOG(INFO, tcs_log) << "run all ok\n";
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
  if (orderpool) {
    orderpool.reset();
  }
  if (scheduler) {
    scheduler.reset();
  }
  if (dispatcher) {
    dispatcher->stop();
    dispatcher.reset();
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
  //  检查是否通路

  auto home_point = v->park_point
                        ? v->park_point
                        : resource->get_recent_park_point(v->last_point);
  if (!home_point) {
    CLOG(ERROR, tcs_log) << "home point not found";
    return;
  }
  auto destination = orderpool->res_to_destination(
      home_point, data::order::DriverOrder::Destination::OpType::MOVE);
  auto dr =
      std::make_shared<data::order::DriverOrder>("driverorder_home_" + v->name);
  dr->destination = destination;
  dr->transport_order = ord;
  ord->driverorders.push_back(dr);
  ord->intended_vehicle = v;
  ord->anytime_drop = true;
  orderpool->orderpool.push_back(ord);
}
void TCS::charge_order(const std::string &name,
                       std::shared_ptr<kernel::driver::Vehicle> v) {
  v->process_chargeing = true;
  auto time = std::chrono::system_clock::now();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>(name);
  std::hash<std::string> hash_fn;
  ord->create_time = time;
  ord->dead_time = time + std::chrono::minutes(60);
  ord->state = data::order::TransportOrder::State::RAW;
  CLOG(INFO, tcs_log) << "new ord " << ord->name << " name_hash "
                      << ord->name_hash << "\n";
  //  检查是否通路
  auto park_point = resource->get_recent_charge_loc(v->last_point);
  if (!park_point) {
    CLOG(ERROR, tcs_log) << "park point not found";
  }
  auto destination = orderpool->res_to_destination(
      park_point, data::order::DriverOrder::Destination::OpType::CHARGE);
  auto dr = std::make_shared<data::order::DriverOrder>("driverorder_charge_" +
                                                       v->name);
  dr->destination = destination;
  dr->transport_order = ord;
  ord->driverorders.push_back(dr);
  ord->intended_vehicle = v;
  orderpool->orderpool.push_back(ord);
}
TCS::~TCS() {
  CLOG(INFO, tcs_log) << "TCS  stop\n";
  stop();
}

bool TCS::init_dispatcher() {
  dispatcher = std::make_shared<kernel::dispatch::Dispatcher>("Dispatcher");
  CLOG(INFO, tcs_log) << "init dispatcher ok\n";
  return true;
}
bool TCS::init_orderpool() {
  orderpool = std::make_shared<kernel::allocate::OrderPool>("OrderPool");
  CLOG(INFO, tcs_log) << "init orderpool ok\n";
  return true;
}
bool TCS::init_scheduler() {
  this->scheduler = std::make_shared<kernel::schedule::Scheduler>("Scheduler");
  CLOG(INFO, tcs_log) << "init scheduler ok\n";
  return true;
}
bool TCS::init_planner() {
  if (!resource) {
    return false;
  }
  this->planner = std::make_shared<kernel::planner::Planner>(this->resource);
  CLOG(INFO, tcs_log) << "init planner ok\n";
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
  assert(dispatcher != nullptr);
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
      dest_v["locationName"] = dest_ ? dest_->name : "";
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
    // 字段检查
    auto ord = std::make_shared<data::order::TransportOrder>(ord_name);
    ord->create_time = std::chrono::system_clock::now();
    if (req.contains("deadline") && !req["deadline"].empty()) {
      auto dt = get_time_from_str(req["deadline"].get<std::string>());
      ord->dead_time = dt.value_or(ord->create_time + std::chrono::minutes(60));
    } else {
      ord->dead_time = ord->create_time + std::chrono::hours(1000000);
    }
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
      //  new orderquence
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
        //  操作类型判断
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
            if (check.first ==
                kernel::allocate::ResourceManager::ResType::Location) {
              auto t1 = std::dynamic_pointer_cast<data::model::Location>(
                  check.second);
              auto it = t1->type.lock()->allowed_ops.find(op);
              if (it == t1->type.lock()->allowed_ops.end()) {
                CLOG(ERROR, driver_log) << op << "type is err";
                ord->state = data::order::TransportOrder::State::FAILED;
                orderpool->ended_orderpool.push_back(ord);
                break;
              }
            }

            auto destination =
                orderpool->res_to_destination(check.second, op_.value());
            if (d.contains("properties")) {
              for (auto &pro : d["properties"]) {
                destination->properties.insert(
                    std::pair<std::string, std::string>(
                        pro["key"].get<std::string>(),
                        pro["value"].get<std::string>()));
              }
            }
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
    dispatcher->notify();
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
    res["properties"][pro.first] = pro.second;
  }
  res["energyLevel"] = v->engerg_level;
  res["energyLevelGood"] = v->energy_level_good;
  std::string integrationlevel;
  if (v->integration_level ==
      kernel::driver::Vehicle::integrationLevel::TO_BE_IGNORED) {
    integrationlevel = "TO_BE_IGNORED";
  } else if (v->integration_level ==
             kernel::driver::Vehicle::integrationLevel::TO_BE_NOTICED) {
    integrationlevel = "TO_BE_NOTICED";
  } else if (v->integration_level ==
             kernel::driver::Vehicle::integrationLevel::TO_BE_RESPECTED) {
    integrationlevel = "TO_BE_RESPECTED";
  } else {
    integrationlevel = "TO_BE_UTILIZED";
  }

  res["integrationLevel"] = integrationlevel;
  res["energyLevelCritical"] = v->energy_level_critical;
  res["paused"] = v->paused;
  res["transportOrder"] = v->current_order ? v->current_order->name : "";
  res["currentPosition"] = v->current_point ? v->current_point->name : "";
  res["precisePosition"]["x"] = v->position.x;
  res["precisePosition"]["y"] = v->position.y;
  res["precisePosition"]["z"] = v->position.z;
  res["state"] = v->get_state();
  res["procState"] = v->get_process_state();
  res["allocatedResources"] = json::array();
  for (auto &r : v->allocated_resources) {
    json ls = json::array();
    for (auto &x : r) {
      ls.push_back(x->name);
    }
    res["allocatedResources"].push_back(ls);
  }
  res["claimedResources"] = json::array();
  for (auto &r : v->claim_resources) {
    json ls = json::array();
    for (auto &x : r) {
      ls.push_back(x->name);
    }
    res["claimedResources"].push_back(ls);
  }
  // todo
  res["allowedOrderTypes"] = json::array();
  for (auto &x : v->allowed_order_type) {
    res["allowedOrderTypes"].push_back(x);
  }
  res["envelopeKey"] = v->envelope_key;
  return res;
}

std::pair<int, std::string> TCS::get_vehicles(const std::string &state) {
  std::shared_lock<std::shared_mutex> lk(mutex);
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
        if (v->state == kernel::driver::Vehicle::State::IDEL) {
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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
  std::shared_lock<std::shared_mutex> lk(mutex);

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
        paused_vehicle(name);
      } else {
        paused_task->action_type =
            vda5050::instantaction::ActionType::stopPause;
        recovery_vehicle(name);
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
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  json res;
  res["name"] = resource->model_name;
  res["properties"] = json::array();
  // visuallayout
  res["visualLayout"]["name"] = resource->visual_layout->name;
  res["visualLayout"]["scaleX"] = resource->visual_layout->scale_x;
  res["visualLayout"]["scaleY"] = resource->visual_layout->scale_y;
  res["visualLayout"]["layers"] = json::array();
  for (auto &x : resource->visual_layout->layers) {
    json layer;
    layer["id"] = x.id;
    layer["ordinal"] = x.ordinal;
    layer["visible"] = x.visible;
    layer["name"] = x.name;
    layer["groupId"] = x.group_id;
    res["visualLayout"]["layers"].push_back(layer);
  }
  res["visualLayout"]["layerGroups"] = json::array();
  for (auto &x : resource->visual_layout->layer_groups) {
    json layer_group;
    layer_group["id"] = x.id;
    layer_group["name"] = x.name;
    layer_group["visible"] = x.visible;
    res["visualLayout"]["layerGroups"].push_back(layer_group);
  }
  res["visualLayout"]["properties"] = json::array();
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
    layout["layerId"] = p->layout.layer_id;
    point["layout"] = layout;
    point["properties"] = json::array();
    for (auto &pro : p->properties) {
      json t;
      t["key"] = pro.first;
      t["value"] = pro.second;
      point["properties"].push_back(t);
    }
    point["vehicleEnvelopes"] = json::array();
    for (auto &x : p->envelopes) {
      json t;
      t["envelopeKey"] = x.first;
      t["vertices"] = json::array();
      auto v = std::dynamic_pointer_cast<data::model::Envelope>(x.second);
      for (auto &v_ : v->vertexs) {
        json ver;
        ver["x"] = v_.x();
        ver["y"] = v_.y();
        t["vertices"].push_back(ver);
      }
      point["vehicleEnvelopes"].push_back(t);
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
      t["key"] = pro.first;
      t["value"] = pro.second;
      path["properties"].push_back(t);
    }
    path["vehicleEnvelope"] = json::array();
    for (auto &x : p->envelopes) {
      json t;
      t["envelopeKey"] = x.first;
      t["vertices"] = json::array();
      auto v = std::dynamic_pointer_cast<data::model::Envelope>(x.second);
      for (auto &v_ : v->vertexs) {
        json ver;
        ver["x"] = v_.x();
        ver["y"] = v_.y();
        t["vertices"].push_back(ver);
      }
      path["vehicleEnvelope"].push_back(t);
    }
    path["peripheralOperations"] = json::array();
    for (auto &x : p->per_acts.acts) {
      json t;
      t["operation"] = x.op_name;
      t["locationName"] = x.op_name;
      t["completionRequired"] = x.completion_required;
      t["executionTrigger"] = x.execution_trigger;
      path["peripheralOperations"].push_back(t);
    }
    res["paths"].push_back(path);
  }
  // location type
  res["locationTypes"] = json::array();
  for (auto &type : resource->location_types) {
    json loc_type;
    loc_type["name"] = type->name;
    loc_type["allowedOperations"] = json::array();
    loc_type["allowedPeripheralOperations"] = json::array();
    loc_type["properties"] = json::array();
    for (auto &x : type->allowed_ops) {
      loc_type["allowedOperations"].push_back(x.first);
    }
    for (auto &x : type->allowrd_per_ops) {
      loc_type["allowedPeripheralOperations"].push_back(x.first);
    }
    for (auto &pro : type->properties) {
      json t;
      t["key"] = pro.first;
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
    location["typeName"] = loc->type.lock()->name;
    location["position"]["x"] = loc->position.x;
    location["position"]["y"] = loc->position.y;
    location["position"]["z"] = loc->position.z;
    location["links"] = json::array();
    if (loc->link.lock()) {
      json link;
      link["pointName"] = loc->link.lock()->name;
      link["allowedOperations"] = json::array();
      for (auto &op : loc->type.lock()->allowed_ops) {
        link["allowedOperations"].push_back(op.first);
      }
      location["links"].push_back(link);
    }
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
      t["key"] = pro.first;
      t["value"] = pro.second;
      location["properties"].push_back(t);
    }
    res["locations"].push_back(location);
  }
  // block
  res["blocks"] = json::array();

  for (auto &x : resource->rules) {
    std::shared_ptr<kernel::allocate::BlockRuleBase> it = nullptr;
    try {
      it = std::dynamic_pointer_cast<kernel::allocate::BlockRuleBase>(x);
    } catch (std::exception &) {
    }
    if (!it) {
      continue;
    }
    json block;
    block["memberNames"] = json::array();
    for (auto &p : it->occs) {
      block["memberNames"].push_back(p->name);
    }
    block["name"] = x->name;
    block["type"] = "SINGLE_VEHICLE_ONLY";
    block["layout"]["color"] = it->color;
    block["properties"] = json::array();
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
    vehicle["energyLevelFullyRecharged"] = v->engrgy_level_recharge;
    vehicle["energyLevelSufficientlyRecharged"] = v->engrgy_level_full;
    vehicle["maxVelocity"] = v->max_vel;
    vehicle["maxReverseVelocity"] = v->max_reverse_vel;
    vehicle["layout"]["routeColor"] = v->color;
    vehicle["paused"] = v->paused;
    vehicle["properties"] = json::array();
    for (auto &pro : v->properties) {
      json t;
      t["name"] = pro.first;
      t["value"] = pro.second;
      vehicle["properties"].push_back(t);
    }
    res["vehicles"].push_back(vehicle);
  }
  return std::pair<int, std::string>(200, res.dump());
}
std::pair<int, std::string> TCS::put_model(const std::string &body) {
  std::unique_lock<std::shared_mutex> lk(mutex);
  stop();
  init_orderpool();
  init_scheduler();
  init_dispatcher();
  try {
    json model = json::parse(body);
    resource =
        std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
    this->resource->is_connected = std::bind(
        &TCS::is_connect, this, std::placeholders::_1, std::placeholders::_2);
    auto control_rule =
        std::make_shared<kernel::allocate::OwnerRule>("control_rule", resource);
    resource->rules.push_back(control_rule);
    auto envelope_rule = std::make_shared<kernel::allocate::CollisionRule>(
        "collision_rule", resource);
    resource->rules.push_back(envelope_rule);
    if (model.contains("name")) {
      resource->model_name = model["name"].get<std::string>();
    }
    // visuallayout
    {
      auto visuallayout = std::make_shared<data::model::VisualLayout>(
          model["visualLayout"]["name"].get<std::string>());
      resource->visual_layout = visuallayout;
      visuallayout->scale_x = model["visualLayout"]["scaleX"].get<double>();
      visuallayout->scale_y = model["visualLayout"]["scaleY"].get<double>();
      for (auto &x : model["visualLayout"]["layers"]) {
        auto layer = data::model::VisualLayout::Layer();
        layer.id = x["id"].get<int>();
        layer.ordinal = x["ordinal"].get<int>();
        layer.visible = x["visible"].get<bool>();
        layer.name = x["name"].get<std::string>();
        layer.group_id = x["groupId"].get<int>();
        visuallayout->layers.push_back(layer);
      }
      for (auto &x : model["visualLayout"]["layerGroups"]) {
        auto layer_group = data::model::VisualLayout::LayerGroup();
        layer_group.id = x["id"].get<int>();
        layer_group.name = x["name"].get<std::string>();
        layer_group.visible = x["visible"].get<bool>();
        visuallayout->layer_groups.push_back(layer_group);
      }
      for (auto &x : model["visualLayout"]["properties"]) {
        visuallayout->properties.insert(std::pair<std::string, std::string>(
            x["key"].get<std::string>(), x["value"].get<std::string>()));
      }
    }
    // point
    {
      for (auto &p : model["points"]) {
        auto point =
            std::make_shared<data::model::Point>(p["name"].get<std::string>());
        point->position.x = p["position"]["x"].get<int>();
        point->position.y = p["position"]["y"].get<int>();
        point->position.z = p["position"]["z"].get<int>();
        if (p.contains("vehicleOrientationAngle")) {
          if (p["vehicleOrientationAngle"].is_number())
            point->client_angle = p["vehicleOrientationAngle"].get<int>();
          else {
            point->client_angle = 0;
          }
        }
        point->type =
            data::model::Point::new_type(p["type"].get<std::string>());
        point->layout.position.x = p["layout"]["position"]["x"].get<int>();
        point->layout.position.y = p["layout"]["position"]["y"].get<int>();
        point->layout.label_offset.x =
            p["layout"]["labelOffset"]["x"].get<int>();
        point->layout.label_offset.y =
            p["layout"]["labelOffset"]["y"].get<int>();
        point->layout.layer_id = p["layout"]["layerId"].get<int>();
        if (p.contains("properties")) {
          for (auto &pro : p["properties"]) {
            std::string value;
            if (pro["value"].is_boolean()) {
              if (pro["value"].get<bool>()) {
                value = "true";
              } else {
                value = "false";
              }
            } else {
              value = pro["value"].get<std::string>();
            }
            point->properties.insert(std::pair<std::string, std::string>(
                pro["key"].get<std::string>(), value));
          }
        }
        for (auto &x : p["vehicleEnvelope"]) {
          auto envelope = std::make_shared<data::model::Envelope>(
              x["envelopeKey"].get<std::string>());
          for (auto &v : p["vehicleEnvelope"]["vertices"]) {
            envelope->add_vertex(v["x"].get<double>(), v["y"].get<double>());
          }
          point->envelopes.insert(
              std::pair<std::string, std::shared_ptr<data::model::Envelope>>(
                  x["envelopeKey"].get<std::string>(), envelope));
        }
        resource->points.push_back(point);
      }
      CLOG(INFO, tcs_log) << "init point size " << resource->points.size();
    }
    // locationtype
    {
      for (auto &x : model["locationTypes"]) {
        auto type = std::make_shared<data::model::LocationType>(
            x["name"].get<std::string>());
        type->layout.location_representation = data::model::new_location_type(
            x["layout"]["locationRepresentation"].get<std::string>());
        for (auto &pro : x["property"]) {
          auto key = pro["key"].get<std::string>();
          auto value = pro["value"].get<std::string>();
          type->properties.insert(
              std::pair<std::string, std::string>(key, value));
        }
        for (auto &allow : x["allowedOperation"]) {
          type->allowed_ops[allow] = std::map<std::string, std::string>();
        }
        for (auto &allow : x["allowedPeripheralOperation"]) {
          type->allowrd_per_ops[allow] = std::map<std::string, std::string>();
        }
        type->get_param();
        resource->location_types.push_back(type);
      }
      CLOG(INFO, tcs_log) << "init loc_type size "
                          << resource->location_types.size();
    }
    // location
    {
      for (auto &l : model["locations"]) {
        auto loc = std::make_shared<data::model::Location>(
            l["name"].get<std::string>());
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
            for (auto &a_op : x->allowed_ops) {
              for (auto &per : loc->properties) {
                a_op.second.insert(per);
              }
            }
            for (auto &a_op : x->allowrd_per_ops) {
              for (auto &per : loc->properties) {
                a_op.second.insert(per);
              }
            }
          }
        }
        if (l["layout"].contains("locationRepresentation")) {
          loc->type.lock()->layout.location_representation =
              data::model::new_location_type(
                  l["layout"]["locationRepresentation"].get<std::string>());
        }
        for (auto &pro : l["property"]) {
          auto key = pro["key"].get<std::string>();
          auto value = pro["value"].get<std::string>();
          loc->properties.insert(
              std::pair<std::string, std::string>(key, value));
        }
        resource->locations.push_back(loc);
      }
      CLOG(INFO, tcs_log) << "init location size "
                          << resource->locations.size();
    }
    // path
    {
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
            std::string value;
            if (pro["value"].is_boolean()) {
              if (pro["value"].get<bool>()) {
                value = "true";
              } else {
                value = "false";
              }
            } else {
              value = pro["value"].get<std::string>();
            }
            path->properties.insert(std::pair<std::string, std::string>(
                pro["key"].get<std::string>(), value));
          }
        }
        if (!p.contains("length")) {
          path->length = (path->source_point.lock()->position -
                          path->destination_point.lock()->position)
                             .norm();
        }
        //
        data::model::Actions acts(path->properties);
        {
          if (p.contains("peripheralOperation")) {
            for (auto &op : p["peripheralOperation"]) {
              auto per_op_name = op["name"].get<std::string>();
              auto wait =
                  op["completionRequired"].get<bool>() ? "SOFT" : "NONE";
              auto when = op["executionTrigger"].get<std::string>();
              auto link_loc_name = op["locationName"].get<std::string>();
              for (auto &loc : resource->locations) {
                if (loc->name == link_loc_name) {
                  if (loc->type.lock()->allowrd_per_ops.find(per_op_name) !=
                      loc->type.lock()->allowrd_per_ops.end()) {
                    // exist
                    data::model::Actions::Action act;
                    act.params.insert(loc->properties.begin(),
                                      loc->properties.end());
                    act.block_type = wait;
                    act.when = when;
                    act.vaild = true;
                    act.id = path->name + "_action_" + per_op_name;
                    act.name = loc->type.lock()->name;
                    acts.append(act);
                    break;
                  }
                }
              }
            }
          }
        }
        path->acts = acts;
        for (auto &x : p["vehicleEnvelope"]) {
          auto envelope = std::make_shared<data::model::Envelope>(
              x["envelopeKey"].get<std::string>());
          for (auto &v : p["vehicleEnvelope"]["vertices"]) {
            envelope->add_vertex(v["x"].get<double>(), v["y"].get<double>());
          }
          path->envelopes.insert(
              std::pair<std::string, std::shared_ptr<data::model::Envelope>>(
                  x["envelopeKey"].get<std::string>(), envelope));
        }
        resource->paths.push_back(path);
      }
      CLOG(INFO, tcs_log) << "init path size " << resource->paths.size();
    }
    // block
    {
      for (auto &block : model["blocks"]) {
        std::unordered_set<std::shared_ptr<TCSResource>> rs;
        for (auto &ch : block["memberNames"]) {
          for (auto &x : resource->points) {
            if (x->name == ch) {
              rs.insert(x);
            }
          }
          for (auto &x : resource->paths) {
            if (x->name == ch) {
              rs.insert(x);
            }
          }
          for (auto &x : resource->locations) {
            if (x->name == ch) {
              rs.insert(x);
            }
          }
        }
        if (block["type"] == "SINGLE_VEHICLE_ONLY") {
          auto rule = std::make_shared<kernel::allocate::OnlyOneGatherRule>(
              block["name"], resource);
          rule->occs = rs;
          rule->color = block["layout"]["color"];
          resource->rules.push_back(rule);
        } else if (block["type"] == "SAME_DIRECTION_ONLY") {
          std::string direction = block["direct"];
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
      }
    }
    // vehicle
    {
      int orderquence{2};
      for (auto &v : model["vehicles"]) {
        std::string address =
            v["properties"]["tcs:preferredAdapterClass"].get<std::string>();
        if (address.find("virtual") != std::string::npos) {
          ///////////////////
          /// // 使用虚拟车辆
          //////////////////
          auto veh = std::make_shared<kernel::driver::SimVehicle>(5, v["name"]);
          veh->length = v["length"].get<int>();
          veh->max_reverse_vel = v["maxReverseVelocity"].get<int>();
          veh->max_vel = v["maxVelocity"].get<int>();
          veh->color = v["layout"]["routeColor"];
          veh->energy_level_critical = v["energyLevelCritical"].get<int>();
          veh->energy_level_good = v["energyLevelGood"].get<int>();
          veh->engrgy_level_full = v["energyLevelFullyRecharged"].get<int>();
          veh->engrgy_level_recharge =
              v["energyLevelSufficientlyRecharged"].get<int>();
          veh->send_queue_size = orderquence;
          veh->envelope_key = v.contains("envelopeKey")
                                  ? v["envelopeKey"].get<std::string>()
                                  : "";
          for (auto &pro : v["properties"]) {
            veh->properties.insert(std::pair<std::string, std::string>(
                pro["key"].get<std::string>(),
                pro["value"].get<std::string>()));
            if (pro["key"] == "loopback:initialPosition") {
              for (auto &x : resource->points) {
                if (x->name == pro["value"].get<std::string>()) {
                  veh->last_point = x;
                }
              }
            }
            if (pro["key"] == "tcs:preferredParkingPosition") {
              for (auto &x : resource->points) {
                if (x->name == pro["value"].get<std::string>()) {
                  veh->park_point = x;
                }
              }
            }
          }
          dispatcher->vehicles.push_back(veh);

        } else if (address.find("vda") != std::string::npos) {
          std::string vda_interfaceName{"rw"};
          std::string vda_serialNumber{"rw"};
          std::string vda_version{"1.0"};
          std::string vda_manufacturer{"rw"};
          for (auto &pro : v["properties"]) {
            if (pro["key"] == "vda5050:interfaceName") {
              vda_interfaceName = pro["value"];
            } else if (pro["key"] == "vda5050:manufacturer") {
              vda_manufacturer = pro["value"];
            } else if (pro["key"] == "vda5050:serialNumber") {
              vda_serialNumber = pro["value"];

            } else if (pro["key"] == "vda5050:version") {
              vda_version = pro["value"];

            } else if (pro["key"] == "vda5050:orderQueueSize") {
              orderquence = pro["value"].get<int>();
            }
          }
          auto veh = std::make_shared<kernel::driver::Rabbit3>(
              v["name"], vda_interfaceName, vda_serialNumber, vda_version,
              vda_manufacturer);
          veh->length = v["length"].get<int>();
          veh->max_reverse_vel = v["maxReverseVelocity"].get<int>();
          veh->max_vel = v["maxVelocity"].get<int>();
          veh->map_id = model["name"];
          veh->color = v["layout"]["routeColor"];
          veh->energy_level_critical = v["energyLevelCritical"].get<int>();
          veh->energy_level_good = v["energyLevelGood"].get<int>();
          veh->engrgy_level_full = v["energyLevelFullyRecharged"].get<int>();
          veh->engrgy_level_recharge =
              v["energyLevelSufficientlyRecharged"].get<int>();
          veh->broker_ip = ip;
          veh->broker_port = port;
          veh->send_queue_size = orderquence;
          veh->envelope_key = v.contains("envelopeKey")
                                  ? v["envelopeKey"].get<std::string>()
                                  : "";
          for (auto &pro : v["properties"]) {
            veh->properties.insert(std::pair<std::string, std::string>(
                pro["key"].get<std::string>(),
                pro["value"].get<std::string>()));
          }
          ///////////////////////////
          dispatcher->vehicles.push_back(veh);
        } else {
          auto veh =
              std::make_shared<kernel::driver::InvalidVehicle>(v["name"]);
          veh->length = v["length"].get<int>();
          veh->max_reverse_vel = v["maxReverseVelocity"].get<int>();
          veh->max_vel = v["maxVelocity"].get<int>();
          veh->color = v["layout"]["routeColor"];
          veh->energy_level_critical = v["energyLevelCritical"].get<int>();
          veh->energy_level_good = v["energyLevelGood"].get<int>();
          veh->engrgy_level_full = v["energyLevelFullyRecharged"].get<int>();
          veh->engrgy_level_recharge =
              v["energyLevelSufficientlyRecharged"].get<int>();
          veh->send_queue_size = orderquence;
          veh->envelope_key = v.contains("envelopeKey")
                                  ? v["envelopeKey"].get<std::string>()
                                  : "";
          dispatcher->vehicles.push_back(veh);
        }
      }
      CLOG(INFO, tcs_log) << "init vehicle size "
                          << dispatcher->vehicles.size();
    }
    CLOG(INFO, tcs_log) << "init resource ok";
    init_planner();
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res = std::bind(&kernel::allocate::ResourceManager::find,
                                     resource, std::placeholders::_1);
    dispatcher->go_home = std::bind(
        &TCS::home_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->order_empty =
        std::bind(&kernel::allocate::OrderPool::is_empty, orderpool);
    dispatcher->get_next_ord =
        std::bind(&kernel::allocate::OrderPool::pop, orderpool);
    dispatcher->go_charge = std::bind(
        &TCS::charge_order, this, std::placeholders::_1, std::placeholders::_2);
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
    assert(dispatcher);
    CLOG(INFO, tcs_log) << "run all ...";
    run();
    CLOG(INFO, tcs_log) << "run all ok";
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
  std::shared_lock<std::shared_mutex> lk(mutex);

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
    veh["envelope"]["type"] = v->envelope_key;
    veh["envelope"]["vertex"] = json::array();
    for (auto &p : resource->points) {
      for (auto &key : p->envelopes) {
        if (v->envelope_key == key.first) {
          auto e = static_cast<data::model::Envelope *>(key.second.get());
          for (auto &vertex : e->vertexs) {
            json ver;
            ver["x"] = vertex.x() * 1000;
            ver["y"] = vertex.y() * 1000;
            veh["envelope"]["vertex"].push_back(ver);
          }
        }
      }
    }

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
    veh["battery_level"] = v->engerg_level;
    value.push_back(veh);
  }
  return value.dump();
}

std::pair<int, std::string> TCS::put_path_locked(const std::string &path_name,
                                                 bool new_value) {
  std::shared_lock<std::shared_mutex> lk(mutex);
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
      reroute();
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
  std::shared_lock<std::shared_mutex> lk(mutex);
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

void TCS::reroute() {
  for (auto &v : dispatcher->vehicles) {
    if (v->current_order) {
      v->reroute();
    }
  }
}

std::pair<int, std::string> TCS::post_reroute() {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  reroute();
  return std::pair<int, std::string>(200, "");
}
bool TCS::is_connect(std::shared_ptr<data::model::Point> a,
                     std::shared_ptr<data::model::Point> b) {
  return planner->find_paths(a, b).size() > 0;
}

std::pair<int, std::string> TCS::post_vehicle_reroute(const std::string &name,
                                                      bool f) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == name) {
      v->reroute();
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "no vehicle named '" + name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

std::pair<int, std::string> TCS::put_vehicle_enable(const std::string &name,
                                                    bool p) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == name) {
      if (!p) {
        v->state = kernel::driver::Vehicle::State::UNAVAILABLE;
      } else {
        v->state = kernel::driver::Vehicle::State::UNKNOWN;
      }
      return std::pair<int, std::string>(200, "");
    }
  }
  json res = json::array();
  auto msg = "no vehicle named '" + name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(404, res.dump());
}

std::pair<int, std::string> TCS::put_vehicle_integration_level(
    const std::string &name, const std::string &p) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == name) {
      if (p == "TO_BE_UTILIZED") {
        v->integration_level =
            kernel::driver::Vehicle::integrationLevel::TO_BE_UTILIZED;
      } else if (p == "TO_BE_RESPECTED") {
        v->integration_level =
            kernel::driver::Vehicle::integrationLevel::TO_BE_RESPECTED;

      } else if (p == "TO_BE_NOTICED") {
        v->integration_level =
            kernel::driver::Vehicle::integrationLevel::TO_BE_NOTICED;

      } else {
        v->integration_level =
            kernel::driver::Vehicle::integrationLevel::TO_BE_IGNORED;
      }
    }
  }
  return std::pair<int, std::string>(200, "");
}

std::pair<int, std::string> TCS::post_vehicle_path_to_point(
    const std::string &name, const std::string &p_) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(404, res.dump());
  }
  json body = json::parse(p_);
  for (auto &v : dispatcher->vehicles) {
    if (v->name == name) {
      json res;
      res["routes"] = json::array();
      std::shared_ptr<data::model::Point> s_p;
      if (body.contains("sourcePoint")) {
        for (auto &p : resource->points) {
          if (p->name == body["sourcePoint"]) {
            s_p = p;
          }
        }
      } else {
        s_p = v->current_point;
      }
      for (auto &e_p : body["destinationPoints"]) {
        std::shared_ptr<data::model::Point> d_p;
        for (auto &p : resource->points) {
          if (p->name == e_p) {
            d_p = p;
          }
        }
        // path
        if (d_p) {
          auto path = planner->find_paths_with_vertex(s_p, d_p);
          if (path.empty()) {
            res["routes"].push_back("");
          } else {
            auto min_path = path.front().first;
            auto cost = path.front().second;
            json route;
            route["destinationPoint"] = e_p;
            route["costs"] = static_cast<int>(cost);
            auto paths = planner->to_model_path(path);
            auto rou = resource->paths_to_route(paths.front());
            route["steps"] = json::array();
            for (auto &r : rou->steps) {
              json step;
              step["path"] = r->path->name;
              step["sourcePoint"] = r->path->source_point.lock()->name;
              step["destinationPoint"] =
                  r->path->destination_point.lock()->name;
              std::string ori;
              if (r->vehicle_orientation ==
                  data::order::Step::Orientation::FORWARD) {
                ori = "FORWARD";
              } else if (r->vehicle_orientation ==
                         data::order::Step::Orientation::BACKWARD) {
                ori = "BACKWARD";
              } else {
                ori = "UNDEFINED";
              }
              step["vehliceOrientation"] = ori;
            }
            res["routes"].push_back(route);
          }
        } else {
          res["routes"].push_back("");
        }
      }
      return std::pair<int, std::string>(200, res.dump());
    }
  }
  return std::pair<int, std::string>(400, "the vehicle does not exist");
}
bool TCS::is_exist_active_order() {
  for (auto &x : orderpool->ended_orderpool) {
    if (x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      return true;
  }
  return false;
}