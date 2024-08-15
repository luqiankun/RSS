#include "../../include/main/rss.hpp"

#include "../../include/kernel/driver/vehicle.hpp"
namespace httplib {
enum StatusCode {
  // Information responses
  Continue_100 = 100,
  SwitchingProtocol_101 = 101,
  Processing_102 = 102,
  EarlyHints_103 = 103,

  // Successful responses
  OK_200 = 200,
  Created_201 = 201,
  Accepted_202 = 202,
  NonAuthoritativeInformation_203 = 203,
  NoContent_204 = 204,
  ResetContent_205 = 205,
  PartialContent_206 = 206,
  MultiStatus_207 = 207,
  AlreadyReported_208 = 208,
  IMUsed_226 = 226,

  // Redirection messages
  MultipleChoices_300 = 300,
  MovedPermanently_301 = 301,
  Found_302 = 302,
  SeeOther_303 = 303,
  NotModified_304 = 304,
  UseProxy_305 = 305,
  unused_306 = 306,
  TemporaryRedirect_307 = 307,
  PermanentRedirect_308 = 308,

  // Client error responses
  BadRequest_400 = 400,
  Unauthorized_401 = 401,
  PaymentRequired_402 = 402,
  Forbidden_403 = 403,
  NotFound_404 = 404,
  MethodNotAllowed_405 = 405,
  NotAcceptable_406 = 406,
  ProxyAuthenticationRequired_407 = 407,
  RequestTimeout_408 = 408,
  Conflict_409 = 409,
  Gone_410 = 410,
  LengthRequired_411 = 411,
  PreconditionFailed_412 = 412,
  PayloadTooLarge_413 = 413,
  UriTooLong_414 = 414,
  UnsupportedMediaType_415 = 415,
  RangeNotSatisfiable_416 = 416,
  ExpectationFailed_417 = 417,
  ImATeapot_418 = 418,
  MisdirectedRequest_421 = 421,
  UnprocessableContent_422 = 422,
  Locked_423 = 423,
  FailedDependency_424 = 424,
  TooEarly_425 = 425,
  UpgradeRequired_426 = 426,
  PreconditionRequired_428 = 428,
  TooManyRequests_429 = 429,
  RequestHeaderFieldsTooLarge_431 = 431,
  UnavailableForLegalReasons_451 = 451,

  // Server error responses
  InternalServerError_500 = 500,
  NotImplemented_501 = 501,
  BadGateway_502 = 502,
  ServiceUnavailable_503 = 503,
  GatewayTimeout_504 = 504,
  HttpVersionNotSupported_505 = 505,
  VariantAlsoNegotiates_506 = 506,
  InsufficientStorage_507 = 507,
  LoopDetected_508 = 508,
  NotExtended_510 = 510,
  NetworkAuthenticationRequired_511 = 511,
};
}
std::pair<int, std::string> RSS::put_model_xml(const std::string &body) {
  std::unique_lock<std::shared_mutex> lock(mutex);
  stop();
  init_orderpool();
  init_planner();
  init_scheduler();
  init_dispatcher();
  this->resource =
      std::make_shared<kernel::allocate::ResourceManager>("ResourceManager");
  this->resource->is_connected = std::bind(
      &RSS::is_connect, this, std::placeholders::_1, std::placeholders::_2);
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
      CLOG(ERROR, rss_log) << "parse error: " << ret.description();
      json res = json::array();
      auto msg =
          "Could not parse XML input '" + std::string(ret.description()) + "'.";
      res.push_back(msg);
      return std::pair<int, std::string>(httplib::BadRequest_400,
                                         res.to_string());
    }
    //
    auto root = doc.first_child();  // <model>
    if (std::string(root.name()) != "model") {
      CLOG(ERROR, rss_log) << "parse error: "
                           << "'don't has model'";
      json res = json::array();
      auto msg =
          "Could not parse XML input '" + std::string("'don't has model'.");
      res.push_back(msg);
      return std::pair<int, std::string>(httplib::BadRequest_400,
                                         res.to_string());
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
        layout.label_offset = Eigen::Vector2i(xLabelOffset, yLabelOffset);
        layout.position = Eigen::Vector2i(xPosition_layout, yPosition_layout);
        auto p = std::make_shared<data::model::Point>(name);
        p->position.x() = xPosition;
        p->position.y() = yPosition;
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
          auto name = envelope.attribute("name").as_string();
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
      CLOG(INFO, rss_log) << "init point size " << resource->points.size();
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
          std::string name_ = allow_ops.attribute("name").as_string();
          std::transform(name_.begin(), name_.end(), name_.begin(), ::tolower);
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
          std::string name_ = allow_per_ops.attribute("name").as_string();
          std::transform(name_.begin(), name_.end(), name_.begin(), ::tolower);
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
      CLOG(INFO, rss_log) << "init loc_type size "
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
        layout.position = Eigen::Vector2i(xPosition_layout, yPosition_layout);
        layout.label_offset = Eigen::Vector2i(xLabelOffset, yLabelOffset);
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

        loc->position = Eigen::Vector3i(xPosition, yPosition, zPosition);
        loc->layout = layout;
        loc->link = link;
        loc->locked = locked;

        resource->locations.push_back(loc);
        location = location.next_sibling();
      }
      CLOG(INFO, rss_log) << "init location size "
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
        int layerId = path.child("pathLayout").attribute("layerId").as_int();

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
        layout.connect_type =
            data::model::Path::new_connect_type(connectionType);
        {
          auto ctrl_point =
              path.child("pathLayout").find_child([](const pugi::xml_node n) {
                return std::string(n.name()) == "controlPoint";
              });
          while (ctrl_point.type() != pugi::node_null) {
            if (std::string(ctrl_point.name()) != "controlPoint") {
              break;
            }
            auto x = ctrl_point.attribute("x").as_int();
            auto y = ctrl_point.attribute("y").as_int();
            layout.control_points.push_back(Eigen::Vector2i(x, y));
            ctrl_point = ctrl_point.next_sibling();
          }
        }
        auto p = std::make_shared<data::model::Path>(name);
        p->layout = layout;
        p->source_point = source_point;
        p->destination_point = destination_point;
        //  xmllength不准
        p->length = Eigen::Vector2d(source_point.lock()->position.x() -
                                        destination_point.lock()->position.x(),
                                    source_point.lock()->position.y() -
                                        destination_point.lock()->position.y())
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
          auto name = envelope.attribute("name").as_string();
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
            std::string per_op_name = peripher_op.attribute("name").as_string();
            std::transform(per_op_name.begin(), per_op_name.end(),
                           per_op_name.begin(), ::tolower);
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
      CLOG(INFO, rss_log) << "init path size " << resource->paths.size();
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
        std::unordered_set<std::shared_ptr<RSSResource>> rs;
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
            CLOG(ERROR, rss_log) << "vehicle " << name << " no init_pos";
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
          veh->engrgy_level_full = energyLevelFullyRecharged;
          veh->engrgy_level_recharge = energyLevelSufficientlyRecharged;
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
          auto deviation_xy = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:deviationXY"));
          });
          auto deviation_theta = vehicle.find_child([](pugi::xml_node node) {
            return (std::string(node.name()) == "property" &&
                    node.attribute("name").as_string() ==
                        std::string("vda5050:deviationTheta"));
          });

          ///////////////////
          /// // 使用mqtt车辆
          //////////////////
          auto vda_deviation_xy =
              deviation_xy.type() == pugi::node_null
                  ? 1.0
                  : deviation_xy.attribute("value").as_double();
          auto vda_deviation_theta =
              deviation_theta.type() == pugi::node_null
                  ? 0.17
                  : deviation_theta.attribute("value").as_double();
          auto vda_interfaceName =
              interfaceName.type() == pugi::node_null
                  ? "virtual"
                  : interfaceName.attribute("value").as_string();
          auto vda_serialNumber =
              serialNumber.type() == pugi::node_null
                  ? "virtual"
                  : serialNumber.attribute("value").as_string();
          std::string vda_version =
              version.type() == pugi::node_null
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
          veh->engrgy_level_full = energyLevelFullyRecharged;
          veh->engrgy_level_recharge = energyLevelSufficientlyRecharged;
          veh->map_id = root.attribute("name").as_string();
          veh->broker_ip = MQTT_IP;
          veh->broker_port = MQTT_PORT;
          veh->deviation_xy = vda_deviation_xy;
          veh->deviation_theta = vda_deviation_theta;
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
          veh->engrgy_level_full = energyLevelFullyRecharged;
          veh->engrgy_level_recharge = energyLevelSufficientlyRecharged;
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
      CLOG(INFO, rss_log) << "init vehicle size "
                          << dispatcher->vehicles.size();
    }
    ///
    CLOG(INFO, rss_log) << "init resource ok\n";
    init_planner();
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res = std::bind(&kernel::allocate::ResourceManager::find,
                                     resource, std::placeholders::_1);
    dispatcher->go_home = std::bind(
        &RSS::home_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->order_empty =
        std::bind(&kernel::allocate::OrderPool::is_empty, orderpool);
    dispatcher->go_charge = std::bind(
        &RSS::charge_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->get_next_ord =
        std::bind(&kernel::allocate::OrderPool::pop, orderpool);
    dispatcher->get_park_point =
        std::bind(&kernel::allocate::ResourceManager::get_recent_park_point,
                  resource, std::placeholders::_1);
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
    assert(dispatcher);
    CLOG(INFO, rss_log) << "run all ...\n";
    run();
    CLOG(INFO, rss_log) << "run all ok\n";
    return std::pair<int, std::string>(static_cast<int>(httplib::OK_200), "");
  } catch (pugi::xpath_exception ec) {
    CLOG(ERROR, rss_log) << "parse error: " << ec.what();
    json res = json::array();
    auto msg = "Could not parse XML input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  }
}

void RSS::paused_vehicle(const std::string &name) {
  std::hash<std::string> hash_fn;
  for (auto &v : dispatcher->vehicles) {
    if (v->name_hash == hash_fn(name)) {
      v->paused = true;
    }
  }
}
void RSS::recovery_vehicle(const std::string &name) {
  std::hash<std::string> hash_fn;
  for (auto &v : dispatcher->vehicles) {
    if (v->name_hash == hash_fn(name)) {
      v->paused = false;
    }
  }
}

void RSS::stop() {
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
void RSS::home_order(const std::string &name,
                     std::shared_ptr<kernel::driver::Vehicle> v) {
  auto time = get_now_utc_time();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>(name);
  std::hash<std::string> hash_fn;
  ord->create_time = time;
  ord->dead_time = time + std::chrono::minutes(60);
  ord->state = data::order::TransportOrder::State::RAW;
  // TODO type ?
  ord->type = "MOVE";
  CLOG(INFO, rss_log) << "new ord " << ord->name << " name_hash "
                      << ord->name_hash << "\n";
  //  检查是否通路

  auto home_point = v->park_point
                        ? v->park_point
                        : resource->get_recent_park_point(v->last_point);
  if (!home_point) {
    CLOG(ERROR, rss_log) << "home point not found";
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
void RSS::charge_order(const std::string &name,
                       std::shared_ptr<kernel::driver::Vehicle> v) {
  v->process_chargeing = true;
  auto time = get_now_utc_time();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>(name);
  std::hash<std::string> hash_fn;
  ord->create_time = time;
  ord->dead_time = time + std::chrono::minutes(60);
  ord->state = data::order::TransportOrder::State::RAW;
  // TODO type ?
  ord->type = "MOVE";
  CLOG(INFO, rss_log) << "new ord " << ord->name << " name_hash "
                      << ord->name_hash << "\n";
  //  检查是否通路
  auto charge_point = resource->get_recent_charge_loc(v->last_point);
  if (!charge_point) {
    CLOG(ERROR, rss_log) << "charge point not found";
    return;
  }
  auto destination = orderpool->res_to_destination(
      charge_point, data::order::DriverOrder::Destination::OpType::CHARGE);
  auto dr = std::make_shared<data::order::DriverOrder>("driverorder_charge_" +
                                                       v->name);
  dr->destination = destination;
  dr->transport_order = ord;
  ord->driverorders.push_back(dr);
  ord->intended_vehicle = v;
  orderpool->orderpool.push_back(ord);
}
RSS::~RSS() {
  CLOG(INFO, rss_log) << "TCS  stop\n";
  stop();
}

bool RSS::init_dispatcher() {
  dispatcher = std::make_shared<kernel::dispatch::Dispatcher>("Dispatcher");
  CLOG(INFO, rss_log) << "init dispatcher ok\n";
  return true;
}
bool RSS::init_orderpool() {
  orderpool = std::make_shared<kernel::allocate::OrderPool>("OrderPool");
  CLOG(INFO, rss_log) << "init orderpool ok\n";
  return true;
}
bool RSS::init_scheduler() {
  this->scheduler = std::make_shared<kernel::schedule::Scheduler>("Scheduler");
  CLOG(INFO, rss_log) << "init scheduler ok\n";
  return true;
}
bool RSS::init_planner() {
  if (!resource) {
    return false;
  }
  this->planner = std::make_shared<kernel::planner::Planner>(this->resource);
  CLOG(INFO, rss_log) << "init planner ok\n";
  return true;
}
void RSS::cancel_all_order() { orderpool->cancel_all_order(); }

void RSS::cancel_order(const std::string &order_name) {
  std::hash<std::string> hash_fn;
  orderpool->cancel_order(hash_fn(order_name));
}
void RSS::cancel_vehicle_all_order(const std::string &vehicle_name) {
  std::hash<std::string> hash_fn;
  for (auto &v : dispatcher->vehicles) {
    if (v->name_hash == hash_fn(vehicle_name)) {
      v->cancel_all_order();
    }
  }
}
void RSS::run() {
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
    value["peripheralReservationToken"] = v->peripheral_reservation_token;
    value["wrappingSequence"] =
        v->ordersequence.lock() ? v->ordersequence.lock()->name : "";
    value["createTime"] = get_time_fmt(v->create_time);
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
        pro["name"] = property.first;
        pro["value"] = property.second;
        dest_v["properties"].push_back(pro);
      }
      value["destinations"].push_back(dest_v);
    }
  }
  return value;
}

std::pair<int, std::string> RSS::get_transport_orders(
    const std::string &vehicle) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
    return std::pair<int, std::string>(httplib::OK_200, res.to_string());
  } else {
    for (auto &v : dispatcher->vehicles) {
      if (v->name == vehicle) {
        json res = json::array();
        for (auto &v : v->orders) {
          json value = order_to_json(v);
          res.push_back(value);
        }
        return std::pair<int, std::string>(httplib::OK_200, res.to_string());
      }
    }
    json res = json::array();
    auto msg = "Could not find the intended vehicle '" + vehicle + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
}

std::pair<int, std::string> RSS::get_transport_order(
    const std::string &ord_name) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  json res = json::array();
  for (auto &ord : orderpool->orderpool) {
    if (ord->name == ord_name) {
      res.push_back(order_to_json(ord));
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    }
  }
  for (auto &ord : orderpool->ended_orderpool) {
    if (ord->name == ord_name) {
      res.push_back(order_to_json(ord));
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    }
  }
  auto msg = "Could not find transport order '" + ord_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}
std::pair<int, std::string> RSS::post_transport_order(
    const std::string &ord_name, const std::string &body) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &o : orderpool->orderpool) {
    if (o->name == ord_name) {
      json res = json::array();
      auto msg = "Transport order '" + ord_name + "' already exists.";
      res.push_back(msg);
      CLOG(ERROR, rss_log) << msg;
      return std::pair<int, std::string>(httplib::Conflict_409,
                                         res.to_string());
    }
  }
  for (auto &o : orderpool->ended_orderpool) {
    if (o->name == ord_name) {
      json res = json::array();
      auto msg = "Transport order '" + ord_name + "' already exists.";
      res.push_back(msg);
      CLOG(ERROR, rss_log) << msg;
      return std::pair<int, std::string>(httplib::Conflict_409,
                                         res.to_string());
    }
  }
  try {
    auto req = json::parse(body);
    // 字段检查
    auto ord = std::make_shared<data::order::TransportOrder>(ord_name);
    ord->create_time = get_now_utc_time();
    if (req.contains("deadline") && !req["deadline"].empty()) {
      auto dt = get_time_from_str(req["deadline"].as_string());
      ord->dead_time = dt.value_or(ord->create_time + std::chrono::minutes(60));
    } else {
      ord->dead_time = ord->create_time + std::chrono::hours(1000000);
    }
    if (req.contains("intendedVehicle")) {
      for (auto &v : dispatcher->vehicles) {
        if (v->name == req["intendedVehicle"].as_string()) {
          ord->intended_vehicle = v;
        }
      }
    }
    bool quence{false};
    if (req.contains("wrappingSequence")) {
      for (auto &o : orderpool->orderquence) {
        if (o->name == req["wrappingSequence"].as_string()) {
          o->add_transport_ord(ord);
          quence = true;
          break;
        }
      }
      if (!quence) {
        //  new orderquence
        auto new_orderquence = std::make_shared<data::order::OrderSequence>(
            req["wrappingSequence"].as_string());
        new_orderquence->add_transport_ord(ord);
        orderpool->orderquence.push_back(new_orderquence);
      }
    }
    if (req.contains("type")) ord->type = req["type"].as_string();
    auto destinations = req["destinations"];
    if (destinations.empty()) {
      LOG(WARNING) << ord->name << " op is null";
      ord->state = data::order::TransportOrder::State::FAILED;
      orderpool->ended_orderpool.push_back(ord);
    } else {
      for (auto &d : destinations.array_range()) {
        //  操作类型判断
        auto loc = d["locationName"].as_string();
        auto check = resource->find(loc);
        if (check.first == kernel::allocate::ResourceManager::ResType::Err) {
          json res = json::array();
          auto msg = "Could not find location '" + loc + "'.";
          res.push_back(msg);
          CLOG(ERROR, rss_log) << msg;
          ord->state = data::order::TransportOrder::State::UNROUTABLE;
          orderpool->ended_orderpool.push_back(ord);
          return std::pair<int, std::string>(httplib::NotFound_404,
                                             res.to_string());
        } else {
          auto op = d["operation"].as_string();
          auto op_ = data::model::Actions::get_optype(op);
          if (op_.has_value()) {
            if (check.first ==
                kernel::allocate::ResourceManager::ResType::Location) {
              auto t1 = std::dynamic_pointer_cast<data::model::Location>(
                  check.second);
              std::string op_lower = op;
              std::transform(op_lower.begin(), op_lower.end(), op_lower.begin(),
                             ::tolower);
              auto it = t1->type.lock()->allowed_ops.find(op_lower);
              if (it == t1->type.lock()->allowed_ops.end()) {
                CLOG(ERROR, driver_log)
                    << t1->name << " not support <" << op << "> type";
                ord->state = data::order::TransportOrder::State::FAILED;
                orderpool->ended_orderpool.push_back(ord);
                json res = json::array();
                auto msg = "not support type '" + op + "'.";
                res.push_back(msg);
                return std::pair<int, std::string>(httplib::NotFound_404,
                                                   res.to_string());
              }
            }

            auto destination =
                orderpool->res_to_destination(check.second, op_.value());
            if (d.contains("properties")) {
              for (auto &pro : d["properties"].array_range()) {
                destination->properties.insert(
                    std::pair<std::string, std::string>(
                        pro["name"].as_string(), pro["value"].as_string()));
              }
            }
            auto dr = std::make_shared<data::order::DriverOrder>(
                "driverorder_" + loc);
            dr->destination = destination;
            dr->transport_order = ord;
            ord->driverorders.push_back(dr);
          } else {
            CLOG(ERROR, rss_log)
                << "op type <" + std::string(op) + "> is not support";
            ord->state = data::order::TransportOrder::State::FAILED;
            orderpool->ended_orderpool.push_back(ord);
            json res = json::array();
            auto msg = "Could not support type '" + op + "'.";
            res.push_back(msg);
            return std::pair<int, std::string>(httplib::NotFound_404,
                                               res.to_string());
          }
        }
      }
    }
    if (req.contains("properties")) {
      for (auto &pro : req["properties"].array_range()) {
        ord->properties.insert(std::pair<std::string, std::string>(
            pro["name"].as_string(), pro["value"].as_string()));
      }
    }
    if (req.contains("dependencies")) {
      for (auto &dep : req["dependencies"].array_range()) {
        for (auto &o : orderpool->orderpool) {
          if (o->name == dep.as_string()) {
            ord->dependencies.push_back(o);
          }
        }
        for (auto &o : orderpool->ended_orderpool) {
          if (o->name == dep.as_string()) {
            ord->dependencies.push_back(o);
          }
        }
      }
    }
    ord->peripheral_reservation_token =
        req.contains("peripheralReservationToken")
            ? req["peripheralReservationToken"].as_string()
            : "";
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
    res["peripheralReservationToken"] = ord->peripheral_reservation_token;
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
        p["name"] = pro.first;
        p["value"] = pro.second;
        value["properties"].push_back(p);
      }
      res["destinations"].push_back(value);
    }
    return std::pair<int, std::string>(httplib::OK_200, res.to_string());
  } catch (jsoncons::ser_error ec) {
    CLOG(ERROR, rss_log) << "ser_error error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  } catch (jsoncons::key_not_found ec) {
    CLOG(ERROR, rss_log) << "key_not_found error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  } catch (jsoncons::not_an_object ec) {
    CLOG(ERROR, rss_log) << "not_an_object error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  }
}

std::pair<int, std::string> RSS::post_transport_order_withdrawl(
    const std::string &ord_name, bool immediate, bool) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &o : orderpool->orderpool) {
    if (o->name == ord_name) {
      o->state = data::order::TransportOrder::State::WITHDRAWL;
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  for (auto &o : orderpool->ended_orderpool) {
    if (o->name == ord_name) {
      o->state = data::order::TransportOrder::State::WITHDRAWL;
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find transport order '" + ord_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
    p["name"] = pro.first;
    p["value"] = pro.second;
    res["properties"].push_back(p);
  }
  return res;
}
std::pair<int, std::string> RSS::get_ordersequences(
    const std::string &vehicle) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
      return std::pair<int, std::string>(httplib::NotFound_404,
                                         res.to_string());
    } else {
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    }
  } else {
    json res = json::array();
    for (auto &q : orderpool->orderquence) {
      res.push_back(orderquence_to_json(q));
    }
    return std::pair<int, std::string>(httplib::OK_200, res.to_string());
  }
}

std::pair<int, std::string> RSS::get_ordersequence(
    const std::string &sequence_name) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &q : orderpool->orderquence) {
    if (q->name == sequence_name) {
      auto res = orderquence_to_json(q);
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    }
  }
  json res = json::array();
  auto msg = "Could not find order sequence '" + sequence_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::post_ordersequence(
    const std::string &sequence_name, const std::string &body) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &s : orderpool->orderquence) {
    if (s->name == sequence_name) {
      json res = json::array();
      auto msg = "Order sequence '" + sequence_name + "' already exists.";
      res.push_back(msg);
      return std::pair<int, std::string>(httplib::Conflict_409,
                                         res.to_string());
    }
  }
  try {
    auto req = json::parse(body);
    std::string veh{};
    if (req.contains("intendedVehicle")) {
      veh = req["intendedVehicle"].as_string();
      for (auto &v : dispatcher->vehicles) {
        if (v->name == veh) {
          auto new_orderquence =
              std::make_shared<data::order::OrderSequence>(sequence_name);
          new_orderquence->intended_vehicle = v;
          if (req.contains("failureFatal")) {
            new_orderquence->failure_fatal = req["failureFatal"].as_bool();
          }
          if (req.contains("incompleteName")) {
            new_orderquence->complete = req["incompleteName"].as_bool();
          }
          if (req.contains("properties")) {
            for (auto &pro : req["properties"].array_range()) {
              new_orderquence->properties[pro["name"].as_string()] =
                  pro["value"].as_string();
            }
          }
          // return
          auto res = orderquence_to_json(new_orderquence);
          orderpool->orderquence.push_back(new_orderquence);
          return std::pair<int, std::string>(httplib::OK_200, res.to_string());
        }
      }
    }
    json res = json::array();
    auto msg = "Could not find Vehicle " + veh + ".";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  } catch (jsoncons::not_an_object ec) {
    CLOG(ERROR, rss_log) << "not_an_object error:" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  } catch (jsoncons::ser_error ec) {
    CLOG(ERROR, rss_log) << "ser_error error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  } catch (jsoncons::key_not_found ec) {
    CLOG(ERROR, rss_log) << "key_not_found error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
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
  res["precisePosition"]["x"] = v->position.x();
  res["precisePosition"]["y"] = v->position.y();
  res["precisePosition"]["z"] = v->position.z();
  res["state"] = v->get_state();
  res["procState"] = v->get_process_state();
  res["allocatedResources"] = json::array();
  for (auto &r : v->allocated_resources) {
    if (r.empty()) {
      continue;
    }
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

std::pair<int, std::string> RSS::get_vehicles(const std::string &state) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  if (state.empty()) {
    json res = json::array();
    for (auto &v : dispatcher->vehicles) {
      res.push_back(vehicle_to_json(v));
    }
    return std::pair<int, std::string>(httplib::OK_200, res.to_string());
  } else {
    if (state == "IDLE") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::IDEL) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    } else if (state == "ERROR") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::ERROR) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    } else if (state == "EXECUTING") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::EXECUTING) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    } else if (state == "UNKNOWN") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::UNKNOWN) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    } else if (state == "UNAVAILABLE") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::UNAVAILABLE) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    } else if (state == "CHARGING") {
      json res = json::array();
      for (auto &v : dispatcher->vehicles) {
        if (v->state == kernel::driver::Vehicle::State::CHARGING) {
          res.push_back(vehicle_to_json(v));
        }
      }
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    } else {
      json res = json::array();
      auto msg = "Could not parse input.";
      res.push_back(msg);
      return std::pair<int, std::string>(httplib::BadRequest_400,
                                         res.to_string());
    }
  }
}

std::pair<int, std::string> RSS::get_vehicle(const std::string &vehicle) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == vehicle) {
      auto res = vehicle_to_json(v);
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    }
  }
  json res = json::array();
  auto msg = "Could not find vehicle '" + vehicle + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::post_vehicle_withdrawl(
    const std::string &vehicle, bool, bool) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == vehicle) {
      if (v->current_order) {
        v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      for (auto &o : v->orders) {
        o->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find vehicle '" + vehicle + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::put_vehicle_paused(const std::string &name,
                                                    bool p) {
  std::shared_lock<std::shared_mutex> lk(mutex);

  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &x : dispatcher->vehicles) {
    if (x->name == name) {
      //
      // x->paused = p;
      auto paused_task = std::make_shared<vda5050::instantaction::Action>();
      paused_task->action_description = "set paused action";
      if (p) {
        paused_task->action_type =
            vda5050::instantaction::ActionType::STARTPAUSE;
        paused_vehicle(name);
      } else {
        paused_task->action_type =
            vda5050::instantaction::ActionType::STOPPAUSE;
        recovery_vehicle(name);
      }
      paused_task->blocking_type =
          vda5050::instantaction::ActionBlockingType::HARD;
      paused_task->action_id = name + "_set_pasued_<" + std::to_string(p) + ">";
      x->execute_instatn_action(paused_task);
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "no vehicle named '" + name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::get_model() {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
    layout["layerId"] = p->layout.layer_id;
    point["layout"] = layout;
    point["properties"] = json::array();
    for (auto &pro : p->properties) {
      json t;
      t["name"] = pro.first;
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
    path["layout"]["controlPoints"] = json::array();
    for (auto &ctrl_p : p->layout.control_points) {
      json ctrl_p_;
      ctrl_p_["x"] = ctrl_p.x();
      ctrl_p_["y"] = ctrl_p.y();
      path["layout"]["controlPoints"].push_back(ctrl_p_);
    }
    path["properties"] = json::array();
    for (auto &pro : p->properties) {
      json t;
      t["name"] = pro.first;
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
      std::string x_type = x.first;
      x_type[0] = ::toupper(x_type[0]);
      loc_type["allowedOperations"].push_back(x_type);
    }
    for (auto &x : type->allowrd_per_ops) {
      std::string x_type = x.first;
      x_type[0] = ::toupper(x_type[0]);
      loc_type["allowedPeripheralOperations"].push_back(x_type);
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
    location["typeName"] = loc->type.lock()->name;
    location["position"]["x"] = loc->position.x();
    location["position"]["y"] = loc->position.y();
    location["position"]["z"] = loc->position.z();
    location["links"] = json::array();
    if (loc->link.lock()) {
      json link;
      link["pointName"] = loc->link.lock()->name;
      link["allowedOperations"] = json::array();
      for (auto &op : loc->type.lock()->allowed_ops) {
        std::string x_type = op.first;
        x_type[0] = ::toupper(x_type[0]);
        link["allowedOperations"].push_back(x_type);
      }
      location["links"].push_back(link);
    }
    location["locked"] = loc->locked;
    location["layout"]["position"]["x"] = loc->layout.position.x();
    location["layout"]["position"]["y"] = loc->layout.position.y();
    location["layout"]["labelOffset"]["x"] = loc->layout.label_offset.x();
    location["layout"]["labelOffset"]["y"] = loc->layout.label_offset.y();
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
    vehicle["energyLevelFullyRecharged"] = v->engrgy_level_full;
    vehicle["energyLevelSufficientlyRecharged"] = v->engrgy_level_recharge;
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
  return std::pair<int, std::string>(httplib::OK_200, res.to_string());
}
std::pair<int, std::string> RSS::put_model(const std::string &body) {
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
        &RSS::is_connect, this, std::placeholders::_1, std::placeholders::_2);
    auto control_rule =
        std::make_shared<kernel::allocate::OwnerRule>("control_rule", resource);
    resource->rules.push_back(control_rule);
    auto envelope_rule = std::make_shared<kernel::allocate::CollisionRule>(
        "collision_rule", resource);
    resource->rules.push_back(envelope_rule);
    if (model.contains("name")) {
      resource->model_name = model["name"].as_string();
    }
    // visuallayout
    {
      auto visuallayout = std::make_shared<data::model::VisualLayout>(
          model["visualLayout"]["name"].as_string());
      resource->visual_layout = visuallayout;
      if (model["visualLayout"].contains("scaleX")) {
        visuallayout->scale_x = model["visualLayout"]["scaleX"].as_double();
      }
      if (model["visualLayout"].contains("scaleY")) {
        visuallayout->scale_y = model["visualLayout"]["scaleY"].as_double();
      }
      if (model["visualLayout"].contains("layers")) {
        for (auto &x : model["visualLayout"]["layers"].array_range()) {
          auto layer = data::model::VisualLayout::Layer();
          layer.id = x["id"].as_integer<int>();
          layer.ordinal = x["ordinal"].as_integer<int>();
          layer.visible = x["visible"].as_bool();
          layer.name = x["name"].as_string();
          layer.group_id = x["groupId"].as_integer<int>();
          visuallayout->layers.push_back(layer);
        }
      }
      if (model["visualLayout"].contains("layerGroups")) {
        for (auto &x : model["visualLayout"]["layerGroups"].array_range()) {
          auto layer_group = data::model::VisualLayout::LayerGroup();
          layer_group.id = x["id"].as_integer<int>();
          layer_group.name = x["name"].as_string();
          layer_group.visible = x["visible"].as_bool();
          visuallayout->layer_groups.push_back(layer_group);
        }
      }
      if (model["visualLayout"].contains("properties")) {
        for (auto &x : model["visualLayout"]["properties"].array_range()) {
          visuallayout->properties.insert(std::pair<std::string, std::string>(
              x["name"].as_string(), x["value"].as_string()));
        }
      }
    }
    // point
    {
      for (auto &p : model["points"].array_range()) {
        auto point =
            std::make_shared<data::model::Point>(p["name"].as_string());
        if (p.contains("position")) {
          point->position.x() = p["position"]["x"].as_integer<int>();
          point->position.y() = p["position"]["y"].as_integer<int>();
          point->position.z() = p["position"]["z"].as_integer<int>();
        }
        if (p.contains("vehicleOrientationAngle")) {
          if (p["vehicleOrientationAngle"].is_number())
            point->client_angle =
                p["vehicleOrientationAngle"].as_integer<int>();
          else {
            point->client_angle = 0;
          }
        }
        if (p.contains("type")) {
          point->type = data::model::Point::new_type(p["type"].as_string());
        }
        if (p.contains("layout")) {
          point->layout.position.x() =
              p["layout"]["position"]["x"].as_integer<int>();
          point->layout.position.y() =
              p["layout"]["position"]["y"].as_integer<int>();
          point->layout.label_offset.x() =
              p["layout"]["labelOffset"]["x"].as_integer<int>();
          point->layout.label_offset.y() =
              p["layout"]["labelOffset"]["y"].as_integer<int>();
          point->layout.layer_id = p["layout"]["layerId"].as_integer<int>();
        }
        if (p.contains("properties")) {
          for (auto &pro : p["properties"].array_range()) {
            std::string value;
            if (pro["value"].is_bool()) {
              if (pro["value"].as_bool()) {
                value = "true";
              } else {
                value = "false";
              }
            } else {
              value = pro["value"].as_string();
            }
            point->properties.insert(std::pair<std::string, std::string>(
                pro["name"].as_string(), value));
          }
        }
        if (p.contains("vehicleEnvelope")) {
          for (auto &x : p["vehicleEnvelope"].array_range()) {
            auto envelope = std::make_shared<data::model::Envelope>(
                x["envelopeKey"].as_string());
            for (auto &v : p["vehicleEnvelope"]["vertices"].array_range()) {
              envelope->add_vertex(v["x"].as_double(), v["y"].as_double());
            }
            point->envelopes.insert(
                std::pair<std::string, std::shared_ptr<data::model::Envelope>>(
                    x["envelopeKey"].as_string(), envelope));
          }
        }
        resource->points.push_back(point);
      }
      CLOG(INFO, rss_log) << "init point size " << resource->points.size();
    }
    // locationtype
    {
      for (auto &x : model["locationTypes"].array_range()) {
        auto type =
            std::make_shared<data::model::LocationType>(x["name"].as_string());
        if (x.contains("layout")) {
          type->layout.location_representation = data::model::new_location_type(
              x["layout"]["locationRepresentation"].as_string());
        }
        if (x.contains("properties")) {
          for (auto &pro : x["properties"].array_range()) {
            auto name = pro["name"].as_string();
            auto value = pro["value"].as_string();
            type->properties.insert(
                std::pair<std::string, std::string>(name, value));
          }
        }
        if (x.contains("allowedOperations")) {
          for (auto &allow : x["allowedOperations"].array_range()) {
            std::string allow_lower = allow.as_string();
            std::transform(allow_lower.begin(), allow_lower.end(),
                           allow_lower.begin(), ::tolower);
            type->allowed_ops[allow_lower] =
                std::map<std::string, std::string>();
          }
        }
        if (x.contains("allowedPeripheralOperations")) {
          for (auto &allow : x["allowedPeripheralOperations"].array_range()) {
            std::string allow_lower = allow.as_string();
            std::transform(allow_lower.begin(), allow_lower.end(),
                           allow_lower.begin(), ::tolower);
            type->allowrd_per_ops[allow_lower] =
                std::map<std::string, std::string>();
          }
        }
        type->get_param();
        resource->location_types.push_back(type);
      }
      CLOG(INFO, rss_log) << "init loc_type size "
                          << resource->location_types.size();
    }
    // location
    {
      for (auto &l : model["locations"].array_range()) {
        auto loc =
            std::make_shared<data::model::Location>(l["name"].as_string());
        loc->position.x() = l["position"]["x"].as_integer<int>();
        loc->position.y() = l["position"]["y"].as_integer<int>();
        loc->position.z() = l["position"]["z"].as_integer<int>();
        if (l.contains("links")) {
          if (!l["links"].empty()) {
            auto p_name = l["links"][0]["pointName"].as_string();
            for (auto &x : resource->points) {
              if (x->name == p_name) {
                loc->link = x;
              }
            }
          }
        }
        if (l.contains("locked")) {
          loc->locked = l["locked"].as_bool();
        }
        for (auto &x : resource->location_types) {
          if (x->name == l["typeName"].as_string()) {
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
        if (l.contains("layout")) {
          loc->layout.position.x() =
              l["layout"]["position"]["x"].as_integer<int>();
          loc->layout.position.y() =
              l["layout"]["position"]["y"].as_integer<int>();
          loc->layout.label_offset.x() =
              l["layout"]["labelOffset"]["x"].as_integer<int>();
          loc->layout.label_offset.y() =
              l["layout"]["labelOffset"]["y"].as_integer<int>();
          loc->layout.layer_id = l["layout"]["layerId"].as_integer<int>();
          if (l["layout"].contains("locationRepresentation")) {
            loc->type.lock()->layout.location_representation =
                data::model::new_location_type(
                    l["layout"]["locationRepresentation"].as_string());
          }
        }
        if (l.contains("properties")) {
          for (auto &pro : l["properties"].array_range()) {
            auto name = pro["name"].as_string();
            auto value = pro["value"].as_string();
            loc->properties.insert(
                std::pair<std::string, std::string>(name, value));
          }
        }
        resource->locations.push_back(loc);
      }
      CLOG(INFO, rss_log) << "init location size "
                          << resource->locations.size();
    }
    // path
    {
      for (auto &p : model["paths"].array_range()) {
        auto path = std::make_shared<data::model::Path>(p["name"].as_string());
        path->max_vel = p["maxVelocity"].as_integer<int>();
        path->max_reverse_vel = p["maxReverseVelocity"].as_integer<int>();
        if (p.contains("locked")) {
          path->locked = p["locked"].as_bool();
        }
        for (auto &x : resource->points) {
          if (x->name == p["srcPointName"].as_string()) {
            path->source_point = x;
            x->incoming_paths.push_back(path);
          }
          if (x->name == p["destPointName"].as_string()) {
            path->destination_point = x;
            x->outgoing_paths.push_back(path);
          }
        }
        if (p.contains("layout")) {
          path->layout.layer_id = p["layout"]["layerId"].as_integer<int>();
          path->layout.connect_type = data::model::Path::new_connect_type(
              p["layout"]["connectionType"].as_string());
          if (p["layout"].contains("controlPoints")) {
            for (auto &ctrl_p : p["layout"]["controlPoints"].array_range()) {
              int x = ctrl_p["x"].as_integer<int>();
              int y = ctrl_p["y"].as_integer<int>();
              path->layout.control_points.push_back(Eigen::Vector2i(x, y));
            }
          }
        }
        if (p.contains("properties")) {
          for (auto &pro : p["properties"].array_range()) {
            std::string value;
            if (pro["value"].is_bool()) {
              if (pro["value"].as_bool()) {
                value = "true";
              } else {
                value = "false";
              }
            } else {
              value = pro["value"].as_string();
            }
            path->properties.insert(std::pair<std::string, std::string>(
                pro["name"].as_string(), value));
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
            for (auto &op : p["peripheralOperation"].array_range()) {
              auto per_op_name = op["name"].as_string();
              std::transform(per_op_name.begin(), per_op_name.end(),
                             per_op_name.begin(), ::tolower);
              auto wait = op["completionRequired"].as_bool() ? "SOFT" : "NONE";
              auto when = op["executionTrigger"].as_string();
              auto link_loc_name = op["locationName"].as_string();
              for (auto &loc : resource->locations) {
                if (loc->name == link_loc_name) {
                  if (loc->type.lock()->allowrd_per_ops.find(per_op_name) !=
                      loc->type.lock()->allowrd_per_ops.end()) {
                    // exist
                    data::model::Actions::Action act;
                    act.action_id = path->name + "_action_" + per_op_name;
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
        if (p.contains("vehicleEnvelope")) {
          for (auto &x : p["vehicleEnvelope"].array_range()) {
            auto envelope = std::make_shared<data::model::Envelope>(
                x["envelopeKey"].as_string());
            for (auto &v : p["vehicleEnvelope"]["vertices"].array_range()) {
              envelope->add_vertex(v["x"].as_double(), v["y"].as_double());
            }
            path->envelopes.insert(
                std::pair<std::string, std::shared_ptr<data::model::Envelope>>(
                    x["envelopeKey"].as_string(), envelope));
          }
        }
        resource->paths.push_back(path);
      }
      CLOG(INFO, rss_log) << "init path size " << resource->paths.size();
    }
    // block
    {
      for (auto &block : model["blocks"].array_range()) {
        std::unordered_set<std::shared_ptr<RSSResource>> rs;
        if (block.contains("memberNames")) {
          for (auto &ch : block["memberNames"].array_range()) {
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
        }
        if (block.contains("type")) {
          if (block["type"] == "SINGLE_VEHICLE_ONLY") {
            auto rule = std::make_shared<kernel::allocate::OnlyOneGatherRule>(
                block["name"].as_string(), resource);
            rule->occs = rs;
            rule->color = block["layout"]["color"].as_string();
            resource->rules.push_back(rule);
          } else if (block["type"] == "SAME_DIRECTION_ONLY") {
            std::string direction = block["direct"].as_string();
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
    }
    // vehicle
    {
      int orderquence{2};
      for (auto &v : model["vehicles"].array_range()) {
        std::string adapter;
        if (v.contains("properties")) {
          for (auto &x : v["properties"].array_range()) {
            if (x["name"] == "tcs:preferredAdapterClass") {
              adapter = x["value"].as_string();
            }
          }
        }
        if (adapter.find("virtual") != std::string::npos) {
          ///////////////////
          /// // 使用虚拟车辆
          //////////////////
          auto veh = std::make_shared<kernel::driver::SimVehicle>(
              5, v["name"].as_string());
          if (v.contains("length")) {
            veh->length = v["length"].as_integer<int>();
          }
          if (v.contains("maxReverseVelocity")) {
            veh->max_reverse_vel = v["maxReverseVelocity"].as_integer<int>();
          }
          if (v.contains("maxVelocity")) {
            veh->max_vel = v["maxVelocity"].as_integer<int>();
          }
          if (v.contains("layout")) {
            veh->color = v["layout"]["routeColor"].as_string();
          }
          if (v.contains("energyLevelCritical")) {
            veh->energy_level_critical =
                v["energyLevelCritical"].as_integer<int>();
          }
          if (v.contains("energyLevelGood")) {
            veh->energy_level_good = v["energyLevelGood"].as_integer<int>();
          }
          if (v.contains("energyLevelFullyRecharged")) {
            veh->engrgy_level_full =
                v["energyLevelFullyRecharged"].as_integer<int>();
          }
          if (v.contains("energyLevelSufficientlyRecharged")) {
            veh->engrgy_level_recharge =
                v["energyLevelSufficientlyRecharged"].as_integer<int>();
          }
          veh->send_queue_size = orderquence;
          veh->envelope_key =
              v.contains("envelopeKey") ? v["envelopeKey"].as_string() : "";
          if (v.contains("properties")) {
            for (auto &pro : v["properties"].array_range()) {
              veh->properties.insert(std::pair<std::string, std::string>(
                  pro["name"].as_string(), pro["value"].as_string()));
              if (pro["name"] == "loopback:initialPosition") {
                for (auto &x : resource->points) {
                  if (x->name == pro["value"].as_string()) {
                    veh->last_point = x;
                  }
                }
              }
              if (pro["name"] == "tcs:preferredParkingPosition") {
                for (auto &x : resource->points) {
                  if (x->name == pro["value"].as_string()) {
                    veh->park_point = x;
                  }
                }
              }
            }
          }
          dispatcher->vehicles.push_back(veh);

        } else if (adapter.find("vda") != std::string::npos) {
          std::string vda_interfaceName{"rw"};
          std::string vda_serialNumber{"rw"};
          std::string vda_version{"v1"};
          std::string vda_manufacturer{"rw"};
          float deviationXY{1.0};
          float deviationTheta{0.17};
          if (v.contains("properties")) {
            for (auto &pro : v["properties"].array_range()) {
              if (pro["name"] == "vda5050:interfaceName") {
                vda_interfaceName = pro["value"].as_string();
              } else if (pro["name"] == "vda5050:manufacturer") {
                vda_manufacturer = pro["value"].as_string();
              } else if (pro["name"] == "vda5050:serialNumber") {
                vda_serialNumber = pro["value"].as_string();

              } else if (pro["name"] == "vda5050:version") {
                vda_version = pro["value"].as_string();

              } else if (pro["name"] == "vda5050:orderQueueSize") {
                orderquence = pro["value"].as_integer<int>();
              } else if (pro["name"] == "vda5050:deviationXY") {
                deviationXY = pro["value"].as_double();
              } else if (pro["name"] == "vda5050:deviationTheta") {
                deviationTheta = pro["value"].as_double();
              }
            }
          }
          auto veh = std::make_shared<kernel::driver::Rabbit3>(
              v["name"].as_string(), vda_interfaceName, vda_serialNumber,
              vda_version, vda_manufacturer);
          if (v.contains("length")) {
            veh->length = v["length"].as_integer<int>();
          }
          if (v.contains("maxReverseVelocity")) {
            veh->max_reverse_vel = v["maxReverseVelocity"].as_integer<int>();
          }
          if (v.contains("maxVelocity")) {
            veh->max_vel = v["maxVelocity"].as_integer<int>();
          }
          veh->map_id = model["name"].as_string();
          if (v.contains("layout")) {
            veh->color = v["layout"]["routeColor"].as_string();
          }
          if (v.contains("energyLevelCritical")) {
            veh->energy_level_critical =
                v["energyLevelCritical"].as_integer<int>();
          }
          if (v.contains("energyLevelGood")) {
            veh->energy_level_good = v["energyLevelGood"].as_integer<int>();
          }
          if (v.contains("energyLevelFullyRecharged")) {
            veh->engrgy_level_full =
                v["energyLevelFullyRecharged"].as_integer<int>();
          }
          if (v.contains("energyLevelSufficientlyRecharged")) {
            veh->engrgy_level_recharge =
                v["energyLevelSufficientlyRecharged"].as_integer<int>();
          }
          veh->broker_ip = MQTT_IP;
          veh->broker_port = MQTT_PORT;
          veh->send_queue_size = orderquence;
          veh->deviation_xy = deviationXY;
          veh->deviation_theta = deviationTheta;
          veh->envelope_key =
              v.contains("envelopeKey") ? v["envelopeKey"].as_string() : "";
          if (v.contains("properties")) {
            for (auto &pro : v["properties"].array_range()) {
              veh->properties.insert(std::pair<std::string, std::string>(
                  pro["name"].as_string(), pro["value"].as_string()));
            }
          }
          ///////////////////////////
          dispatcher->vehicles.push_back(veh);
        } else {
          auto veh = std::make_shared<kernel::driver::InvalidVehicle>(
              v["name"].as_string());
          if (v.contains("length")) veh->length = v["length"].as_integer<int>();
          if (v.contains("maxReverseVelocity")) {
            veh->max_reverse_vel = v["maxReverseVelocity"].as_integer<int>();
          }
          if (v.contains("maxVelocity")) {
            veh->max_vel = v["maxVelocity"].as_integer<int>();
          }
          if (v.contains("layout")) {
            veh->color = v["layout"]["routeColor"].as_string();
          }
          if (v.contains("energyLevelCritical")) {
            veh->energy_level_critical =
                v["energyLevelCritical"].as_integer<int>();
          }
          if (v.contains("energyLevelGood")) {
            veh->energy_level_good = v["energyLevelGood"].as_integer<int>();
          }
          if (v.contains("energyLevelFullyRecharged")) {
            veh->engrgy_level_full =
                v["energyLevelFullyRecharged"].as_integer<int>();
          }
          if (v.contains("energyLevelSufficientlyRecharged")) {
            veh->engrgy_level_recharge =
                v["energyLevelSufficientlyRecharged"].as_integer<int>();
          }
          veh->send_queue_size = orderquence;
          veh->envelope_key =
              v.contains("envelopeKey") ? v["envelopeKey"].as_string() : "";
          dispatcher->vehicles.push_back(veh);
        }
      }
      CLOG(INFO, rss_log) << "init vehicle size "
                          << dispatcher->vehicles.size();
    }
    CLOG(INFO, rss_log) << "init resource ok";
    init_planner();
    scheduler->resource = resource;
    // connect signals
    dispatcher->find_res = std::bind(&kernel::allocate::ResourceManager::find,
                                     resource, std::placeholders::_1);
    dispatcher->go_home = std::bind(
        &RSS::home_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->order_empty =
        std::bind(&kernel::allocate::OrderPool::is_empty, orderpool);
    dispatcher->get_next_ord =
        std::bind(&kernel::allocate::OrderPool::pop, orderpool);
    dispatcher->go_charge = std::bind(
        &RSS::charge_order, this, std::placeholders::_1, std::placeholders::_2);
    dispatcher->get_park_point =
        std::bind(&kernel::allocate::ResourceManager::get_recent_park_point,
                  resource, std::placeholders::_1);
    for (auto &v : dispatcher->vehicles) {
      v->planner = planner;
      v->scheduler = scheduler;
      v->orderpool = orderpool;
      v->resource = resource;
    }
    assert(dispatcher);
    CLOG(INFO, rss_log) << "run all ...\n";
    run();
    CLOG(INFO, rss_log) << "run all ok\n";
    return std::pair<int, std::string>(httplib::OK_200, "");
  } catch (jsoncons::not_an_object ec) {
    CLOG(ERROR, rss_log) << "not_an_object error: " << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  } catch (jsoncons::ser_error ec) {
    CLOG(ERROR, rss_log) << "ser_error error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  } catch (jsoncons::key_not_found ec) {
    CLOG(ERROR, rss_log) << "key_not_found error :" << ec.what();
    json res = json::array();
    auto msg = "Could not parse JSON input '" + std::string(ec.what()) + "'.";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::BadRequest_400,
                                       res.to_string());
  }
}

std::pair<int, std::string> RSS::get_view() {
  std::shared_lock<std::shared_mutex> lk(mutex);

  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  auto r = get_vehicles_step();
  return std::pair<int, std::string>(httplib::OK_200, r);
}

std::string RSS::get_vehicles_step() {
  json value = json::array();
  for (auto &v : dispatcher->vehicles) {
    json veh;
    veh["name"] = v->name;
    veh["color"] = v->color;
    veh["step"] = json::array();
    veh["position"]["x"] = v->position.x();
    veh["position"]["y"] = v->position.y();
    veh["envelope"]["type"] = v->envelope_key;
    veh["envelope"]["vertex"] = json::array();
    veh["allocatedResources"] = json::array();
    for (auto &x : v->allocated_resources) {
      for (auto &x_ : x) {
        veh["allocatedResources"].push_back(x_->name);
      }
    }
    if (v->current_order) {
      if (v->current_order->state ==
          data::order::TransportOrder::State::BEING_PROCESSED) {
        for (auto &dr : v->current_order->driverorders) {
          if (dr->state == data::order::DriverOrder::State::TRAVELLING) {
            // current first
            auto step_ = dr->route->current_step;
            if (step_) {  // LOG(INFO) << "++++++++++++++++++" << step_->name;
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
              step["type"] = data::model::Path::get_connect_type(
                  step_->path->layout.connect_type);
              step["control_point"] = json::array();
              for (auto &ctrl : step_->path->layout.control_points) {
                json ctrl_point;
                ctrl_point["x"] = ctrl.x();
                ctrl_point["y"] = ctrl.y();
                step["control_point"].push_back(ctrl_point);
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
              step["type"] = data::model::Path::get_connect_type(
                  path->path->layout.connect_type);
              step["control_point"] = json::array();
              for (auto &ctrl : path->path->layout.control_points) {
                json ctrl_point;
                ctrl_point["x"] = ctrl.x();
                ctrl_point["y"] = ctrl.y();
                step["control_point"].push_back(ctrl_point);
              }
              veh["step"].push_back(step);
            }
            // location
            auto t = resource->find(dr->destination->destination.lock()->name);
            if (t.first ==
                kernel::allocate::ResourceManager::ResType::Location) {
              veh["destination"]["op"] = dr->destination->get_type();
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
  return value.to_string();
}

std::pair<int, std::string> RSS::put_path_locked(const std::string &path_name,
                                                 bool new_value) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find path '" + path_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::put_location_locked(
    const std::string &loc_name, bool new_value) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &x : resource->locations) {
    if (x->name == loc_name) {
      x->locked = new_value;
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "Could not find location  '" + loc_name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

void RSS::reroute() {
  for (auto &v : dispatcher->vehicles) {
    if (v->current_order) {
      v->reroute();
    }
  }
}

std::pair<int, std::string> RSS::post_reroute() {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  reroute();
  return std::pair<int, std::string>(httplib::OK_200, "");
}
bool RSS::is_connect(std::shared_ptr<data::model::Point> a,
                     std::shared_ptr<data::model::Point> b) {
  return planner->find_paths(a, b).size() > 0;
}

std::pair<int, std::string> RSS::post_vehicle_reroute(const std::string &name,
                                                      bool f) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == name) {
      v->reroute();
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "no vehicle named '" + name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::put_vehicle_enable(const std::string &name,
                                                    bool p) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
  }
  for (auto &v : dispatcher->vehicles) {
    if (v->name == name) {
      if (!p) {
        v->state = kernel::driver::Vehicle::State::UNAVAILABLE;
      } else {
        v->state = kernel::driver::Vehicle::State::UNKNOWN;
      }
      return std::pair<int, std::string>(httplib::OK_200, "");
    }
  }
  json res = json::array();
  auto msg = "no vehicle named '" + name + "'.";
  res.push_back(msg);
  return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
}

std::pair<int, std::string> RSS::put_vehicle_integration_level(
    const std::string &name, const std::string &p) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
  return std::pair<int, std::string>(httplib::OK_200, "");
}

std::pair<int, std::string> RSS::post_vehicle_path_to_point(
    const std::string &name, const std::string &p_) {
  std::shared_lock<std::shared_mutex> lk(mutex);
  if (!is_run) {
    json res = json::array();
    auto msg = "TCS is not running";
    res.push_back(msg);
    return std::pair<int, std::string>(httplib::NotFound_404, res.to_string());
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
      for (auto &e_p : body["destinationPoints"].array_range()) {
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
      return std::pair<int, std::string>(httplib::OK_200, res.to_string());
    }
  }
  return std::pair<int, std::string>(httplib::BadRequest_400,
                                     "the vehicle does not exist");
}
bool RSS::is_exist_active_order() {
  for (auto &x : orderpool->ended_orderpool) {
    if (x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      return true;
  }
  return false;
}