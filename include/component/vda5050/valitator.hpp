#ifndef VALIDATOR_VDA5050_HPP
#define VALIDATOR_VDA5050_HPP
#include "../../3rdparty/jsoncons_ext/jsonschema/jsonschema.hpp"
namespace vda5050 {
class SchemaValidator {
 public:
  static std::vector<std::string> validate(
      jsoncons::json obj,
      std::shared_ptr<jsoncons::jsonschema::json_schema<jsoncons::json>>
          schema) {
    std::vector<std::string> errors;
    try {
      auto reporter =
          [&errors](const jsoncons::jsonschema::validation_output& o) {
            auto t = o.instance_location() + ": " + o.message();
            errors.push_back(t);
          };
      jsoncons::jsonschema::json_validator<jsoncons::json> vad(schema);
      vad.validate(obj, reporter);
      return errors;
    } catch (std::exception& ec) {
      errors.push_back(std::string{"std::exception: "} + ec.what());
      return errors;
    }
  }
};
static std::string state_sch = R"(
{
    "$schema": "https://json-schema.org/draft/2019-09/schema",
    "title": "state",
    "description": "all encompassing state of the AGV.",
    "subtopic": "/state",
    "type": "object",
    "required": [
        "headerId",
        "timestamp",
        "version",
        "manufacturer",
        "serialNumber",
        "orderId",
        "orderUpdateId",
        "lastNodeId",
        "lastNodeSequenceId",
        "nodeStates",
        "edgeStates",
        "driving",
        "actionStates",
        "batteryState",
        "operatingMode",
        "errors",
        "safetyState"
    ],
    "properties": {
        "headerId": {
            "type": "integer",
            "description": "headerId of the message. The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message."
        },
        "timestamp": {
            "type": "string",
            "format": "date-time",
            "description": "Timestamp in ISO8601 format.",
            "examples": [
                "1991-03-11T11:40:03.12Z"
            ]
        },
        "version": {
            "type": "string",
            "description": "Version of the protocol [Major].[Minor].[Patch]",
            "examples": [
                "1.3.2"
            ]
        },
        "manufacturer": {
            "type": "string",
            "description": "Manufacturer of the AGV"
        },
        "serialNumber": {
            "type": "string",
            "description": "Serial number of the AGV."
        },
        "orderId": {
            "type": "string",
            "description": "Unique order identification of the current order or the previous finished order. The orderId is kept until a new order is received. Empty string (â€œâ€) if no previous orderId is available. "
        },
        "orderUpdateId": {
            "type": "integer",
            "description": "Order Update Identification to identify that an order update has been accepted by the AGV. â€œ0â€ if no previous orderUpdateId is available."
        },
        "zoneSetId": {
            "type": "string",
            "description": "Unique ID of the zone set that the AGV currently uses for path planning. Must be the same as the one used in the order, otherwise the AGV is to reject the order.\nOptional: If the AGV does not use zones, this field can be omitted."
        },
        "lastNodeId": {
            "type": "string",
            "description": "nodeID of last reached node or, if AGV is currently on a node, current node (e. g. â€œnode7â€). Empty string (â€œâ€) if no lastNodeId is available."
        },
        "lastNodeSequenceId": {
            "type": "integer",
            "description": "sequenceId of the last reached node or, if the AGV is currently on a node, sequenceId of current node.\nâ€œ0â€ if no lastNodeSequenceId is available. "
        },
        "driving": {
            "type": "boolean",
            "description": "True: indicates that the AGV is driving and/or rotating. Other movements of the AGV (e.g. lift movements) are not included here.\nFalse: indicates that the AGV is neither driving nor rotating "
        },
        "paused": {
            "type": "boolean",
            "description": "True: AGV is currently in a paused state, either because of the push of a physical button on the AGV or because of an instantAction. The AGV can resume the order.\nFalse: The AGV is currently not in a paused state."
        },
        "newBaseRequest": {
            "type": "boolean",
            "description": "True: AGV is almost at the end of the base and will reduce speed if no new base is transmitted. Trigger for MC to send ne base\nFalse: no base update required "
        },
        "distanceSinceLastNode": {
            "type": "number",
            "description": "Used by line guided vehicles to indicate the distance it has been driving past the â€žlastNodeIdâ€œ.\nDistance is in meters"
        },
        "operatingMode": {
            "type": "string",
            "description": "Current operating mode of the AGV.",
            "enum": [
                "AUTOMATIC",
                "SEMIAUTOMATIC",
                "MANUAL",
                "SERVICE",
                "TEACHIN"
            ]
        },
        "nodeStates": {
            "type": "array",
            "description": "Information about the nodes the AGV still has to drive over. Empty list if idle.",
            "items": {
                "type": "object",
                "title": "nodeState",
                "required": [
                    "nodeId",
                    "released",
                    "sequenceId"
                ],
                "properties": {
                    "nodeId": {
                        "type": "string",
                        "description": "Unique node identification"
                    },
                    "sequenceId": {
                        "type": "integer",
                        "description": "sequenceId of the node."
                    },
                    "nodeDescription": {
                        "type": "string",
                        "description": "Verbose node description"
                    },
                    "nodePosition": {
                        "type": "object",
                        "required": [
                            "x",
                            "y",
                            "theta",
                            "mapId"
                        ],
                        "description": "Node position. The object is defined in chapter 5.4 Topic: Order (from master control to AGV).\nOptional:\nMaster control has this information. Can be sent additionally, e. g. for debugging purposes. ",
                        "properties": {
                            "x": {
                                "type": "number"
                            },
                            "y": {
                                "type": "number"
                            },
                            "theta": {
                                "type": "number"
                            },
                            "mapId": {
                                "type": "string"
                            }
                        }
                    },
                    "released": {
                        "type": "boolean",
                        "description": "True: indicates that the node is part of the base. False: indicates that the node is part of the horizon."
                    }
                }
            }
        },
        "edgeStates": {
            "type": "array",
            "description": "Information about the edges the AGV still has to drive over. Empty list if the AGV is idle.",
            "items": {
                "type": "object",
                "required": [
                    "edgeId",
                    "sequenceId",
                    "released"
                ],
                "properties": {
                    "edgeId": {
                        "type": "string",
                        "description": "Unique edge identification"
                    },
                    "sequenceId": {
                        "type": "integer",
                        "description": "sequenceId of the edge."
                    },
                    "edgeDescription": {
                        "type": "string",
                        "description": "Verbose Edge description"
                    },
                    "released": {
                        "type": "boolean",
                        "description": "True: Edge is part of base. False: Edge is part of horizon."
                    },
                    "trajectory": {
                        "type": "object",
                        "description": "The trajectory is to be communicated as a NURBS and is defined in chapter 5.4 Topic: Order (from master control to AGV).\nTrajectory segments are from the point where the AGV starts to enter the edge until the point where it reports that the next node was traversed. ",
                        "required": [
                            "degree",
                            "knotVector",
                            "controlPoints"
                        ],
                        "properties": {
                            "degree": {
                                "type": "integer",
                                "description": "The degree of the NURBS."
                            },
                            "knotVector": {
                                "type": "array",
                                "description": "Sequence of parameter values that determine where and how the control points affect the NURBS curve. knotVector has size of number of control points + degree + 1",
                                "items": {
                                    "type": "number",
                                    "maximum": 1.0,
                                    "minimum": 0.0
                                }
                            },
                            "controlPoints": {
                                "type": "array",
                                "description": "List of JSON controlPoint objects defining the control points of the NURBS. This includes the start and end point.",
                                "items": {
                                    "type": "object",
                                    "required": [
                                        "x",
                                        "y",
                                        "weight"
                                    ],
                                    "properties": {
                                        "x": {
                                            "type": "number"
                                        },
                                        "y": {
                                            "type": "number"
                                        },
                                        "weight": {
                                            "type": "number",
                                            "description": "The weight with which this control point pulls no the curve.\nWhen not defined, the default will be 1.0."
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        },
        "agvPosition": {
            "type": "object",
            "required": [
                "x",
                "y",
                "theta",
                "mapId",
                "positionInitialized"
            ],
            "description": "The AGVs position",
            "properties": {
                "x": {
                    "type": "number"
                },
                "y": {
                    "type": "number"
                },
                "theta": {
                    "type": "number"
                },
                "mapId": {
                    "type": "string"
                },
                "mapDescription": {
                    "type": "string"
                },
                "positionInitialized": {
                    "type": "boolean",
                    "description": "True if the AGVs position is initialized, false, if position is not initizalized."
                },
                "localizationScore": {
                    "type": "number",
                    "description": "Describes the quality of the localization and therefore, can be used e. g. by SLAM-AGVs to describe how accurate the current position information is.\n0.0: position unknown\n1.0: position known\nOptional for vehicles that cannot estimate their localization score.\nOnly for logging and visualization purposes",
                    "minimum": 0.0,
                    "maximum": 1.0
                },
                "deviationRange": {
                    "type": "number",
                    "description": "Value for position deviation range in meters. Can be used if the AGV is able to derive it."
                }
            }
        },
        "velocity": {
            "type": "object",
            "description": "The AGVs velocity in vehicle coordinates",
            "properties": {
                "vx": {
                    "type": "number"
                },
                "vy": {
                    "type": "number"
                },
                "omega": {
                    "type": "number"
                }
            }
        },
        "loads": {
            "type": "array",
            "description": "Array for information about the loads that an AGV currently carries, if the AGV has any information about them. This array is optional: if an AGV cannot reason about its load state, it shall not send this field. If an empty field is sent, MC is to assume that the AGV can reason about its load state and that the AGV currently does not carry a load.",
            "items": {
                "type": "object",
                "required": [],
                "description": "Load object that describes the load if the AGV has information about it.",
                "title": "load",
                "properties": {
                    "loadId": {
                        "type": "string",
                        "description": "Unique identification number of the load (e. g. barcode or RFID)\nEmpty field if the AGV can identify the load but didnâ€™t identify the load yet.\nOptional if the AGV has cannot identify the load."
                    },
                    "loadType": {
                        "type": "string"
                    },
                    "loadPosition": {
                        "type": "string",
                        "description": "Indicates which load handling/carrying unit of the AGV is used, e. g. in case the AGV has multiple spots/positions to carry loads.\nFor example: â€œfrontâ€, â€œbackâ€, â€œpositionC1â€, etc.\nOptional for vehicles with only one loadPosition."
                    },
                    "boundingBoxReference": {
                        "type": "object",
                        "required": [
                            "x",
                            "y",
                            "z"
                        ],
                        "description": "This point describes the loads position on the AGV in the vehicle coordinates. The boundingBoxReference point is in the middle of the footprint of the load, so length/2 and width/2.",
                        "properties": {
                            "x": {
                                "type": "number"
                            },
                            "y": {
                                "type": "number"
                            },
                            "z": {
                                "type": "number"
                            },
                            "theta": {
                                "type": "number"
                            }
                        }
                    },
                    "loadDimensions": {
                        "type": "object",
                        "required": [
                            "length",
                            "width"
                        ],
                        "description": "Dimensions of the loadâ€™s bounding box in meters. ",
                        "properties": {
                            "length": {
                                "type": "number",
                                "description": "Absolute length of the loads bounding box in meter."
                            },
                            "width": {
                                "type": "number",
                                "description": "Absolute width of the loads bounding box in meter."
                            },
                            "height": {
                                "type": "number",
                                "description": "Absolute height of the loads bounding box in meter.\nOptional:\nSet value only if known."
                            }
                        }
                    },
                    "weight": {
                        "type": "number",
                        "description": "Weight of load in kg"
                    }
                }
            }
        },
        "actionStates": {
            "type": "array",
            "description": "Contains a list of the current actions and the actions which are yet to be finished. This may include actions from previous nodes that are still in progress\nWhen an action is completed, an updated state message is published with actionStatus set to finished and if applicable with the corresponding resultDescription. The actionStates are kept until a new order is received.",
            "items": {
                "type": "object",
                "required": [
                    "actionId",
                    "actionStatus"
                ],
                "title": "actionState",
                "properties": {
                    "actionId": {
                        "type": "string",
                        "description": "Unique actionId",
                        "examples": [
                            "blink_123jdaimoim234"
                        ]
                    },
                    "actionType": {
                        "type": "string",
                        "description": "actionType of the action.\nOptional: Only for informational or visualization purposes. Order knows the type."
                    },
                    "actionDescription": {
                        "type": "string",
                        "description": "Additional information on the action."
                    },
                    "actionStatus": {
                        "type": "string",
                        "description": "WAITING: waiting for trigger\nFAILED: action could not be performed.",
                        "enum": [
                            "WAITING",
                            "INITIALIZING",
                            "RUNNING",
                            "FINISHED",
                            "FAILED"
                        ]
                    },
                    "resultDescription": {
                        "type": "string",
                        "description": "Description of the result, e.g. the result of a rfid-read."
                    }
                }
            }
        },
        "batteryState": {
            "type": "object",
            "required": [
                "batteryCharge",
                "charging"
            ],
            "description": "All information relating to the battery.",
            "properties": {
                "batteryCharge": {
                    "type": "number",
                    "description": "State of Charge:\nIf AGV only provides values for good or bad battery levels, these will be indicated as 20% (bad) and 80% (good)."
                },
                "batteryVoltage": {
                    "type": "number",
                    "description": "Battery voltage"
                },
                "batteryHealth": {
                    "type": "integer",
                    "description": "State of health in percent."
                },
                "charging": {
                    "type": "boolean",
                    "description": "If true: Charging in progress."
                },
                "reach": {
                    "type": "integer",
                    "description": "estimated reach with actual State of Charge"
                }
            }
        },
        "errors": {
            "type": "array",
            "description": "Array of errors. Errors are kept until resolution.",
            "items": {
                "type": "object",
                "required": [
                    "errorType",
                    "errorLevel"
                ],
                "title": "Error",
                "properties": {
                    "errorType": {
                        "type": "string",
                        "description": "Type of error."
                    },
                    "errorReferences": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "title": "errorReference",
                            "description": "Object that holds the error reference (e.g. orderId, orderUpdateId, actionId...) as key-value pairs.",
                            "properties": {
                                "referenceKey": {
                                    "type": "string"
                                },
                                "referenceValue": {
                                    "type": "string"
                                }
                            },
                            "required": [
                                "referenceKey",
                                "referenceValue"
                            ]
                        }
                    },
                    "errorDescription": {
                        "type": "string",
                        "description": "Verbose description of error."
                    },
                    "errorLevel": {
                        "type": "string",
                        "description": "warning: AGV is ready to drive without human intervention \n fatal: AGV is not in running condition.",
                        "enum": [
                            "WARNING",
                            "FATAL"
                        ]
                    }
                }
            }
        },
        "information": {
            "type": "array",
            "description": "Array of information messages. Messages are only for visualization/debugging. There's no specification when these messages are deleted.",
            "items": {
                "type": "object",
                "required": [
                    "infoType",
                    "infoLevel"
                ],
                "properties": {
                    "infoType": {
                        "type": "string",
                        "description": "Type of Information."
                    },
                    "infoReferences": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "required": [
                                "referenceKey",
                                "referenceValue"
                            ],
                            "title": "infoReference",
                            "description": "Object that holds the info reference (e.g. orderId, orderUpdateId, actionId...) as key-value pairs.",
                            "properties": {
                                "referenceKey": {
                                    "type": "string"
                                },
                                "referenceValue": {
                                    "type": "string"
                                }
                            }
                        }
                    },
                    "infoDescription": {
                        "type": "string",
                        "description": "Verbose description of error."
                    },
                    "infoLevel": {
                        "type": "string",
                        "description": "Type of information",
                        "enum": [
                            "INFO",
                            "DEBUG"
                        ]
                    }
                }
            }
        },
        "safetyState": {
            "type": "object",
            "required": [
                "eStop",
                "fieldViolation"
            ],
            "description": "Object that holds information about the safety status",
            "properties": {
                "eStop": {
                    "type": "string",
                    "description": "autoAck, manual, remote, none",
                    "enum": [
                        "AUTOACK",
                        "MANUAL",
                        "REMOTE",
                        "NONE"
                    ]
                },
                "fieldViolation": {
                    "type": "boolean",
                    "description": "Protective field violation"
                }
            }
        }
    }
}
  )";
static jsoncons::json state_schema = jsoncons::json::parse(state_sch);

static std::string con_sch = R"(
{
    "$schema": "https://json-schema.org/draft/2019-09/schema",
    "title": "connection",
    "description": "The last will message of the AGV. Has to be sent with retain flag.\nOnce the AGV comes online, it has to send this message on its connect topic, with the connectionState enum set to \"ONLINE\".\n The last will message is to be configured with the connection state set to \"CONNECTIONBROKEN\".\nThus, if the AGV disconnects from the broker, master control gets notified via the topic \"connection\".\nIf the AGV is disconnecting in an orderly fashion (e.g. shutting down, sleeping), the AGV is to publish a message on this topic with the connectionState set to \"DISCONNECTED\".",
    "subtopic": "/connection",
    "type": "object",
    "required": [
        "headerId",
        "timestamp",
        "version",
        "manufacturer",
        "serialNumber",
        "connectionState"
    ],
    "properties": {
        "headerId": {
            "type": "integer",
            "description": "headerId of the message. The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message."
        },
        "timestamp": {
            "type": "string",
            "format": "date-time",
            "description": "Timestamp in ISO8601 format.",
            "examples": [
                "1991-03-11T11:40:03.12Z"
            ]
        },
        "version": {
            "type": "string",
            "description": "Version of the protocol [Major].[Minor].[Patch]",
            "examples": [
                "1.3.2"
            ]
        },
        "manufacturer": {
            "type": "string",
            "description": "Manufacturer of the AGV"
        },
        "serialNumber": {
            "type": "string",
            "description": "Serial number of the AGV."
        },
        "connectionState": {
            "type": "string",
            "enum": [
                "ONLINE",
                "OFFLINE",
                "CONNECTIONBROKEN"
            ],
            "description": "online: connection between AGV and broker is active.\noffline: connection between AGV and broker has gone offline in a coordinated way.\nconnectionBroken: The connection between AGV and broker has unexpectedly ended."
        }
    }
}
  )";
static jsoncons::json con_schema = jsoncons::json::parse(con_sch);

static std::string ord_sch = R"(
{
    "$schema": "https://json-schema.org/draft/2019-09/schema",
    "title": "Order Message",
    "description": "The message schema to communicate orders from master control to the AGV.",
    "subtopic": "/order",
    "type": "object",
    "required": [
        "headerId",
        "timestamp",
        "version",
        "manufacturer",
        "serialNumber",
        "orderId",
        "orderUpdateId",
        "nodes",
        "edges"
    ],
    "properties": {
        "headerId": {
            "type": "integer",
            "description": "headerId of the message. The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message."
        },
        "timestamp": {
            "type": "string",
            "format": "date-time",
            "description": "Timestamp in ISO8601 format (YYYY-MM-DDTHH:mm:ss.ssZ).",
            "examples": [
                "1991-03-11T11:40:03.12Z"
            ]
        },
        "version": {
            "type": "string",
            "description": "Version of the protocol [Major].[Minor].[Patch]",
            "examples": [
                "1.3.2"
            ]
        },
        "manufacturer": {
            "type": "string",
            "description": "Manufacturer of the AGV"
        },
        "serialNumber": {
            "type": "string",
            "description": "Serial number of the AGV."
        },
        "orderId": {
            "description": "Order Identification. This is to be used to identify multiple order messages that belong to the same order.",
            "type": "string"
        },
        "orderUpdateId": {
            "description": "orderUpdate identification. Is unique per orderId. If an order update is rejected, this field is to be passed in the rejection message.",
            "type": "integer",
            "minimum": 0
        },
        "zoneSetId": {
            "description": "Unique identifier of the zone set that the AGV has to use for navigation or that was used by MC for planning.\nOptional: Some MC systems do not use zones. Some AGVs do not understand zones. Do not add to message if no zones are used.",
            "type": "string"
        },
        "nodes": {
            "description": "Array of nodes objects to be traversed for fulfilling the order. One node is enough for a valid order. Leave edge list empty for that case.",
            "type": "array",
            "items": {
                "type": "object",
                "title": "node",
                "required": [
                    "nodeId",
                    "sequenceId",
                    "released",
                    "actions"
                ],
                "properties": {
                    "nodeId": {
                        "type": "string",
                        "description": "Unique node identification",
                        "examples": [
                            "pumpenhaus_1",
                            "MONTAGE"
                        ]
                    },
                    "sequenceId": {
                        "type": "integer",
                        "minimum": 0,
                        "description": "Number to track the sequence of nodes and edges in an order and to simplify order updates.\nThe main purpose is to distinguish between a node which is passed more than once within one orderId. The variable sequenceId runs across all nodes and edges of the same order and is reset when a new orderId is issued."
                    },
                    "nodeDescription": {
                        "type": "string",
                        "description": "Additional information on the node."
                    },
                    "released": {
                        "type": "boolean",
                        "description": "True indicates that the node is part of the base. False indicates that the node is part of the horizon."
                    },
                    "nodePosition": {
                        "description": "Defines the position on a map in world coordinates. Each floor has its own map. All maps must use the same project specific global origin. \nOptional for vehicle-types that do not require the node position (e.g., line-guided vehicles).",
                        "type": "object",
                        "required": [
                            "x",
                            "y",
                            "mapId"
                        ],
                        "properties": {
                            "x": {
                                "type": "number",
                                "description": "X-position on the map in reference to the map coordinate system. Precision is up to the specific implementation."
                            },
                            "y": {
                                "type": "number",
                                "description":"Y-position on the map in reference to the map coordinate system. Precision is up to the specific implementation."
                            },
                            "theta": {
                                "type": "number",
                                "description": "Absolute orientation of the AGV on the node. \nOptional: vehicle can plan the path by itself.\nIf defined, the AGV has to assume the theta angle on this node. If previous edge disallows rotation, the AGV must rotate on the node. If following edge has a differing orientation defined but disallows rotation, the AGV is to rotate on the node to the edges desired rotation before entering the edge.",
                                "minimum": -3.14159265359,
                                "maximum": 3.14159265359
                            },
                            "allowedDeviationXy": {
                                "type": "number",
                                "description": "Indicates how exact an AGV has to drive over a node in order for it to count as traversed.\nIf = 0: no deviation is allowed (no deviation means within the normal tolerance of the AGV manufacturer).\nIf > 0: allowed deviation-radius in meters. If the AGV passes a node within the deviation-radius, the node is considered to have been traversed.",
                                "minimum": 0
                            },
                            "allowedDeviationTheta": {
                                "type": "number",
                                "minimum": -3.141592654,
                                "maximum": 3.141592654,
                                "description": "Indicates how big the deviation of theta angle can be. \nThe lowest acceptable angle is theta - allowedDeviationTheta and the highest acceptable angle is theta + allowedDeviationTheta."
                            },
                            "mapId": {
                                "description": "Unique identification of the map in which the position is referenced.\nEach map has the same origin of coordinates. When an AGV uses an elevator, e.g., leading from a departure floor to a target floor, it will disappear off the map of the departure floor and spawn in the related lift node on the map of the target floor.",
                                "type": "string"
                            },
                            "mapDescription": {
                                "description": "Additional information on the map.",
                                "type": "string"
                            }
                        }
                    },
                    "actions": {
                        "description": "Array of actions to be executed on a node. Empty array, if no actions required.",
                        "type": "array",
                        "items": {
                            "$ref": "#/definitions/action"
                        }
                    }
                }
            }
        },
        "edges": {
            "type": "array",
            "description": "Directional connection between two nodes. Array of edge objects to be traversed for fulfilling the order. One node is enough for a valid order. Leave edge list empty for that case.",
            "items": {
                "type": "object",
                "title": "edge",
                "required": [
                    "edgeId",
                    "sequenceId",
                    "released",
                    "startNodeId",
                    "endNodeId",
                    "actions"
                ],
                "properties": {
                    "edgeId": {
                        "type": "string",
                        "description": "Unique edge identification"
                    },
                    "sequenceId": {
                        "type": "integer",
                        "minimum": 0,
                        "description": "Number to track the sequence of nodes and edges in an order and to simplify order updates. The variable sequenceId runs across all nodes and edges of the same order and is reset when a new orderId is issued."
                    },
                    "edgeDescription": {
                        "type": "string",
                        "description": "Additional information on the edge."
                    },
                    "released": {
                        "type": "boolean",
                        "description": "True indicates that the edge is part of the base. False indicates that the edge is part of the horizon."
                    },
                    "startNodeId": {
                        "type": "string",
                        "description": "The nodeId of the start node."
                    },
                    "endNodeId": {
                        "type": "string",
                        "description": "The nodeId of the end node."
                    },
                    "maxSpeed": {
                        "type": "number",
                        "description": "Permitted maximum speed on the edge in m/s. Speed is defined by the fastest measurement of the vehicle."
                    },
                    "maxHeight": {
                        "type": "number",
                        "description": "Permitted maximum height of the vehicle, including the load, on edge in meters."
                    },
                    "minHeight": {
                        "type": "number",
                        "description": "Permitted minimal height of the load handling device on the edge in meters"
                    },
                    "orientation": {
                        "type": "number",
                        "description": "Orientation of the AGV on the edge. The value orientationType defines if it has to be interpreted relative to the global project specific map coordinate system or tangential to the edge. In case of interpreted tangential to the edge 0.0 = forwards and PI = backwards. Example: orientation Pi/2 rad will lead to a rotation of 90 degrees. \nIf AGV starts in different orientation, rotate the vehicle on the edge to the desired orientation if rotationAllowed is set to True. If rotationAllowed is False, rotate before entering the edge. If that is not possible, reject the order. \nIf no trajectory is defined, apply the rotation to the direct path between the two connecting nodes of the edge. If a trajectory is defined for the edge, apply the orientation to the trajectory.",
                        "minimum": -3.14159265359,
                        "maximum": 3.14159265359
                    },
                    "orientationType":{
                        "type": "string",
                        "description": "Enum {GLOBALGLOBAL, TANGENTIALTANGENTIAL}: \n\"GLOBAL\"- relative to the global project specific map coordinate system; \n\"TANGENTIAL\"- tangential to the edge. \nIf not defined, the default value is \"TANGENTIAL\"."
                    },
                    "direction": {
                        "type": "string",
                        "description": "Sets direction at junctions for line-guided or wire-guided vehicles, to be defined initially (vehicle-individual).",
                        "examples": [
                            "left",
                            "right",
                            "straight",
                            "433MHz"
                        ]
                    },
                    "rotationAllowed": {
                        "type": "boolean",
                        "description": "True: rotation is allowed on the edge. False: rotation is not allowed on the edge. \nOptional: No limit, if not set."
                    },
                    "maxRotationSpeed": {
                        "type": "number",
                        "description": "Maximum rotation speed in rad/s. \nOptional: No limit, if not set."
                    },
                    "length": {
                        "type": "number",
                        "description": "Distance of the path from startNode to endNode in meters. \nOptional: This value is used by line-guided AGVs to decrease their speed before reaching a stop position."
                    },
                    "trajectory": {
                        "type": "object",
                        "description": "Trajectory JSON-object for this edge as a NURBS. Defines the curve, on which the AGV should move between startNode and endNode. \nOptional: Can be omitted, if AGV cannot process trajectories or if AGV plans its own trajectory.",
                        "required": [
                            "degree",
                            "knotVector",
                            "controlPoints"
                        ],
                        "properties": {
                            "degree": {
                                "type": "integer",
                                "description": "Defines the number of control points that influence any given point on the curve. Increasing the degree increases continuity. If not defined, the default value is 1.",
                                "minimum": 1
                            },
                            "knotVector": {
                                "type": "array",
                                "description": "Sequence of parameter values that determines where and how the control points affect the NURBS curve. knotVector has size of number of control points + degree + 1.",
                                "items": {
                                    "type": "number",
                                    "maximum": 1,
                                    "minimum": 0
                                }
                            },
                            "controlPoints": {
                                "type": "array",
                                "description": "List of JSON controlPoint objects defining the control points of the NURBS, which includes the beginning and end point.",
                                "items": {
                                    "type": "object",
                                    "title": "controlPoint",
                                    "properties": {
                                        "x": {
                                            "type": "number",
                                            "description": "X coordinate described in the world coordinate system."
                                        },
                                        "y": {
                                            "type": "number",
                                            "description": "Y coordinate described in the world coordinate system."
                                        },
                                        "weight": {
                                            "type": "number",
                                            "minimum": 0,
                                            "description": "The weight, with which this control point pulls on the curve. When not defined, the default will be 1.0."
                                        }
                                    },
                                    "required": [
                                        "x",
                                        "y"
                                    ]
                                }
                            }
                        }
                    },
                    "actions": {
                        "description": "Array of action objects with detailed information.",
                        "type": "array",
                        "items": {
                            "$ref": "#/definitions/action"
                        }
                    }
                }
            }
        }
    },
    "definitions": {
        "action": {
            "type": "object",
            "description": "Describes an action that the AGV can perform.",
            "required": [
                "actionId",
                "actionType",
                "blockingType"
            ],
            "properties": {
                "actionType": {
                    "type": "string",
                    "description": "Name of action as described in the first column of \"Actions and Parameters\". Identifies the function of the action."
                },
                "actionId": {
                    "type": "string",
                    "description": "Unique ID to identify the action and map them to the actionState in the state. Suggestion: Use UUIDs."
                },
                "actionDescription": {
                    "type": "string",
                    "description": "Additional information on the action."
                },
                "blockingType": {
                    "type": "string",
                    "description": "Regulates if the action is allowed to be executed during movement and/or parallel to other actions.\nnone: action can happen in parallel with others, including movement.\nsoft: action can happen simultaneously with others, but not while moving.\nhard: no other actions can be performed while this action is running.",
                    "enum": [
                        "NONE",
                        "SOFT",
                        "HARD"
                    ]
                },
                "actionParameters": {
                    "type": "array",
                    "description": "Array of actionParameter-objects for the indicated action e. g. deviceId, loadId, external Triggers.",
                    "items": {
                        "title": "actionParameter",
                        "type": "object",
                        "required": [
                            "key",
                            "value"
                        ],
                        "properties": {
                            "key": {
                                "type": "string",
                                "description": "The key of the action parameter.",
                                "examples": [
                                    "duration",
                                    "direction",
                                    "signal"
                                ]
                            },
                            "value": {
                                "type": [
                                    "array",
                                    "boolean",
                                    "number",
                                    "string"
                                ],
                                "description": "The value of the action parameter",
                                "examples": [
                                    103.2,
                                    "left",
                                    true,
                                    [
                                        "arrays",
                                        "are",
                                        "also",
                                        "valid"
                                    ]
                                ]
                            }
                        }
                    }
                }
            }
        }
    }
}
  )";

static jsoncons::json ord_schema = jsoncons::json::parse(ord_sch);

static std::string ins_sch = R"(
{
    "$schema": "https://json-schema.org/draft/2019-09/schema",
    "title": "instantActions",
    "description": "JSON Schema for publishing instantActions that the AGV is to execute as soon as they arrive.",
    "subtopic": "/instantActions",
    "type": "object",
    "properties": {
        "headerId": {
            "title": "headerId",
            "type": "integer",
            "description": "headerId of the message. The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message."
        },
        "timestamp": {
            "title": "timestamp",
            "type": "string",
            "format": "date-time",
            "description": "Timestamp in ISO8601 format (YYYY-MM-DDTHH:mm:ss.ssZ).",
            "examples": [
                "1991-03-11T11:40:03.12Z"
            ]
        },
        "version": {
            "title": "Version",
            "type": "string",
            "description": "Version of the protocol [Major].[Minor].[Patch]",
            "examples": [
                "1.3.2"
            ]
        },
        "manufacturer": {
            "type": "string",
            "description": "Manufacturer of the AGV"
        },
        "serialNumber": {
            "type": "string",
            "description": "Serial number of the AGV."
        },
        "actions": {
            "type": "array",
            "items": {
                "type": "object",
                "description": "Describes an action that the AGV can perform.",
                "required": [
                    "actionId",
                    "actionType",
                    "blockingType"
                ],
                "properties": {
                    "actionType": {
                        "type": "string",
                        "description": "Name of action as described in the first column of \"Actions and Parameters\". Identifies the function of the action."
                    },
                    "actionId": {
                        "type": "string",
                        "description": "Unique ID to identify the action and map them to the actionState in the state. Suggestion: Use UUIDs."
                    },
                    "actionDescription": {
                        "type": "string",
                        "description": "Additional information on the action."
                    },
                    "blockingType": {
                        "type": "string",
                        "description": "Regulates if the action is allowed to be executed during movement and/or parallel to other actions.\nnone: action can happen in parallel with others, including movement.\nsoft: action can happen simultaneously with others, but not while moving.\nhard: no other actions can be performed while this action is running.",
                        "enum": [
                            "NONE",
                            "SOFT",
                            "HARD"
                        ]
                    },
                    "actionParameters": {
                        "type": "array",
                        "description": "Array of actionParameter-objects for the indicated action e. g. deviceId, loadId, external Triggers.",
                        "items": {
                            "title": "actionParameter",
                            "type": "object",
                            "required": [
                                "key",
                                "value"
                            ],
                            "properties": {
                                "key": {
                                    "type": "string",
                                    "description": "The key of the action parameter.",
                                    "examples": [
                                        "duration",
                                        "direction",
                                        "signal"
                                    ]
                                },
                                "value": {
                                    "type": [
                                        "array",
                                        "boolean",
                                        "number",
                                        "string"
                                    ],
                                    "description": "The value of the action parameter",
                                    "examples": [
                                        103.2,
                                        "left",
                                        true,
                                        [
                                            "arrays",
                                            "are",
                                            "also",
                                            "valid"
                                        ]
                                    ]
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
  )";

static jsoncons::json ins_schema = jsoncons::json::parse(ins_sch);

static auto state_schema_ptr = jsoncons::jsonschema::make_schema(state_schema);
static auto con_schema_ptr = jsoncons::jsonschema::make_schema(con_schema);
static auto ord_schema_ptr = jsoncons::jsonschema::make_schema(ord_schema);
static auto ins_schema_ptr = jsoncons::jsonschema::make_schema(ins_schema);

static std::vector<std::string> state_validate(jsoncons::json src) {
  return SchemaValidator::validate(src, state_schema_ptr);
}

static std::vector<std::string> connection_validate(jsoncons::json src) {
  return SchemaValidator::validate(src, con_schema_ptr);
}

static std::vector<std::string> order_validate(jsoncons::json src) {
  return SchemaValidator::validate(src, ord_schema_ptr);
}

static std::vector<std::string> instantaction_validate(jsoncons::json src) {
  return SchemaValidator::validate(src, ins_schema_ptr);
}

}  // namespace vda5050
#endif
