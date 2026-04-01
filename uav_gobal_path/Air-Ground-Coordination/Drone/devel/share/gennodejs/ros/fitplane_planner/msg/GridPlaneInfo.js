// Auto-generated. Do not edit!

// (in-package fitplane_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GridPlaneInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.resolution = null;
      this.width = null;
      this.height = null;
      this.origin = null;
      this.traversability = null;
      this.plane_angle = null;
      this.plane_height = null;
      this.occupancy = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('traversability')) {
        this.traversability = initObj.traversability
      }
      else {
        this.traversability = [];
      }
      if (initObj.hasOwnProperty('plane_angle')) {
        this.plane_angle = initObj.plane_angle
      }
      else {
        this.plane_angle = [];
      }
      if (initObj.hasOwnProperty('plane_height')) {
        this.plane_height = initObj.plane_height
      }
      else {
        this.plane_height = [];
      }
      if (initObj.hasOwnProperty('occupancy')) {
        this.occupancy = initObj.occupancy
      }
      else {
        this.occupancy = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GridPlaneInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float32(obj.resolution, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.uint32(obj.height, buffer, bufferOffset);
    // Serialize message field [origin]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.origin, buffer, bufferOffset);
    // Serialize message field [traversability]
    bufferOffset = _arraySerializer.float32(obj.traversability, buffer, bufferOffset, null);
    // Serialize message field [plane_angle]
    bufferOffset = _arraySerializer.float32(obj.plane_angle, buffer, bufferOffset, null);
    // Serialize message field [plane_height]
    bufferOffset = _arraySerializer.float32(obj.plane_height, buffer, bufferOffset, null);
    // Serialize message field [occupancy]
    bufferOffset = _arraySerializer.int32(obj.occupancy, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GridPlaneInfo
    let len;
    let data = new GridPlaneInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [origin]
    data.origin = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [traversability]
    data.traversability = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [plane_angle]
    data.plane_angle = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [plane_height]
    data.plane_height = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [occupancy]
    data.occupancy = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.traversability.length;
    length += 4 * object.plane_angle.length;
    length += 4 * object.plane_height.length;
    length += 4 * object.occupancy.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fitplane_planner/GridPlaneInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16e811bb180035be79c09c3399b1506a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 resolution
    uint32 width
    uint32 height
    geometry_msgs/Point origin
    float32[] traversability  # 每个栅格的可通行性
    float32[] plane_angle    # 每个栅格的平面角度
    float32[] plane_height   # 每个栅格的高度
    int32[] occupancy       # 栅格占用状态：-1未知，0空闲，100占用 
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GridPlaneInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.origin !== undefined) {
      resolved.origin = geometry_msgs.msg.Point.Resolve(msg.origin)
    }
    else {
      resolved.origin = new geometry_msgs.msg.Point()
    }

    if (msg.traversability !== undefined) {
      resolved.traversability = msg.traversability;
    }
    else {
      resolved.traversability = []
    }

    if (msg.plane_angle !== undefined) {
      resolved.plane_angle = msg.plane_angle;
    }
    else {
      resolved.plane_angle = []
    }

    if (msg.plane_height !== undefined) {
      resolved.plane_height = msg.plane_height;
    }
    else {
      resolved.plane_height = []
    }

    if (msg.occupancy !== undefined) {
      resolved.occupancy = msg.occupancy;
    }
    else {
      resolved.occupancy = []
    }

    return resolved;
    }
};

module.exports = GridPlaneInfo;
