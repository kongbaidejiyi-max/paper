// Auto-generated. Do not edit!

// (in-package fitplane_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GlobalPath {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.map_version = null;
      this.poses = null;
      this.corridor_half_width = null;
      this.path_confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('map_version')) {
        this.map_version = initObj.map_version
      }
      else {
        this.map_version = 0;
      }
      if (initObj.hasOwnProperty('poses')) {
        this.poses = initObj.poses
      }
      else {
        this.poses = [];
      }
      if (initObj.hasOwnProperty('corridor_half_width')) {
        this.corridor_half_width = initObj.corridor_half_width
      }
      else {
        this.corridor_half_width = [];
      }
      if (initObj.hasOwnProperty('path_confidence')) {
        this.path_confidence = initObj.path_confidence
      }
      else {
        this.path_confidence = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GlobalPath
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [map_version]
    bufferOffset = _serializer.uint32(obj.map_version, buffer, bufferOffset);
    // Serialize message field [poses]
    // Serialize the length for message field [poses]
    bufferOffset = _serializer.uint32(obj.poses.length, buffer, bufferOffset);
    obj.poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [corridor_half_width]
    bufferOffset = _arraySerializer.float32(obj.corridor_half_width, buffer, bufferOffset, null);
    // Serialize message field [path_confidence]
    bufferOffset = _arraySerializer.float32(obj.path_confidence, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GlobalPath
    let len;
    let data = new GlobalPath(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [map_version]
    data.map_version = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [poses]
    // Deserialize array length for message field [poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [corridor_half_width]
    data.corridor_half_width = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [path_confidence]
    data.path_confidence = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 56 * object.poses.length;
    length += 4 * object.corridor_half_width.length;
    length += 4 * object.path_confidence.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fitplane_planner/GlobalPath';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9916ef7263e04949af022154c3512e69';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Global reference path for UGV (UAV → UGV)
    std_msgs/Header header
    uint32 map_version                # 地图/路径版本（UAV 侧每次接收新图自增）
    geometry_msgs/Pose[] poses        # 路径控制点（世界系）
    float32[] corridor_half_width     # 对应poses的走廊半宽 w_i，单位m
    float32[] path_confidence         # 对应poses的段置信度 c_i ∈ [0,1]
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GlobalPath(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.map_version !== undefined) {
      resolved.map_version = msg.map_version;
    }
    else {
      resolved.map_version = 0
    }

    if (msg.poses !== undefined) {
      resolved.poses = new Array(msg.poses.length);
      for (let i = 0; i < resolved.poses.length; ++i) {
        resolved.poses[i] = geometry_msgs.msg.Pose.Resolve(msg.poses[i]);
      }
    }
    else {
      resolved.poses = []
    }

    if (msg.corridor_half_width !== undefined) {
      resolved.corridor_half_width = msg.corridor_half_width;
    }
    else {
      resolved.corridor_half_width = []
    }

    if (msg.path_confidence !== undefined) {
      resolved.path_confidence = msg.path_confidence;
    }
    else {
      resolved.path_confidence = []
    }

    return resolved;
    }
};

module.exports = GlobalPath;
