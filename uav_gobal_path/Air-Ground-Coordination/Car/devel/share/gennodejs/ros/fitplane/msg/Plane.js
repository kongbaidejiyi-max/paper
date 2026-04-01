// Auto-generated. Do not edit!

// (in-package fitplane.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Plane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.PlaneCellHeight = null;
      this.PlaneCellAngle = null;
    }
    else {
      if (initObj.hasOwnProperty('PlaneCellHeight')) {
        this.PlaneCellHeight = initObj.PlaneCellHeight
      }
      else {
        this.PlaneCellHeight = 0.0;
      }
      if (initObj.hasOwnProperty('PlaneCellAngle')) {
        this.PlaneCellAngle = initObj.PlaneCellAngle
      }
      else {
        this.PlaneCellAngle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Plane
    // Serialize message field [PlaneCellHeight]
    bufferOffset = _serializer.float32(obj.PlaneCellHeight, buffer, bufferOffset);
    // Serialize message field [PlaneCellAngle]
    bufferOffset = _serializer.float32(obj.PlaneCellAngle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Plane
    let len;
    let data = new Plane(null);
    // Deserialize message field [PlaneCellHeight]
    data.PlaneCellHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [PlaneCellAngle]
    data.PlaneCellAngle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fitplane/Plane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '92467c111abb460b6d16e58287e7864b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 PlaneCellHeight
    float32 PlaneCellAngle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Plane(null);
    if (msg.PlaneCellHeight !== undefined) {
      resolved.PlaneCellHeight = msg.PlaneCellHeight;
    }
    else {
      resolved.PlaneCellHeight = 0.0
    }

    if (msg.PlaneCellAngle !== undefined) {
      resolved.PlaneCellAngle = msg.PlaneCellAngle;
    }
    else {
      resolved.PlaneCellAngle = 0.0
    }

    return resolved;
    }
};

module.exports = Plane;
