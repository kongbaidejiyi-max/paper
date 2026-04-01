// Auto-generated. Do not edit!

// (in-package fitplane.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Plane = require('./Plane.js');

//-----------------------------------------------------------

class PlaneMap {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.width = null;
      this.height = null;
      this.resolution = null;
      this.origin_x = null;
      this.origin_y = null;
      this.PlaneGridMap = null;
    }
    else {
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
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
      if (initObj.hasOwnProperty('origin_x')) {
        this.origin_x = initObj.origin_x
      }
      else {
        this.origin_x = 0.0;
      }
      if (initObj.hasOwnProperty('origin_y')) {
        this.origin_y = initObj.origin_y
      }
      else {
        this.origin_y = 0.0;
      }
      if (initObj.hasOwnProperty('PlaneGridMap')) {
        this.PlaneGridMap = initObj.PlaneGridMap
      }
      else {
        this.PlaneGridMap = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlaneMap
    // Serialize message field [width]
    bufferOffset = _serializer.int16(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int16(obj.height, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float32(obj.resolution, buffer, bufferOffset);
    // Serialize message field [origin_x]
    bufferOffset = _serializer.float32(obj.origin_x, buffer, bufferOffset);
    // Serialize message field [origin_y]
    bufferOffset = _serializer.float32(obj.origin_y, buffer, bufferOffset);
    // Serialize message field [PlaneGridMap]
    // Serialize the length for message field [PlaneGridMap]
    bufferOffset = _serializer.uint32(obj.PlaneGridMap.length, buffer, bufferOffset);
    obj.PlaneGridMap.forEach((val) => {
      bufferOffset = Plane.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlaneMap
    let len;
    let data = new PlaneMap(null);
    // Deserialize message field [width]
    data.width = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [origin_x]
    data.origin_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [origin_y]
    data.origin_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [PlaneGridMap]
    // Deserialize array length for message field [PlaneGridMap]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.PlaneGridMap = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.PlaneGridMap[i] = Plane.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.PlaneGridMap.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fitplane/PlaneMap';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0eee82ac8b4aa09e68a738e9785183cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 width
    int16 height
    float32 resolution
    float32 origin_x
    float32 origin_y
    Plane[] PlaneGridMap
    ================================================================================
    MSG: fitplane/Plane
    float32 PlaneCellHeight
    float32 PlaneCellAngle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlaneMap(null);
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

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    if (msg.origin_x !== undefined) {
      resolved.origin_x = msg.origin_x;
    }
    else {
      resolved.origin_x = 0.0
    }

    if (msg.origin_y !== undefined) {
      resolved.origin_y = msg.origin_y;
    }
    else {
      resolved.origin_y = 0.0
    }

    if (msg.PlaneGridMap !== undefined) {
      resolved.PlaneGridMap = new Array(msg.PlaneGridMap.length);
      for (let i = 0; i < resolved.PlaneGridMap.length; ++i) {
        resolved.PlaneGridMap[i] = Plane.Resolve(msg.PlaneGridMap[i]);
      }
    }
    else {
      resolved.PlaneGridMap = []
    }

    return resolved;
    }
};

module.exports = PlaneMap;
