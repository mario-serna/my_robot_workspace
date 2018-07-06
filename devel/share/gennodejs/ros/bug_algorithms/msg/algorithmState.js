// Auto-generated. Do not edit!

// (in-package bug_algorithms.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class algorithmState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.algorithm = null;
      this.name = null;
      this.pose_x = null;
      this.pose_y = null;
      this.yaw = null;
      this.initial_to_goal_distance = null;
      this.current_to_goal_distance = null;
      this.path_length = null;
    }
    else {
      if (initObj.hasOwnProperty('algorithm')) {
        this.algorithm = initObj.algorithm
      }
      else {
        this.algorithm = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('pose_x')) {
        this.pose_x = initObj.pose_x
      }
      else {
        this.pose_x = 0.0;
      }
      if (initObj.hasOwnProperty('pose_y')) {
        this.pose_y = initObj.pose_y
      }
      else {
        this.pose_y = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('initial_to_goal_distance')) {
        this.initial_to_goal_distance = initObj.initial_to_goal_distance
      }
      else {
        this.initial_to_goal_distance = 0.0;
      }
      if (initObj.hasOwnProperty('current_to_goal_distance')) {
        this.current_to_goal_distance = initObj.current_to_goal_distance
      }
      else {
        this.current_to_goal_distance = 0.0;
      }
      if (initObj.hasOwnProperty('path_length')) {
        this.path_length = initObj.path_length
      }
      else {
        this.path_length = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type algorithmState
    // Serialize message field [algorithm]
    bufferOffset = _serializer.uint8(obj.algorithm, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [pose_x]
    bufferOffset = _serializer.float32(obj.pose_x, buffer, bufferOffset);
    // Serialize message field [pose_y]
    bufferOffset = _serializer.float32(obj.pose_y, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [initial_to_goal_distance]
    bufferOffset = _serializer.float32(obj.initial_to_goal_distance, buffer, bufferOffset);
    // Serialize message field [current_to_goal_distance]
    bufferOffset = _serializer.float32(obj.current_to_goal_distance, buffer, bufferOffset);
    // Serialize message field [path_length]
    bufferOffset = _serializer.float32(obj.path_length, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type algorithmState
    let len;
    let data = new algorithmState(null);
    // Deserialize message field [algorithm]
    data.algorithm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose_x]
    data.pose_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pose_y]
    data.pose_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_to_goal_distance]
    data.initial_to_goal_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_to_goal_distance]
    data.current_to_goal_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [path_length]
    data.path_length = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bug_algorithms/algorithmState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '952308ef00da013e137aadd9bfb800a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 algorithm
    string name
    float32 pose_x
    float32 pose_y
    float32 yaw
    float32 initial_to_goal_distance
    float32 current_to_goal_distance
    float32 path_length
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new algorithmState(null);
    if (msg.algorithm !== undefined) {
      resolved.algorithm = msg.algorithm;
    }
    else {
      resolved.algorithm = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.pose_x !== undefined) {
      resolved.pose_x = msg.pose_x;
    }
    else {
      resolved.pose_x = 0.0
    }

    if (msg.pose_y !== undefined) {
      resolved.pose_y = msg.pose_y;
    }
    else {
      resolved.pose_y = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.initial_to_goal_distance !== undefined) {
      resolved.initial_to_goal_distance = msg.initial_to_goal_distance;
    }
    else {
      resolved.initial_to_goal_distance = 0.0
    }

    if (msg.current_to_goal_distance !== undefined) {
      resolved.current_to_goal_distance = msg.current_to_goal_distance;
    }
    else {
      resolved.current_to_goal_distance = 0.0
    }

    if (msg.path_length !== undefined) {
      resolved.path_length = msg.path_length;
    }
    else {
      resolved.path_length = 0.0
    }

    return resolved;
    }
};

module.exports = algorithmState;
