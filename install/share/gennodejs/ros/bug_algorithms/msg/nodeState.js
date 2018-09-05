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

class nodeState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.algorithm = null;
      this.node_state = null;
      this.node_state_desc = null;
      this.node_state_time = null;
      this.bug_state = null;
      this.bug_state_desc = null;
      this.bug_state_time = null;
    }
    else {
      if (initObj.hasOwnProperty('algorithm')) {
        this.algorithm = initObj.algorithm
      }
      else {
        this.algorithm = 0;
      }
      if (initObj.hasOwnProperty('node_state')) {
        this.node_state = initObj.node_state
      }
      else {
        this.node_state = 0;
      }
      if (initObj.hasOwnProperty('node_state_desc')) {
        this.node_state_desc = initObj.node_state_desc
      }
      else {
        this.node_state_desc = '';
      }
      if (initObj.hasOwnProperty('node_state_time')) {
        this.node_state_time = initObj.node_state_time
      }
      else {
        this.node_state_time = 0.0;
      }
      if (initObj.hasOwnProperty('bug_state')) {
        this.bug_state = initObj.bug_state
      }
      else {
        this.bug_state = 0;
      }
      if (initObj.hasOwnProperty('bug_state_desc')) {
        this.bug_state_desc = initObj.bug_state_desc
      }
      else {
        this.bug_state_desc = '';
      }
      if (initObj.hasOwnProperty('bug_state_time')) {
        this.bug_state_time = initObj.bug_state_time
      }
      else {
        this.bug_state_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nodeState
    // Serialize message field [algorithm]
    bufferOffset = _serializer.uint8(obj.algorithm, buffer, bufferOffset);
    // Serialize message field [node_state]
    bufferOffset = _serializer.uint8(obj.node_state, buffer, bufferOffset);
    // Serialize message field [node_state_desc]
    bufferOffset = _serializer.string(obj.node_state_desc, buffer, bufferOffset);
    // Serialize message field [node_state_time]
    bufferOffset = _serializer.float32(obj.node_state_time, buffer, bufferOffset);
    // Serialize message field [bug_state]
    bufferOffset = _serializer.uint8(obj.bug_state, buffer, bufferOffset);
    // Serialize message field [bug_state_desc]
    bufferOffset = _serializer.string(obj.bug_state_desc, buffer, bufferOffset);
    // Serialize message field [bug_state_time]
    bufferOffset = _serializer.float32(obj.bug_state_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nodeState
    let len;
    let data = new nodeState(null);
    // Deserialize message field [algorithm]
    data.algorithm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [node_state]
    data.node_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [node_state_desc]
    data.node_state_desc = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [node_state_time]
    data.node_state_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bug_state]
    data.bug_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [bug_state_desc]
    data.bug_state_desc = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [bug_state_time]
    data.bug_state_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.node_state_desc.length;
    length += object.bug_state_desc.length;
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bug_algorithms/nodeState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eacf4f1a4f8ef654fd25492c527c277f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 algorithm
    uint8 node_state
    string node_state_desc
    float32 node_state_time
    uint8 bug_state
    string bug_state_desc
    float32 bug_state_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nodeState(null);
    if (msg.algorithm !== undefined) {
      resolved.algorithm = msg.algorithm;
    }
    else {
      resolved.algorithm = 0
    }

    if (msg.node_state !== undefined) {
      resolved.node_state = msg.node_state;
    }
    else {
      resolved.node_state = 0
    }

    if (msg.node_state_desc !== undefined) {
      resolved.node_state_desc = msg.node_state_desc;
    }
    else {
      resolved.node_state_desc = ''
    }

    if (msg.node_state_time !== undefined) {
      resolved.node_state_time = msg.node_state_time;
    }
    else {
      resolved.node_state_time = 0.0
    }

    if (msg.bug_state !== undefined) {
      resolved.bug_state = msg.bug_state;
    }
    else {
      resolved.bug_state = 0
    }

    if (msg.bug_state_desc !== undefined) {
      resolved.bug_state_desc = msg.bug_state_desc;
    }
    else {
      resolved.bug_state_desc = ''
    }

    if (msg.bug_state_time !== undefined) {
      resolved.bug_state_time = msg.bug_state_time;
    }
    else {
      resolved.bug_state_time = 0.0
    }

    return resolved;
    }
};

module.exports = nodeState;
