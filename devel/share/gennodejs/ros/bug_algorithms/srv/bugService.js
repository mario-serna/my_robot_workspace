// Auto-generated. Do not edit!

// (in-package bug_algorithms.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class bugServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.algorithm = null;
      this.velocity = null;
      this.initial_x = null;
      this.initial_y = null;
      this.desired_x = null;
      this.desired_y = null;
      this.simulation = null;
      this.reverse = null;
      this.choose = null;
    }
    else {
      if (initObj.hasOwnProperty('algorithm')) {
        this.algorithm = initObj.algorithm
      }
      else {
        this.algorithm = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('initial_x')) {
        this.initial_x = initObj.initial_x
      }
      else {
        this.initial_x = 0.0;
      }
      if (initObj.hasOwnProperty('initial_y')) {
        this.initial_y = initObj.initial_y
      }
      else {
        this.initial_y = 0.0;
      }
      if (initObj.hasOwnProperty('desired_x')) {
        this.desired_x = initObj.desired_x
      }
      else {
        this.desired_x = 0.0;
      }
      if (initObj.hasOwnProperty('desired_y')) {
        this.desired_y = initObj.desired_y
      }
      else {
        this.desired_y = 0.0;
      }
      if (initObj.hasOwnProperty('simulation')) {
        this.simulation = initObj.simulation
      }
      else {
        this.simulation = false;
      }
      if (initObj.hasOwnProperty('reverse')) {
        this.reverse = initObj.reverse
      }
      else {
        this.reverse = false;
      }
      if (initObj.hasOwnProperty('choose')) {
        this.choose = initObj.choose
      }
      else {
        this.choose = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bugServiceRequest
    // Serialize message field [algorithm]
    bufferOffset = _serializer.int32(obj.algorithm, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [initial_x]
    bufferOffset = _serializer.float32(obj.initial_x, buffer, bufferOffset);
    // Serialize message field [initial_y]
    bufferOffset = _serializer.float32(obj.initial_y, buffer, bufferOffset);
    // Serialize message field [desired_x]
    bufferOffset = _serializer.float32(obj.desired_x, buffer, bufferOffset);
    // Serialize message field [desired_y]
    bufferOffset = _serializer.float32(obj.desired_y, buffer, bufferOffset);
    // Serialize message field [simulation]
    bufferOffset = _serializer.bool(obj.simulation, buffer, bufferOffset);
    // Serialize message field [reverse]
    bufferOffset = _serializer.bool(obj.reverse, buffer, bufferOffset);
    // Serialize message field [choose]
    bufferOffset = _serializer.bool(obj.choose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bugServiceRequest
    let len;
    let data = new bugServiceRequest(null);
    // Deserialize message field [algorithm]
    data.algorithm = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_x]
    data.initial_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_y]
    data.initial_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_x]
    data.desired_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_y]
    data.desired_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [simulation]
    data.simulation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reverse]
    data.reverse = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [choose]
    data.choose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 27;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bug_algorithms/bugServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '310f018babea00a829c4f64be9e6a75a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 algorithm
    float32 velocity
    float32 initial_x
    float32 initial_y
    float32 desired_x
    float32 desired_y
    bool simulation
    bool reverse
    bool choose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bugServiceRequest(null);
    if (msg.algorithm !== undefined) {
      resolved.algorithm = msg.algorithm;
    }
    else {
      resolved.algorithm = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.initial_x !== undefined) {
      resolved.initial_x = msg.initial_x;
    }
    else {
      resolved.initial_x = 0.0
    }

    if (msg.initial_y !== undefined) {
      resolved.initial_y = msg.initial_y;
    }
    else {
      resolved.initial_y = 0.0
    }

    if (msg.desired_x !== undefined) {
      resolved.desired_x = msg.desired_x;
    }
    else {
      resolved.desired_x = 0.0
    }

    if (msg.desired_y !== undefined) {
      resolved.desired_y = msg.desired_y;
    }
    else {
      resolved.desired_y = 0.0
    }

    if (msg.simulation !== undefined) {
      resolved.simulation = msg.simulation;
    }
    else {
      resolved.simulation = false
    }

    if (msg.reverse !== undefined) {
      resolved.reverse = msg.reverse;
    }
    else {
      resolved.reverse = false
    }

    if (msg.choose !== undefined) {
      resolved.choose = msg.choose;
    }
    else {
      resolved.choose = false
    }

    return resolved;
    }
};

class bugServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bugServiceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bugServiceResponse
    let len;
    let data = new bugServiceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bug_algorithms/bugServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bugServiceResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: bugServiceRequest,
  Response: bugServiceResponse,
  md5sum() { return 'c7f9b178e6a7aa0cd2e64b98d5c26b60'; },
  datatype() { return 'bug_algorithms/bugService'; }
};
