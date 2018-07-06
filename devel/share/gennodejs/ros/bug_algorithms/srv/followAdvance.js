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

class followAdvanceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.algorithm = null;
      this.state = null;
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
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
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
    // Serializes a message object of type followAdvanceRequest
    // Serialize message field [algorithm]
    bufferOffset = _serializer.int32(obj.algorithm, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int32(obj.state, buffer, bufferOffset);
    // Serialize message field [reverse]
    bufferOffset = _serializer.bool(obj.reverse, buffer, bufferOffset);
    // Serialize message field [choose]
    bufferOffset = _serializer.bool(obj.choose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type followAdvanceRequest
    let len;
    let data = new followAdvanceRequest(null);
    // Deserialize message field [algorithm]
    data.algorithm = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [reverse]
    data.reverse = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [choose]
    data.choose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bug_algorithms/followAdvanceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc69dfe3904d0c237938b0639fbc5a79';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 algorithm
    int32 state
    bool reverse
    bool choose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new followAdvanceRequest(null);
    if (msg.algorithm !== undefined) {
      resolved.algorithm = msg.algorithm;
    }
    else {
      resolved.algorithm = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
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

class followAdvanceResponse {
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
    // Serializes a message object of type followAdvanceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type followAdvanceResponse
    let len;
    let data = new followAdvanceResponse(null);
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
    return 'bug_algorithms/followAdvanceResponse';
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
    const resolved = new followAdvanceResponse(null);
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
  Request: followAdvanceRequest,
  Response: followAdvanceResponse,
  md5sum() { return '0cdc9d6762a3b9cb2474f738c3771c80'; },
  datatype() { return 'bug_algorithms/followAdvance'; }
};
