// Auto-generated. Do not edit!

// (in-package ros_whill.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SpeedPack = require('../msg/SpeedPack.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetSpeedProfileRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.forward = null;
      this.backward = null;
      this.turn = null;
    }
    else {
      if (initObj.hasOwnProperty('forward')) {
        this.forward = initObj.forward
      }
      else {
        this.forward = new SpeedPack();
      }
      if (initObj.hasOwnProperty('backward')) {
        this.backward = initObj.backward
      }
      else {
        this.backward = new SpeedPack();
      }
      if (initObj.hasOwnProperty('turn')) {
        this.turn = initObj.turn
      }
      else {
        this.turn = new SpeedPack();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSpeedProfileRequest
    // Serialize message field [forward]
    bufferOffset = SpeedPack.serialize(obj.forward, buffer, bufferOffset);
    // Serialize message field [backward]
    bufferOffset = SpeedPack.serialize(obj.backward, buffer, bufferOffset);
    // Serialize message field [turn]
    bufferOffset = SpeedPack.serialize(obj.turn, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSpeedProfileRequest
    let len;
    let data = new SetSpeedProfileRequest(null);
    // Deserialize message field [forward]
    data.forward = SpeedPack.deserialize(buffer, bufferOffset);
    // Deserialize message field [backward]
    data.backward = SpeedPack.deserialize(buffer, bufferOffset);
    // Deserialize message field [turn]
    data.turn = SpeedPack.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_whill/SetSpeedProfileRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '199fa9ff7314ba707449d97b2d9d09c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ros_whill/SpeedPack forward      # m/s(Speed) and m/ss(Acceleration/Deceleration)
    ros_whill/SpeedPack backward     # m/s(Speed) and m/ss(Acceleration/Deceleration)
    ros_whill/SpeedPack turn         # rad/s(Speed) and rad/ss(Acceleration/Deceleration)
    
    
    ================================================================================
    MSG: ros_whill/SpeedPack
    float32 speed  # m/s   or rad/s
    float32 acc    # m/ss  or rad/ss
    float32 dec    # m/ss  or rad/ss
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSpeedProfileRequest(null);
    if (msg.forward !== undefined) {
      resolved.forward = SpeedPack.Resolve(msg.forward)
    }
    else {
      resolved.forward = new SpeedPack()
    }

    if (msg.backward !== undefined) {
      resolved.backward = SpeedPack.Resolve(msg.backward)
    }
    else {
      resolved.backward = new SpeedPack()
    }

    if (msg.turn !== undefined) {
      resolved.turn = SpeedPack.Resolve(msg.turn)
    }
    else {
      resolved.turn = new SpeedPack()
    }

    return resolved;
    }
};

class SetSpeedProfileResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.status_message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('status_message')) {
        this.status_message = initObj.status_message
      }
      else {
        this.status_message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSpeedProfileResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status_message]
    bufferOffset = _serializer.string(obj.status_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSpeedProfileResponse
    let len;
    let data = new SetSpeedProfileResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status_message]
    data.status_message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.status_message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_whill/SetSpeedProfileResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ec6f3eff0161f4257b808b12bc830c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string status_message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSpeedProfileResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.status_message !== undefined) {
      resolved.status_message = msg.status_message;
    }
    else {
      resolved.status_message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetSpeedProfileRequest,
  Response: SetSpeedProfileResponse,
  md5sum() { return '7edfb7c10ea7c5bdda72a3f1fbcf0705'; },
  datatype() { return 'ros_whill/SetSpeedProfile'; }
};
