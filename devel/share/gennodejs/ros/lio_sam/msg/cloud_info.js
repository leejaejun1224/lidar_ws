// Auto-generated. Do not edit!

// (in-package lio_sam.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class cloud_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.startRingIndex = null;
      this.endRingIndex = null;
      this.pointColInd = null;
      this.pointRange = null;
      this.imuAvailable = null;
      this.odomAvailable = null;
      this.imuRollInit = null;
      this.imuPitchInit = null;
      this.imuYawInit = null;
      this.initialGuessX = null;
      this.initialGuessY = null;
      this.initialGuessZ = null;
      this.initialGuessRoll = null;
      this.initialGuessPitch = null;
      this.initialGuessYaw = null;
      this.cloud_deskewed = null;
      this.cloud_corner = null;
      this.cloud_surface = null;
      this.cone_inner = null;
      this.cone_outer = null;
      this.left_cone = null;
      this.right_cone = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('startRingIndex')) {
        this.startRingIndex = initObj.startRingIndex
      }
      else {
        this.startRingIndex = [];
      }
      if (initObj.hasOwnProperty('endRingIndex')) {
        this.endRingIndex = initObj.endRingIndex
      }
      else {
        this.endRingIndex = [];
      }
      if (initObj.hasOwnProperty('pointColInd')) {
        this.pointColInd = initObj.pointColInd
      }
      else {
        this.pointColInd = [];
      }
      if (initObj.hasOwnProperty('pointRange')) {
        this.pointRange = initObj.pointRange
      }
      else {
        this.pointRange = [];
      }
      if (initObj.hasOwnProperty('imuAvailable')) {
        this.imuAvailable = initObj.imuAvailable
      }
      else {
        this.imuAvailable = 0;
      }
      if (initObj.hasOwnProperty('odomAvailable')) {
        this.odomAvailable = initObj.odomAvailable
      }
      else {
        this.odomAvailable = 0;
      }
      if (initObj.hasOwnProperty('imuRollInit')) {
        this.imuRollInit = initObj.imuRollInit
      }
      else {
        this.imuRollInit = 0.0;
      }
      if (initObj.hasOwnProperty('imuPitchInit')) {
        this.imuPitchInit = initObj.imuPitchInit
      }
      else {
        this.imuPitchInit = 0.0;
      }
      if (initObj.hasOwnProperty('imuYawInit')) {
        this.imuYawInit = initObj.imuYawInit
      }
      else {
        this.imuYawInit = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessX')) {
        this.initialGuessX = initObj.initialGuessX
      }
      else {
        this.initialGuessX = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessY')) {
        this.initialGuessY = initObj.initialGuessY
      }
      else {
        this.initialGuessY = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessZ')) {
        this.initialGuessZ = initObj.initialGuessZ
      }
      else {
        this.initialGuessZ = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessRoll')) {
        this.initialGuessRoll = initObj.initialGuessRoll
      }
      else {
        this.initialGuessRoll = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessPitch')) {
        this.initialGuessPitch = initObj.initialGuessPitch
      }
      else {
        this.initialGuessPitch = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessYaw')) {
        this.initialGuessYaw = initObj.initialGuessYaw
      }
      else {
        this.initialGuessYaw = 0.0;
      }
      if (initObj.hasOwnProperty('cloud_deskewed')) {
        this.cloud_deskewed = initObj.cloud_deskewed
      }
      else {
        this.cloud_deskewed = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('cloud_corner')) {
        this.cloud_corner = initObj.cloud_corner
      }
      else {
        this.cloud_corner = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('cloud_surface')) {
        this.cloud_surface = initObj.cloud_surface
      }
      else {
        this.cloud_surface = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('cone_inner')) {
        this.cone_inner = initObj.cone_inner
      }
      else {
        this.cone_inner = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('cone_outer')) {
        this.cone_outer = initObj.cone_outer
      }
      else {
        this.cone_outer = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('left_cone')) {
        this.left_cone = initObj.left_cone
      }
      else {
        this.left_cone = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('right_cone')) {
        this.right_cone = initObj.right_cone
      }
      else {
        this.right_cone = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cloud_info
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [startRingIndex]
    bufferOffset = _arraySerializer.int32(obj.startRingIndex, buffer, bufferOffset, null);
    // Serialize message field [endRingIndex]
    bufferOffset = _arraySerializer.int32(obj.endRingIndex, buffer, bufferOffset, null);
    // Serialize message field [pointColInd]
    bufferOffset = _arraySerializer.int32(obj.pointColInd, buffer, bufferOffset, null);
    // Serialize message field [pointRange]
    bufferOffset = _arraySerializer.float32(obj.pointRange, buffer, bufferOffset, null);
    // Serialize message field [imuAvailable]
    bufferOffset = _serializer.int64(obj.imuAvailable, buffer, bufferOffset);
    // Serialize message field [odomAvailable]
    bufferOffset = _serializer.int64(obj.odomAvailable, buffer, bufferOffset);
    // Serialize message field [imuRollInit]
    bufferOffset = _serializer.float32(obj.imuRollInit, buffer, bufferOffset);
    // Serialize message field [imuPitchInit]
    bufferOffset = _serializer.float32(obj.imuPitchInit, buffer, bufferOffset);
    // Serialize message field [imuYawInit]
    bufferOffset = _serializer.float32(obj.imuYawInit, buffer, bufferOffset);
    // Serialize message field [initialGuessX]
    bufferOffset = _serializer.float32(obj.initialGuessX, buffer, bufferOffset);
    // Serialize message field [initialGuessY]
    bufferOffset = _serializer.float32(obj.initialGuessY, buffer, bufferOffset);
    // Serialize message field [initialGuessZ]
    bufferOffset = _serializer.float32(obj.initialGuessZ, buffer, bufferOffset);
    // Serialize message field [initialGuessRoll]
    bufferOffset = _serializer.float32(obj.initialGuessRoll, buffer, bufferOffset);
    // Serialize message field [initialGuessPitch]
    bufferOffset = _serializer.float32(obj.initialGuessPitch, buffer, bufferOffset);
    // Serialize message field [initialGuessYaw]
    bufferOffset = _serializer.float32(obj.initialGuessYaw, buffer, bufferOffset);
    // Serialize message field [cloud_deskewed]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_deskewed, buffer, bufferOffset);
    // Serialize message field [cloud_corner]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_corner, buffer, bufferOffset);
    // Serialize message field [cloud_surface]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_surface, buffer, bufferOffset);
    // Serialize message field [cone_inner]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cone_inner, buffer, bufferOffset);
    // Serialize message field [cone_outer]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cone_outer, buffer, bufferOffset);
    // Serialize message field [left_cone]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.left_cone, buffer, bufferOffset);
    // Serialize message field [right_cone]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.right_cone, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cloud_info
    let len;
    let data = new cloud_info(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [startRingIndex]
    data.startRingIndex = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [endRingIndex]
    data.endRingIndex = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [pointColInd]
    data.pointColInd = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [pointRange]
    data.pointRange = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [imuAvailable]
    data.imuAvailable = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [odomAvailable]
    data.odomAvailable = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [imuRollInit]
    data.imuRollInit = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imuPitchInit]
    data.imuPitchInit = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imuYawInit]
    data.imuYawInit = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessX]
    data.initialGuessX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessY]
    data.initialGuessY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessZ]
    data.initialGuessZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessRoll]
    data.initialGuessRoll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessPitch]
    data.initialGuessPitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessYaw]
    data.initialGuessYaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cloud_deskewed]
    data.cloud_deskewed = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [cloud_corner]
    data.cloud_corner = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [cloud_surface]
    data.cloud_surface = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [cone_inner]
    data.cone_inner = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [cone_outer]
    data.cone_outer = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_cone]
    data.left_cone = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [right_cone]
    data.right_cone = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.startRingIndex.length;
    length += 4 * object.endRingIndex.length;
    length += 4 * object.pointColInd.length;
    length += 4 * object.pointRange.length;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_deskewed);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_corner);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_surface);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cone_inner);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cone_outer);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.left_cone);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.right_cone);
    return length + 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lio_sam/cloud_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7eeb8b423d5a30ad7be7c5f84fbec6c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Cloud Info
    Header header 
    
    int32[] startRingIndex
    int32[] endRingIndex
    
    int32[]  pointColInd # point column index in range image
    float32[] pointRange # point range 
    
    int64 imuAvailable
    int64 odomAvailable
    
    # Attitude for LOAM initialization
    float32 imuRollInit
    float32 imuPitchInit
    float32 imuYawInit
    
    # Initial guess from imu pre-integration
    float32 initialGuessX
    float32 initialGuessY
    float32 initialGuessZ
    float32 initialGuessRoll
    float32 initialGuessPitch
    float32 initialGuessYaw
    
    # Point cloud messages
    sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
    sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
    sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature
    sensor_msgs/PointCloud2 cone_inner   # extracted inner cone feature
    sensor_msgs/PointCloud2 cone_outer   # extracted outer cone feature
    
    sensor_msgs/PointCloud2 left_cone   # extracted inner cone feature
    sensor_msgs/PointCloud2 right_cone   # extracted inner cone feature
    
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
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
    ================================================================================
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cloud_info(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.startRingIndex !== undefined) {
      resolved.startRingIndex = msg.startRingIndex;
    }
    else {
      resolved.startRingIndex = []
    }

    if (msg.endRingIndex !== undefined) {
      resolved.endRingIndex = msg.endRingIndex;
    }
    else {
      resolved.endRingIndex = []
    }

    if (msg.pointColInd !== undefined) {
      resolved.pointColInd = msg.pointColInd;
    }
    else {
      resolved.pointColInd = []
    }

    if (msg.pointRange !== undefined) {
      resolved.pointRange = msg.pointRange;
    }
    else {
      resolved.pointRange = []
    }

    if (msg.imuAvailable !== undefined) {
      resolved.imuAvailable = msg.imuAvailable;
    }
    else {
      resolved.imuAvailable = 0
    }

    if (msg.odomAvailable !== undefined) {
      resolved.odomAvailable = msg.odomAvailable;
    }
    else {
      resolved.odomAvailable = 0
    }

    if (msg.imuRollInit !== undefined) {
      resolved.imuRollInit = msg.imuRollInit;
    }
    else {
      resolved.imuRollInit = 0.0
    }

    if (msg.imuPitchInit !== undefined) {
      resolved.imuPitchInit = msg.imuPitchInit;
    }
    else {
      resolved.imuPitchInit = 0.0
    }

    if (msg.imuYawInit !== undefined) {
      resolved.imuYawInit = msg.imuYawInit;
    }
    else {
      resolved.imuYawInit = 0.0
    }

    if (msg.initialGuessX !== undefined) {
      resolved.initialGuessX = msg.initialGuessX;
    }
    else {
      resolved.initialGuessX = 0.0
    }

    if (msg.initialGuessY !== undefined) {
      resolved.initialGuessY = msg.initialGuessY;
    }
    else {
      resolved.initialGuessY = 0.0
    }

    if (msg.initialGuessZ !== undefined) {
      resolved.initialGuessZ = msg.initialGuessZ;
    }
    else {
      resolved.initialGuessZ = 0.0
    }

    if (msg.initialGuessRoll !== undefined) {
      resolved.initialGuessRoll = msg.initialGuessRoll;
    }
    else {
      resolved.initialGuessRoll = 0.0
    }

    if (msg.initialGuessPitch !== undefined) {
      resolved.initialGuessPitch = msg.initialGuessPitch;
    }
    else {
      resolved.initialGuessPitch = 0.0
    }

    if (msg.initialGuessYaw !== undefined) {
      resolved.initialGuessYaw = msg.initialGuessYaw;
    }
    else {
      resolved.initialGuessYaw = 0.0
    }

    if (msg.cloud_deskewed !== undefined) {
      resolved.cloud_deskewed = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_deskewed)
    }
    else {
      resolved.cloud_deskewed = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.cloud_corner !== undefined) {
      resolved.cloud_corner = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_corner)
    }
    else {
      resolved.cloud_corner = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.cloud_surface !== undefined) {
      resolved.cloud_surface = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_surface)
    }
    else {
      resolved.cloud_surface = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.cone_inner !== undefined) {
      resolved.cone_inner = sensor_msgs.msg.PointCloud2.Resolve(msg.cone_inner)
    }
    else {
      resolved.cone_inner = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.cone_outer !== undefined) {
      resolved.cone_outer = sensor_msgs.msg.PointCloud2.Resolve(msg.cone_outer)
    }
    else {
      resolved.cone_outer = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.left_cone !== undefined) {
      resolved.left_cone = sensor_msgs.msg.PointCloud2.Resolve(msg.left_cone)
    }
    else {
      resolved.left_cone = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.right_cone !== undefined) {
      resolved.right_cone = sensor_msgs.msg.PointCloud2.Resolve(msg.right_cone)
    }
    else {
      resolved.right_cone = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

module.exports = cloud_info;
