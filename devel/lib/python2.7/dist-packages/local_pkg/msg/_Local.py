# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from local_pkg/Local.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class Local(genpy.Message):
  _md5sum = "97c056fa8f8e5feb2a27b1606009be69"
  _type = "local_pkg/Local"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
float64 x
float64 y
float64 heading
float64 imu_heading
float64 gps_heading
float64 gps_heading_under_4_headAcc
float64 gps_heading_raw_data
float64 heading_from_position
float64 dist
float64 roll
float64 pitch
float64 dr_x
float64 dr_y
float64 dead_m
int64 encoder_left
int64 encoder_right
float64 dead_right
float64 distance
float64 encoder_x
float64 encoder_y
int64 hAcc
float64 headAcc
int64 gear
float64 speeed
float64 gspeed
float64 dis
float64 position_error_between_encoder_and_gps
geometry_msgs/Quaternion orientation
float64 distance_x
float64 distance_y
float64 distance_z

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
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['header','x','y','heading','imu_heading','gps_heading','gps_heading_under_4_headAcc','gps_heading_raw_data','heading_from_position','dist','roll','pitch','dr_x','dr_y','dead_m','encoder_left','encoder_right','dead_right','distance','encoder_x','encoder_y','hAcc','headAcc','gear','speeed','gspeed','dis','position_error_between_encoder_and_gps','orientation','distance_x','distance_y','distance_z']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','int64','int64','float64','float64','float64','float64','int64','float64','int64','float64','float64','float64','float64','geometry_msgs/Quaternion','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,x,y,heading,imu_heading,gps_heading,gps_heading_under_4_headAcc,gps_heading_raw_data,heading_from_position,dist,roll,pitch,dr_x,dr_y,dead_m,encoder_left,encoder_right,dead_right,distance,encoder_x,encoder_y,hAcc,headAcc,gear,speeed,gspeed,dis,position_error_between_encoder_and_gps,orientation,distance_x,distance_y,distance_z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Local, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.heading is None:
        self.heading = 0.
      if self.imu_heading is None:
        self.imu_heading = 0.
      if self.gps_heading is None:
        self.gps_heading = 0.
      if self.gps_heading_under_4_headAcc is None:
        self.gps_heading_under_4_headAcc = 0.
      if self.gps_heading_raw_data is None:
        self.gps_heading_raw_data = 0.
      if self.heading_from_position is None:
        self.heading_from_position = 0.
      if self.dist is None:
        self.dist = 0.
      if self.roll is None:
        self.roll = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.dr_x is None:
        self.dr_x = 0.
      if self.dr_y is None:
        self.dr_y = 0.
      if self.dead_m is None:
        self.dead_m = 0.
      if self.encoder_left is None:
        self.encoder_left = 0
      if self.encoder_right is None:
        self.encoder_right = 0
      if self.dead_right is None:
        self.dead_right = 0.
      if self.distance is None:
        self.distance = 0.
      if self.encoder_x is None:
        self.encoder_x = 0.
      if self.encoder_y is None:
        self.encoder_y = 0.
      if self.hAcc is None:
        self.hAcc = 0
      if self.headAcc is None:
        self.headAcc = 0.
      if self.gear is None:
        self.gear = 0
      if self.speeed is None:
        self.speeed = 0.
      if self.gspeed is None:
        self.gspeed = 0.
      if self.dis is None:
        self.dis = 0.
      if self.position_error_between_encoder_and_gps is None:
        self.position_error_between_encoder_and_gps = 0.
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      if self.distance_x is None:
        self.distance_x = 0.
      if self.distance_y is None:
        self.distance_y = 0.
      if self.distance_z is None:
        self.distance_z = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.x = 0.
      self.y = 0.
      self.heading = 0.
      self.imu_heading = 0.
      self.gps_heading = 0.
      self.gps_heading_under_4_headAcc = 0.
      self.gps_heading_raw_data = 0.
      self.heading_from_position = 0.
      self.dist = 0.
      self.roll = 0.
      self.pitch = 0.
      self.dr_x = 0.
      self.dr_y = 0.
      self.dead_m = 0.
      self.encoder_left = 0
      self.encoder_right = 0
      self.dead_right = 0.
      self.distance = 0.
      self.encoder_x = 0.
      self.encoder_y = 0.
      self.hAcc = 0
      self.headAcc = 0.
      self.gear = 0
      self.speeed = 0.
      self.gspeed = 0.
      self.dis = 0.
      self.position_error_between_encoder_and_gps = 0.
      self.orientation = geometry_msgs.msg.Quaternion()
      self.distance_x = 0.
      self.distance_y = 0.
      self.distance_z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_14d2q4dqdq11d().pack(_x.x, _x.y, _x.heading, _x.imu_heading, _x.gps_heading, _x.gps_heading_under_4_headAcc, _x.gps_heading_raw_data, _x.heading_from_position, _x.dist, _x.roll, _x.pitch, _x.dr_x, _x.dr_y, _x.dead_m, _x.encoder_left, _x.encoder_right, _x.dead_right, _x.distance, _x.encoder_x, _x.encoder_y, _x.hAcc, _x.headAcc, _x.gear, _x.speeed, _x.gspeed, _x.dis, _x.position_error_between_encoder_and_gps, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.distance_x, _x.distance_y, _x.distance_z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 272
      (_x.x, _x.y, _x.heading, _x.imu_heading, _x.gps_heading, _x.gps_heading_under_4_headAcc, _x.gps_heading_raw_data, _x.heading_from_position, _x.dist, _x.roll, _x.pitch, _x.dr_x, _x.dr_y, _x.dead_m, _x.encoder_left, _x.encoder_right, _x.dead_right, _x.distance, _x.encoder_x, _x.encoder_y, _x.hAcc, _x.headAcc, _x.gear, _x.speeed, _x.gspeed, _x.dis, _x.position_error_between_encoder_and_gps, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.distance_x, _x.distance_y, _x.distance_z,) = _get_struct_14d2q4dqdq11d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_14d2q4dqdq11d().pack(_x.x, _x.y, _x.heading, _x.imu_heading, _x.gps_heading, _x.gps_heading_under_4_headAcc, _x.gps_heading_raw_data, _x.heading_from_position, _x.dist, _x.roll, _x.pitch, _x.dr_x, _x.dr_y, _x.dead_m, _x.encoder_left, _x.encoder_right, _x.dead_right, _x.distance, _x.encoder_x, _x.encoder_y, _x.hAcc, _x.headAcc, _x.gear, _x.speeed, _x.gspeed, _x.dis, _x.position_error_between_encoder_and_gps, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.distance_x, _x.distance_y, _x.distance_z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 272
      (_x.x, _x.y, _x.heading, _x.imu_heading, _x.gps_heading, _x.gps_heading_under_4_headAcc, _x.gps_heading_raw_data, _x.heading_from_position, _x.dist, _x.roll, _x.pitch, _x.dr_x, _x.dr_y, _x.dead_m, _x.encoder_left, _x.encoder_right, _x.dead_right, _x.distance, _x.encoder_x, _x.encoder_y, _x.hAcc, _x.headAcc, _x.gear, _x.speeed, _x.gspeed, _x.dis, _x.position_error_between_encoder_and_gps, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.distance_x, _x.distance_y, _x.distance_z,) = _get_struct_14d2q4dqdq11d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_14d2q4dqdq11d = None
def _get_struct_14d2q4dqdq11d():
    global _struct_14d2q4dqdq11d
    if _struct_14d2q4dqdq11d is None:
        _struct_14d2q4dqdq11d = struct.Struct("<14d2q4dqdq11d")
    return _struct_14d2q4dqdq11d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I