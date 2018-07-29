# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bug_algorithms/algorithmState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class algorithmState(genpy.Message):
  _md5sum = "eed00cdf20aebf4db1b70bfc80462703"
  _type = "bug_algorithms/algorithmState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 algorithm
string name
float32 pose_x
float32 pose_y
float32 yaw
float32 initial_to_goal_distance
float32 current_to_goal_distance
float32 best_distance
float32 path_length
"""
  __slots__ = ['algorithm','name','pose_x','pose_y','yaw','initial_to_goal_distance','current_to_goal_distance','best_distance','path_length']
  _slot_types = ['uint8','string','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       algorithm,name,pose_x,pose_y,yaw,initial_to_goal_distance,current_to_goal_distance,best_distance,path_length

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(algorithmState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.algorithm is None:
        self.algorithm = 0
      if self.name is None:
        self.name = ''
      if self.pose_x is None:
        self.pose_x = 0.
      if self.pose_y is None:
        self.pose_y = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.initial_to_goal_distance is None:
        self.initial_to_goal_distance = 0.
      if self.current_to_goal_distance is None:
        self.current_to_goal_distance = 0.
      if self.best_distance is None:
        self.best_distance = 0.
      if self.path_length is None:
        self.path_length = 0.
    else:
      self.algorithm = 0
      self.name = ''
      self.pose_x = 0.
      self.pose_y = 0.
      self.yaw = 0.
      self.initial_to_goal_distance = 0.
      self.current_to_goal_distance = 0.
      self.best_distance = 0.
      self.path_length = 0.

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
      buff.write(_get_struct_B().pack(self.algorithm))
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7f().pack(_x.pose_x, _x.pose_y, _x.yaw, _x.initial_to_goal_distance, _x.current_to_goal_distance, _x.best_distance, _x.path_length))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.algorithm,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.pose_x, _x.pose_y, _x.yaw, _x.initial_to_goal_distance, _x.current_to_goal_distance, _x.best_distance, _x.path_length,) = _get_struct_7f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_get_struct_B().pack(self.algorithm))
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7f().pack(_x.pose_x, _x.pose_y, _x.yaw, _x.initial_to_goal_distance, _x.current_to_goal_distance, _x.best_distance, _x.path_length))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.algorithm,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.pose_x, _x.pose_y, _x.yaw, _x.initial_to_goal_distance, _x.current_to_goal_distance, _x.best_distance, _x.path_length,) = _get_struct_7f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7f = None
def _get_struct_7f():
    global _struct_7f
    if _struct_7f is None:
        _struct_7f = struct.Struct("<7f")
    return _struct_7f
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
