# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bug_algorithms/nodeState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class nodeState(genpy.Message):
  _md5sum = "eacf4f1a4f8ef654fd25492c527c277f"
  _type = "bug_algorithms/nodeState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 algorithm
uint8 node_state
string node_state_desc
float32 node_state_time
uint8 bug_state
string bug_state_desc
float32 bug_state_time
"""
  __slots__ = ['algorithm','node_state','node_state_desc','node_state_time','bug_state','bug_state_desc','bug_state_time']
  _slot_types = ['uint8','uint8','string','float32','uint8','string','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       algorithm,node_state,node_state_desc,node_state_time,bug_state,bug_state_desc,bug_state_time

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(nodeState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.algorithm is None:
        self.algorithm = 0
      if self.node_state is None:
        self.node_state = 0
      if self.node_state_desc is None:
        self.node_state_desc = ''
      if self.node_state_time is None:
        self.node_state_time = 0.
      if self.bug_state is None:
        self.bug_state = 0
      if self.bug_state_desc is None:
        self.bug_state_desc = ''
      if self.bug_state_time is None:
        self.bug_state_time = 0.
    else:
      self.algorithm = 0
      self.node_state = 0
      self.node_state_desc = ''
      self.node_state_time = 0.
      self.bug_state = 0
      self.bug_state_desc = ''
      self.bug_state_time = 0.

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
      buff.write(_get_struct_2B().pack(_x.algorithm, _x.node_state))
      _x = self.node_state_desc
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_fB().pack(_x.node_state_time, _x.bug_state))
      _x = self.bug_state_desc
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_get_struct_f().pack(self.bug_state_time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.algorithm, _x.node_state,) = _get_struct_2B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node_state_desc = str[start:end].decode('utf-8')
      else:
        self.node_state_desc = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.node_state_time, _x.bug_state,) = _get_struct_fB().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.bug_state_desc = str[start:end].decode('utf-8')
      else:
        self.bug_state_desc = str[start:end]
      start = end
      end += 4
      (self.bug_state_time,) = _get_struct_f().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_2B().pack(_x.algorithm, _x.node_state))
      _x = self.node_state_desc
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_fB().pack(_x.node_state_time, _x.bug_state))
      _x = self.bug_state_desc
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_get_struct_f().pack(self.bug_state_time))
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
      _x = self
      start = end
      end += 2
      (_x.algorithm, _x.node_state,) = _get_struct_2B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node_state_desc = str[start:end].decode('utf-8')
      else:
        self.node_state_desc = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.node_state_time, _x.bug_state,) = _get_struct_fB().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.bug_state_desc = str[start:end].decode('utf-8')
      else:
        self.bug_state_desc = str[start:end]
      start = end
      end += 4
      (self.bug_state_time,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_fB = None
def _get_struct_fB():
    global _struct_fB
    if _struct_fB is None:
        _struct_fB = struct.Struct("<fB")
    return _struct_fB
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
