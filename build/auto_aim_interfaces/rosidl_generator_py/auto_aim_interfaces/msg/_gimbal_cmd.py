# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/GimbalCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalCmd(type):
    """Metaclass of message 'GimbalCmd'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('auto_aim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim_interfaces.msg.GimbalCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gimbal_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gimbal_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gimbal_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gimbal_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gimbal_cmd

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GimbalCmd(metaclass=Metaclass_GimbalCmd):
    """Message class 'GimbalCmd'."""

    __slots__ = [
        '_header',
        '_control',
        '_fire',
        '_target_yaw',
        '_target_pitch',
        '_yaw',
        '_yaw_vel',
        '_yaw_acc',
        '_pitch',
        '_pitch_vel',
        '_pitch_acc',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'control': 'boolean',
        'fire': 'boolean',
        'target_yaw': 'double',
        'target_pitch': 'double',
        'yaw': 'double',
        'yaw_vel': 'double',
        'yaw_acc': 'double',
        'pitch': 'double',
        'pitch_vel': 'double',
        'pitch_acc': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.control = kwargs.get('control', bool())
        self.fire = kwargs.get('fire', bool())
        self.target_yaw = kwargs.get('target_yaw', float())
        self.target_pitch = kwargs.get('target_pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.yaw_vel = kwargs.get('yaw_vel', float())
        self.yaw_acc = kwargs.get('yaw_acc', float())
        self.pitch = kwargs.get('pitch', float())
        self.pitch_vel = kwargs.get('pitch_vel', float())
        self.pitch_acc = kwargs.get('pitch_acc', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.control != other.control:
            return False
        if self.fire != other.fire:
            return False
        if self.target_yaw != other.target_yaw:
            return False
        if self.target_pitch != other.target_pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.yaw_vel != other.yaw_vel:
            return False
        if self.yaw_acc != other.yaw_acc:
            return False
        if self.pitch != other.pitch:
            return False
        if self.pitch_vel != other.pitch_vel:
            return False
        if self.pitch_acc != other.pitch_acc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def control(self):
        """Message field 'control'."""
        return self._control

    @control.setter
    def control(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'control' field must be of type 'bool'"
        self._control = value

    @builtins.property
    def fire(self):
        """Message field 'fire'."""
        return self._fire

    @fire.setter
    def fire(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fire' field must be of type 'bool'"
        self._fire = value

    @builtins.property
    def target_yaw(self):
        """Message field 'target_yaw'."""
        return self._target_yaw

    @target_yaw.setter
    def target_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_yaw = value

    @builtins.property
    def target_pitch(self):
        """Message field 'target_pitch'."""
        return self._target_pitch

    @target_pitch.setter
    def target_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_pitch = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw = value

    @builtins.property
    def yaw_vel(self):
        """Message field 'yaw_vel'."""
        return self._yaw_vel

    @yaw_vel.setter
    def yaw_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_vel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_vel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_vel = value

    @builtins.property
    def yaw_acc(self):
        """Message field 'yaw_acc'."""
        return self._yaw_acc

    @yaw_acc.setter
    def yaw_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_acc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_acc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_acc = value

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch = value

    @builtins.property
    def pitch_vel(self):
        """Message field 'pitch_vel'."""
        return self._pitch_vel

    @pitch_vel.setter
    def pitch_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_vel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_vel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_vel = value

    @builtins.property
    def pitch_acc(self):
        """Message field 'pitch_acc'."""
        return self._pitch_acc

    @pitch_acc.setter
    def pitch_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_acc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_acc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_acc = value
