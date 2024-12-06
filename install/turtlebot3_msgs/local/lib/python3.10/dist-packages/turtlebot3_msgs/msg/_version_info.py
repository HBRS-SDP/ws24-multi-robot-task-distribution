# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot3_msgs:msg/VersionInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VersionInfo(type):
    """Metaclass of message 'VersionInfo'."""

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
            module = import_type_support('turtlebot3_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_msgs.msg.VersionInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__version_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__version_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__version_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__version_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__version_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VersionInfo(metaclass=Metaclass_VersionInfo):
    """Message class 'VersionInfo'."""

    __slots__ = [
        '_hardware',
        '_firmware',
        '_software',
    ]

    _fields_and_field_types = {
        'hardware': 'string',
        'firmware': 'string',
        'software': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.hardware = kwargs.get('hardware', str())
        self.firmware = kwargs.get('firmware', str())
        self.software = kwargs.get('software', str())

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
        if self.hardware != other.hardware:
            return False
        if self.firmware != other.firmware:
            return False
        if self.software != other.software:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def hardware(self):
        """Message field 'hardware'."""
        return self._hardware

    @hardware.setter
    def hardware(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'hardware' field must be of type 'str'"
        self._hardware = value

    @builtins.property
    def firmware(self):
        """Message field 'firmware'."""
        return self._firmware

    @firmware.setter
    def firmware(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'firmware' field must be of type 'str'"
        self._firmware = value

    @builtins.property
    def software(self):
        """Message field 'software'."""
        return self._software

    @software.setter
    def software(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'software' field must be of type 'str'"
        self._software = value
