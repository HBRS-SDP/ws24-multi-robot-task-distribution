# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot3_msgs:msg/Sound.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Sound(type):
    """Metaclass of message 'Sound'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OFF': 0,
        'ON': 1,
        'LOW_BATTERY': 2,
        'ERROR': 3,
        'BUTTON1': 4,
        'BUTTON2': 5,
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
                'turtlebot3_msgs.msg.Sound')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sound
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sound
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sound
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sound
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sound

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OFF': cls.__constants['OFF'],
            'ON': cls.__constants['ON'],
            'LOW_BATTERY': cls.__constants['LOW_BATTERY'],
            'ERROR': cls.__constants['ERROR'],
            'BUTTON1': cls.__constants['BUTTON1'],
            'BUTTON2': cls.__constants['BUTTON2'],
        }

    @property
    def OFF(self):
        """Message constant 'OFF'."""
        return Metaclass_Sound.__constants['OFF']

    @property
    def ON(self):
        """Message constant 'ON'."""
        return Metaclass_Sound.__constants['ON']

    @property
    def LOW_BATTERY(self):
        """Message constant 'LOW_BATTERY'."""
        return Metaclass_Sound.__constants['LOW_BATTERY']

    @property
    def ERROR(self):
        """Message constant 'ERROR'."""
        return Metaclass_Sound.__constants['ERROR']

    @property
    def BUTTON1(self):
        """Message constant 'BUTTON1'."""
        return Metaclass_Sound.__constants['BUTTON1']

    @property
    def BUTTON2(self):
        """Message constant 'BUTTON2'."""
        return Metaclass_Sound.__constants['BUTTON2']


class Sound(metaclass=Metaclass_Sound):
    """
    Message class 'Sound'.

    Constants:
      OFF
      ON
      LOW_BATTERY
      ERROR
      BUTTON1
      BUTTON2
    """

    __slots__ = [
        '_value',
    ]

    _fields_and_field_types = {
        'value': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.value = kwargs.get('value', int())

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
        if self.value != other.value:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'value' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'value' field must be an unsigned integer in [0, 255]"
        self._value = value
