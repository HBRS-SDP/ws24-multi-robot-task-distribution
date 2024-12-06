# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot3_msgs:msg/SensorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SensorState(type):
    """Metaclass of message 'SensorState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'BUMPER_FORWARD': 1,
        'BUMPER_BACKWARD': 2,
        'CLIFF': 1,
        'SONAR': 1,
        'ILLUMINATION': 1,
        'BUTTON0': 1,
        'BUTTON1': 2,
        'ERROR_LEFT_MOTOR': 1,
        'ERROR_RIGHT_MOTOR': 2,
        'TORQUE_ON': 1,
        'TORQUE_OFF': 2,
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
                'turtlebot3_msgs.msg.SensorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sensor_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sensor_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sensor_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sensor_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sensor_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'BUMPER_FORWARD': cls.__constants['BUMPER_FORWARD'],
            'BUMPER_BACKWARD': cls.__constants['BUMPER_BACKWARD'],
            'CLIFF': cls.__constants['CLIFF'],
            'SONAR': cls.__constants['SONAR'],
            'ILLUMINATION': cls.__constants['ILLUMINATION'],
            'BUTTON0': cls.__constants['BUTTON0'],
            'BUTTON1': cls.__constants['BUTTON1'],
            'ERROR_LEFT_MOTOR': cls.__constants['ERROR_LEFT_MOTOR'],
            'ERROR_RIGHT_MOTOR': cls.__constants['ERROR_RIGHT_MOTOR'],
            'TORQUE_ON': cls.__constants['TORQUE_ON'],
            'TORQUE_OFF': cls.__constants['TORQUE_OFF'],
        }

    @property
    def BUMPER_FORWARD(self):
        """Message constant 'BUMPER_FORWARD'."""
        return Metaclass_SensorState.__constants['BUMPER_FORWARD']

    @property
    def BUMPER_BACKWARD(self):
        """Message constant 'BUMPER_BACKWARD'."""
        return Metaclass_SensorState.__constants['BUMPER_BACKWARD']

    @property
    def CLIFF(self):
        """Message constant 'CLIFF'."""
        return Metaclass_SensorState.__constants['CLIFF']

    @property
    def SONAR(self):
        """Message constant 'SONAR'."""
        return Metaclass_SensorState.__constants['SONAR']

    @property
    def ILLUMINATION(self):
        """Message constant 'ILLUMINATION'."""
        return Metaclass_SensorState.__constants['ILLUMINATION']

    @property
    def BUTTON0(self):
        """Message constant 'BUTTON0'."""
        return Metaclass_SensorState.__constants['BUTTON0']

    @property
    def BUTTON1(self):
        """Message constant 'BUTTON1'."""
        return Metaclass_SensorState.__constants['BUTTON1']

    @property
    def ERROR_LEFT_MOTOR(self):
        """Message constant 'ERROR_LEFT_MOTOR'."""
        return Metaclass_SensorState.__constants['ERROR_LEFT_MOTOR']

    @property
    def ERROR_RIGHT_MOTOR(self):
        """Message constant 'ERROR_RIGHT_MOTOR'."""
        return Metaclass_SensorState.__constants['ERROR_RIGHT_MOTOR']

    @property
    def TORQUE_ON(self):
        """Message constant 'TORQUE_ON'."""
        return Metaclass_SensorState.__constants['TORQUE_ON']

    @property
    def TORQUE_OFF(self):
        """Message constant 'TORQUE_OFF'."""
        return Metaclass_SensorState.__constants['TORQUE_OFF']


class SensorState(metaclass=Metaclass_SensorState):
    """
    Message class 'SensorState'.

    Constants:
      BUMPER_FORWARD
      BUMPER_BACKWARD
      CLIFF
      SONAR
      ILLUMINATION
      BUTTON0
      BUTTON1
      ERROR_LEFT_MOTOR
      ERROR_RIGHT_MOTOR
      TORQUE_ON
      TORQUE_OFF
    """

    __slots__ = [
        '_header',
        '_bumper',
        '_cliff',
        '_sonar',
        '_illumination',
        '_led',
        '_button',
        '_torque',
        '_left_encoder',
        '_right_encoder',
        '_battery',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'bumper': 'uint8',
        'cliff': 'float',
        'sonar': 'float',
        'illumination': 'float',
        'led': 'uint8',
        'button': 'uint8',
        'torque': 'boolean',
        'left_encoder': 'int32',
        'right_encoder': 'int32',
        'battery': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.bumper = kwargs.get('bumper', int())
        self.cliff = kwargs.get('cliff', float())
        self.sonar = kwargs.get('sonar', float())
        self.illumination = kwargs.get('illumination', float())
        self.led = kwargs.get('led', int())
        self.button = kwargs.get('button', int())
        self.torque = kwargs.get('torque', bool())
        self.left_encoder = kwargs.get('left_encoder', int())
        self.right_encoder = kwargs.get('right_encoder', int())
        self.battery = kwargs.get('battery', float())

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
        if self.bumper != other.bumper:
            return False
        if self.cliff != other.cliff:
            return False
        if self.sonar != other.sonar:
            return False
        if self.illumination != other.illumination:
            return False
        if self.led != other.led:
            return False
        if self.button != other.button:
            return False
        if self.torque != other.torque:
            return False
        if self.left_encoder != other.left_encoder:
            return False
        if self.right_encoder != other.right_encoder:
            return False
        if self.battery != other.battery:
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
    def bumper(self):
        """Message field 'bumper'."""
        return self._bumper

    @bumper.setter
    def bumper(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bumper' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'bumper' field must be an unsigned integer in [0, 255]"
        self._bumper = value

    @builtins.property
    def cliff(self):
        """Message field 'cliff'."""
        return self._cliff

    @cliff.setter
    def cliff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cliff' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cliff' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cliff = value

    @builtins.property
    def sonar(self):
        """Message field 'sonar'."""
        return self._sonar

    @sonar.setter
    def sonar(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'sonar' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'sonar' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._sonar = value

    @builtins.property
    def illumination(self):
        """Message field 'illumination'."""
        return self._illumination

    @illumination.setter
    def illumination(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'illumination' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'illumination' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._illumination = value

    @builtins.property
    def led(self):
        """Message field 'led'."""
        return self._led

    @led.setter
    def led(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'led' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'led' field must be an unsigned integer in [0, 255]"
        self._led = value

    @builtins.property
    def button(self):
        """Message field 'button'."""
        return self._button

    @button.setter
    def button(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'button' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'button' field must be an unsigned integer in [0, 255]"
        self._button = value

    @builtins.property
    def torque(self):
        """Message field 'torque'."""
        return self._torque

    @torque.setter
    def torque(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'torque' field must be of type 'bool'"
        self._torque = value

    @builtins.property
    def left_encoder(self):
        """Message field 'left_encoder'."""
        return self._left_encoder

    @left_encoder.setter
    def left_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._left_encoder = value

    @builtins.property
    def right_encoder(self):
        """Message field 'right_encoder'."""
        return self._right_encoder

    @right_encoder.setter
    def right_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._right_encoder = value

    @builtins.property
    def battery(self):
        """Message field 'battery'."""
        return self._battery

    @battery.setter
    def battery(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery = value
