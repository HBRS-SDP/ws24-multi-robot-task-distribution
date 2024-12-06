# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot3_msgs:srv/Dqn.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Dqn_Request(type):
    """Metaclass of message 'Dqn_Request'."""

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
                'turtlebot3_msgs.srv.Dqn_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__dqn__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__dqn__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__dqn__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__dqn__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__dqn__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Dqn_Request(metaclass=Metaclass_Dqn_Request):
    """Message class 'Dqn_Request'."""

    __slots__ = [
        '_action',
        '_init',
    ]

    _fields_and_field_types = {
        'action': 'uint8',
        'init': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.action = kwargs.get('action', int())
        self.init = kwargs.get('init', bool())

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
        if self.action != other.action:
            return False
        if self.init != other.init:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def action(self):
        """Message field 'action'."""
        return self._action

    @action.setter
    def action(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'action' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'action' field must be an unsigned integer in [0, 255]"
        self._action = value

    @builtins.property
    def init(self):
        """Message field 'init'."""
        return self._init

    @init.setter
    def init(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'init' field must be of type 'bool'"
        self._init = value


# Import statements for member types

# Member 'state'
import array  # noqa: E402, I100

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_Dqn_Response(type):
    """Metaclass of message 'Dqn_Response'."""

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
                'turtlebot3_msgs.srv.Dqn_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__dqn__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__dqn__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__dqn__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__dqn__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__dqn__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Dqn_Response(metaclass=Metaclass_Dqn_Response):
    """Message class 'Dqn_Response'."""

    __slots__ = [
        '_state',
        '_reward',
        '_done',
    ]

    _fields_and_field_types = {
        'state': 'sequence<float>',
        'reward': 'float',
        'done': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.state = array.array('f', kwargs.get('state', []))
        self.reward = kwargs.get('reward', float())
        self.done = kwargs.get('done', bool())

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
        if self.state != other.state:
            return False
        if self.reward != other.reward:
            return False
        if self.done != other.done:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'state' array.array() must have the type code of 'f'"
            self._state = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'state' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._state = array.array('f', value)

    @builtins.property
    def reward(self):
        """Message field 'reward'."""
        return self._reward

    @reward.setter
    def reward(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'reward' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'reward' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._reward = value

    @builtins.property
    def done(self):
        """Message field 'done'."""
        return self._done

    @done.setter
    def done(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'done' field must be of type 'bool'"
        self._done = value


class Metaclass_Dqn(type):
    """Metaclass of service 'Dqn'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('turtlebot3_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_msgs.srv.Dqn')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__dqn

            from turtlebot3_msgs.srv import _dqn
            if _dqn.Metaclass_Dqn_Request._TYPE_SUPPORT is None:
                _dqn.Metaclass_Dqn_Request.__import_type_support__()
            if _dqn.Metaclass_Dqn_Response._TYPE_SUPPORT is None:
                _dqn.Metaclass_Dqn_Response.__import_type_support__()


class Dqn(metaclass=Metaclass_Dqn):
    from turtlebot3_msgs.srv._dqn import Dqn_Request as Request
    from turtlebot3_msgs.srv._dqn import Dqn_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
