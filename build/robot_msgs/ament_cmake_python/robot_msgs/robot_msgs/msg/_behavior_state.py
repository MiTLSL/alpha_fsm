# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:msg/BehaviorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BehaviorState(type):
    """Metaclass of message 'BehaviorState'."""

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
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.msg.BehaviorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__behavior_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__behavior_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__behavior_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__behavior_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__behavior_state

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


class BehaviorState(metaclass=Metaclass_BehaviorState):
    """Message class 'BehaviorState'."""

    __slots__ = [
        '_header',
        '_system_state',
        '_task_state',
        '_wall_state',
        '_active_substate',
        '_task_id',
        '_last_error_code',
        '_summary_json',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'system_state': 'string',
        'task_state': 'string',
        'wall_state': 'string',
        'active_substate': 'string',
        'task_id': 'string',
        'last_error_code': 'uint16',
        'summary_json': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.system_state = kwargs.get('system_state', str())
        self.task_state = kwargs.get('task_state', str())
        self.wall_state = kwargs.get('wall_state', str())
        self.active_substate = kwargs.get('active_substate', str())
        self.task_id = kwargs.get('task_id', str())
        self.last_error_code = kwargs.get('last_error_code', int())
        self.summary_json = kwargs.get('summary_json', str())

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
        if self.system_state != other.system_state:
            return False
        if self.task_state != other.task_state:
            return False
        if self.wall_state != other.wall_state:
            return False
        if self.active_substate != other.active_substate:
            return False
        if self.task_id != other.task_id:
            return False
        if self.last_error_code != other.last_error_code:
            return False
        if self.summary_json != other.summary_json:
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
    def system_state(self):
        """Message field 'system_state'."""
        return self._system_state

    @system_state.setter
    def system_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'system_state' field must be of type 'str'"
        self._system_state = value

    @builtins.property
    def task_state(self):
        """Message field 'task_state'."""
        return self._task_state

    @task_state.setter
    def task_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_state' field must be of type 'str'"
        self._task_state = value

    @builtins.property
    def wall_state(self):
        """Message field 'wall_state'."""
        return self._wall_state

    @wall_state.setter
    def wall_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'wall_state' field must be of type 'str'"
        self._wall_state = value

    @builtins.property
    def active_substate(self):
        """Message field 'active_substate'."""
        return self._active_substate

    @active_substate.setter
    def active_substate(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'active_substate' field must be of type 'str'"
        self._active_substate = value

    @builtins.property
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_id' field must be of type 'str'"
        self._task_id = value

    @builtins.property
    def last_error_code(self):
        """Message field 'last_error_code'."""
        return self._last_error_code

    @last_error_code.setter
    def last_error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'last_error_code' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'last_error_code' field must be an unsigned integer in [0, 65535]"
        self._last_error_code = value

    @builtins.property
    def summary_json(self):
        """Message field 'summary_json'."""
        return self._summary_json

    @summary_json.setter
    def summary_json(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'summary_json' field must be of type 'str'"
        self._summary_json = value
