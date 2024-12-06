// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_msgs/msg/detail/sensor_state__struct.h"
#include "turtlebot3_msgs/msg/detail/sensor_state__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool turtlebot3_msgs__msg__sensor_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("turtlebot3_msgs.msg._sensor_state.SensorState", full_classname_dest, 45) == 0);
  }
  turtlebot3_msgs__msg__SensorState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // bumper
    PyObject * field = PyObject_GetAttrString(_pymsg, "bumper");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bumper = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cliff
    PyObject * field = PyObject_GetAttrString(_pymsg, "cliff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cliff = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sonar
    PyObject * field = PyObject_GetAttrString(_pymsg, "sonar");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sonar = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // illumination
    PyObject * field = PyObject_GetAttrString(_pymsg, "illumination");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->illumination = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // led
    PyObject * field = PyObject_GetAttrString(_pymsg, "led");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->led = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // button
    PyObject * field = PyObject_GetAttrString(_pymsg, "button");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->button = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "torque");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->torque = (Py_True == field);
    Py_DECREF(field);
  }
  {  // left_encoder
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_encoder");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left_encoder = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // right_encoder
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_encoder");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right_encoder = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // battery
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * turtlebot3_msgs__msg__sensor_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SensorState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("turtlebot3_msgs.msg._sensor_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SensorState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  turtlebot3_msgs__msg__SensorState * ros_message = (turtlebot3_msgs__msg__SensorState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bumper
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->bumper);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bumper", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cliff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cliff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cliff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sonar
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sonar);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sonar", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // illumination
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->illumination);
    {
      int rc = PyObject_SetAttrString(_pymessage, "illumination", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // led
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->led);
    {
      int rc = PyObject_SetAttrString(_pymessage, "led", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // button
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->button);
    {
      int rc = PyObject_SetAttrString(_pymessage, "button", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // torque
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->torque ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "torque", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_encoder
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->left_encoder);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_encoder", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_encoder
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->right_encoder);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_encoder", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
