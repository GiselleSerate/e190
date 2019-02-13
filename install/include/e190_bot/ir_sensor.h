// Generated by gencpp from file e190_bot/ir_sensor.msg
// DO NOT EDIT!


#ifndef E190_BOT_MESSAGE_IR_SENSOR_H
#define E190_BOT_MESSAGE_IR_SENSOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace e190_bot
{
template <class ContainerAllocator>
struct ir_sensor_
{
  typedef ir_sensor_<ContainerAllocator> Type;

  ir_sensor_()
    : distance(0.0)  {
    }
  ir_sensor_(const ContainerAllocator& _alloc)
    : distance(0.0)  {
  (void)_alloc;
    }



   typedef float _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::e190_bot::ir_sensor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::e190_bot::ir_sensor_<ContainerAllocator> const> ConstPtr;

}; // struct ir_sensor_

typedef ::e190_bot::ir_sensor_<std::allocator<void> > ir_sensor;

typedef boost::shared_ptr< ::e190_bot::ir_sensor > ir_sensorPtr;
typedef boost::shared_ptr< ::e190_bot::ir_sensor const> ir_sensorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::e190_bot::ir_sensor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::e190_bot::ir_sensor_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace e190_bot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'e190_bot': ['/home/gserate/e190_ws/src/e190_bot/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::e190_bot::ir_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::e190_bot::ir_sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::e190_bot::ir_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::e190_bot::ir_sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::e190_bot::ir_sensor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::e190_bot::ir_sensor_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::e190_bot::ir_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6e77fb10f0c8b4833ec273aa9ac74459";
  }

  static const char* value(const ::e190_bot::ir_sensor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6e77fb10f0c8b483ULL;
  static const uint64_t static_value2 = 0x3ec273aa9ac74459ULL;
};

template<class ContainerAllocator>
struct DataType< ::e190_bot::ir_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e190_bot/ir_sensor";
  }

  static const char* value(const ::e190_bot::ir_sensor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::e190_bot::ir_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 distance\n\
";
  }

  static const char* value(const ::e190_bot::ir_sensor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::e190_bot::ir_sensor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ir_sensor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::e190_bot::ir_sensor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::e190_bot::ir_sensor_<ContainerAllocator>& v)
  {
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // E190_BOT_MESSAGE_IR_SENSOR_H