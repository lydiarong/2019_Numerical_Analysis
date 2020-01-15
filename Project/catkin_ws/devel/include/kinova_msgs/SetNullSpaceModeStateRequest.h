// Generated by gencpp from file kinova_msgs/SetNullSpaceModeStateRequest.msg
// DO NOT EDIT!


#ifndef KINOVA_MSGS_MESSAGE_SETNULLSPACEMODESTATEREQUEST_H
#define KINOVA_MSGS_MESSAGE_SETNULLSPACEMODESTATEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kinova_msgs
{
template <class ContainerAllocator>
struct SetNullSpaceModeStateRequest_
{
  typedef SetNullSpaceModeStateRequest_<ContainerAllocator> Type;

  SetNullSpaceModeStateRequest_()
    : state(0)  {
    }
  SetNullSpaceModeStateRequest_(const ContainerAllocator& _alloc)
    : state(0)  {
  (void)_alloc;
    }



   typedef uint16_t _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetNullSpaceModeStateRequest_

typedef ::kinova_msgs::SetNullSpaceModeStateRequest_<std::allocator<void> > SetNullSpaceModeStateRequest;

typedef boost::shared_ptr< ::kinova_msgs::SetNullSpaceModeStateRequest > SetNullSpaceModeStateRequestPtr;
typedef boost::shared_ptr< ::kinova_msgs::SetNullSpaceModeStateRequest const> SetNullSpaceModeStateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace kinova_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'kinova_msgs': ['/home/lima/catkin_ws/src/kinova-ros/kinova_msgs/msg', '/home/lima/catkin_ws/devel/share/kinova_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "891b541ef99af7889d0f22a062410be8";
  }

  static const char* value(const ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x891b541ef99af788ULL;
  static const uint64_t static_value2 = 0x9d0f22a062410be8ULL;
};

template<class ContainerAllocator>
struct DataType< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinova_msgs/SetNullSpaceModeStateRequest";
  }

  static const char* value(const ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 state\n\
";
  }

  static const char* value(const ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetNullSpaceModeStateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinova_msgs::SetNullSpaceModeStateRequest_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINOVA_MSGS_MESSAGE_SETNULLSPACEMODESTATEREQUEST_H
