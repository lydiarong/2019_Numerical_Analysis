// Generated by gencpp from file kinova_msgs/SetEndEffectorOffsetRequest.msg
// DO NOT EDIT!


#ifndef KINOVA_MSGS_MESSAGE_SETENDEFFECTOROFFSETREQUEST_H
#define KINOVA_MSGS_MESSAGE_SETENDEFFECTOROFFSETREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>

namespace kinova_msgs
{
template <class ContainerAllocator>
struct SetEndEffectorOffsetRequest_
{
  typedef SetEndEffectorOffsetRequest_<ContainerAllocator> Type;

  SetEndEffectorOffsetRequest_()
    : status(0)
    , offset()  {
    }
  SetEndEffectorOffsetRequest_(const ContainerAllocator& _alloc)
    : status(0)
    , offset(_alloc)  {
  (void)_alloc;
    }



   typedef uint16_t _status_type;
  _status_type status;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _offset_type;
  _offset_type offset;





  typedef boost::shared_ptr< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetEndEffectorOffsetRequest_

typedef ::kinova_msgs::SetEndEffectorOffsetRequest_<std::allocator<void> > SetEndEffectorOffsetRequest;

typedef boost::shared_ptr< ::kinova_msgs::SetEndEffectorOffsetRequest > SetEndEffectorOffsetRequestPtr;
typedef boost::shared_ptr< ::kinova_msgs::SetEndEffectorOffsetRequest const> SetEndEffectorOffsetRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7a5c04366489f137e01b31118dcce900";
  }

  static const char* value(const ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7a5c04366489f137ULL;
  static const uint64_t static_value2 = 0xe01b31118dcce900ULL;
};

template<class ContainerAllocator>
struct DataType< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinova_msgs/SetEndEffectorOffsetRequest";
  }

  static const char* value(const ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 status\n\
geometry_msgs/Vector3 offset\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
      stream.next(m.offset);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetEndEffectorOffsetRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinova_msgs::SetEndEffectorOffsetRequest_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.status);
    s << indent << "offset: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.offset);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINOVA_MSGS_MESSAGE_SETENDEFFECTOROFFSETREQUEST_H
