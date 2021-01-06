// Generated by gencpp from file iiwa_msgs/SetSmartServoLinSpeedLimitsRequest.msg
// DO NOT EDIT!


#ifndef IIWA_MSGS_MESSAGE_SETSMARTSERVOLINSPEEDLIMITSREQUEST_H
#define IIWA_MSGS_MESSAGE_SETSMARTSERVOLINSPEEDLIMITSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Twist.h>

namespace iiwa_msgs
{
template <class ContainerAllocator>
struct SetSmartServoLinSpeedLimitsRequest_
{
  typedef SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> Type;

  SetSmartServoLinSpeedLimitsRequest_()
    : max_cartesian_velocity()  {
    }
  SetSmartServoLinSpeedLimitsRequest_(const ContainerAllocator& _alloc)
    : max_cartesian_velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _max_cartesian_velocity_type;
  _max_cartesian_velocity_type max_cartesian_velocity;





  typedef boost::shared_ptr< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetSmartServoLinSpeedLimitsRequest_

typedef ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<std::allocator<void> > SetSmartServoLinSpeedLimitsRequest;

typedef boost::shared_ptr< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest > SetSmartServoLinSpeedLimitsRequestPtr;
typedef boost::shared_ptr< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest const> SetSmartServoLinSpeedLimitsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator1> & lhs, const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.max_cartesian_velocity == rhs.max_cartesian_velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator1> & lhs, const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace iiwa_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b0bfa3511a969819a8206a03ae719380";
  }

  static const char* value(const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb0bfa3511a969819ULL;
  static const uint64_t static_value2 = 0xa8206a03ae719380ULL;
};

template<class ContainerAllocator>
struct DataType< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "iiwa_msgs/SetSmartServoLinSpeedLimitsRequest";
  }

  static const char* value(const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Translational and rotational speed in m/s and rad/s\n"
"geometry_msgs/Twist max_cartesian_velocity\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.max_cartesian_velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetSmartServoLinSpeedLimitsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::iiwa_msgs::SetSmartServoLinSpeedLimitsRequest_<ContainerAllocator>& v)
  {
    s << indent << "max_cartesian_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.max_cartesian_velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IIWA_MSGS_MESSAGE_SETSMARTSERVOLINSPEEDLIMITSREQUEST_H
