// Generated by gencpp from file quadrotor_msgs/SwarmInfo.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_SWARMINFO_H
#define QUADROTOR_MSGS_MESSAGE_SWARMINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <quadrotor_msgs/TrajectoryMatrix.h>

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct SwarmInfo_
{
  typedef SwarmInfo_<ContainerAllocator> Type;

  SwarmInfo_()
    : path()
    , start()  {
    }
  SwarmInfo_(const ContainerAllocator& _alloc)
    : path(_alloc)
    , start()  {
  (void)_alloc;
    }



   typedef  ::quadrotor_msgs::TrajectoryMatrix_<ContainerAllocator>  _path_type;
  _path_type path;

   typedef ros::Time _start_type;
  _start_type start;





  typedef boost::shared_ptr< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> const> ConstPtr;

}; // struct SwarmInfo_

typedef ::quadrotor_msgs::SwarmInfo_<std::allocator<void> > SwarmInfo;

typedef boost::shared_ptr< ::quadrotor_msgs::SwarmInfo > SwarmInfoPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::SwarmInfo const> SwarmInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator2> & rhs)
{
  return lhs.path == rhs.path &&
    lhs.start == rhs.start;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "908ae631e70132160c2527a9926df867";
  }

  static const char* value(const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x908ae631e7013216ULL;
  static const uint64_t static_value2 = 0x0c2527a9926df867ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/SwarmInfo";
  }

  static const char* value(const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/TrajectoryMatrix path\n"
"time start\n"
"\n"
"================================================================================\n"
"MSG: quadrotor_msgs/TrajectoryMatrix\n"
"#type\n"
"uint8 TYPE_UNKNOWN = 0\n"
"uint8 TYPE_POLY    = 1\n"
"uint8 TYPE_TIME    = 2\n"
"\n"
"#data structure\n"
"Header    header\n"
"uint8     type\n"
"uint32    id\n"
"string    name\n"
"uint32[]  format\n"
"float64[] data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.path);
      stream.next(m.start);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SwarmInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::SwarmInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::SwarmInfo_<ContainerAllocator>& v)
  {
    s << indent << "path: ";
    s << std::endl;
    Printer< ::quadrotor_msgs::TrajectoryMatrix_<ContainerAllocator> >::stream(s, indent + "  ", v.path);
    s << indent << "start: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_SWARMINFO_H
