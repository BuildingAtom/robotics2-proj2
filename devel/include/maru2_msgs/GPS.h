// Generated by gencpp from file maru2_msgs/GPS.msg
// DO NOT EDIT!


#ifndef MARU2_MSGS_MESSAGE_GPS_H
#define MARU2_MSGS_MESSAGE_GPS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace maru2_msgs
{
template <class ContainerAllocator>
struct GPS_
{
  typedef GPS_<ContainerAllocator> Type;

  GPS_()
    : header()
    , beacon00(0.0)
    , beacon01(0.0)
    , beacon10(0.0)
    , beacon11(0.0)  {
    }
  GPS_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , beacon00(0.0)
    , beacon01(0.0)
    , beacon10(0.0)
    , beacon11(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _beacon00_type;
  _beacon00_type beacon00;

   typedef double _beacon01_type;
  _beacon01_type beacon01;

   typedef double _beacon10_type;
  _beacon10_type beacon10;

   typedef double _beacon11_type;
  _beacon11_type beacon11;





  typedef boost::shared_ptr< ::maru2_msgs::GPS_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::maru2_msgs::GPS_<ContainerAllocator> const> ConstPtr;

}; // struct GPS_

typedef ::maru2_msgs::GPS_<std::allocator<void> > GPS;

typedef boost::shared_ptr< ::maru2_msgs::GPS > GPSPtr;
typedef boost::shared_ptr< ::maru2_msgs::GPS const> GPSConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::maru2_msgs::GPS_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::maru2_msgs::GPS_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::maru2_msgs::GPS_<ContainerAllocator1> & lhs, const ::maru2_msgs::GPS_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.beacon00 == rhs.beacon00 &&
    lhs.beacon01 == rhs.beacon01 &&
    lhs.beacon10 == rhs.beacon10 &&
    lhs.beacon11 == rhs.beacon11;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::maru2_msgs::GPS_<ContainerAllocator1> & lhs, const ::maru2_msgs::GPS_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace maru2_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::maru2_msgs::GPS_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::maru2_msgs::GPS_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::maru2_msgs::GPS_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::maru2_msgs::GPS_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::maru2_msgs::GPS_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::maru2_msgs::GPS_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::maru2_msgs::GPS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6a88cfec89b2d3899365b3c2815c9280";
  }

  static const char* value(const ::maru2_msgs::GPS_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6a88cfec89b2d389ULL;
  static const uint64_t static_value2 = 0x9365b3c2815c9280ULL;
};

template<class ContainerAllocator>
struct DataType< ::maru2_msgs::GPS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "maru2_msgs/GPS";
  }

  static const char* value(const ::maru2_msgs::GPS_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::maru2_msgs::GPS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# A basic message for the local GPS beacons.\n"
"\n"
"Header header       # Reference shenanigans\n"
"\n"
"float64 beacon00    # Distance [m] from the beacon at [0,0,4]\n"
"float64 beacon01    # Distance [m] from the beacon at [0,10,4]\n"
"float64 beacon10    # Distance [m] from the beacon at [10,0,4]\n"
"float64 beacon11    # Distance [m] from the beacon at [10,10,4]\n"
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

  static const char* value(const ::maru2_msgs::GPS_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::maru2_msgs::GPS_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.beacon00);
      stream.next(m.beacon01);
      stream.next(m.beacon10);
      stream.next(m.beacon11);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPS_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::maru2_msgs::GPS_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::maru2_msgs::GPS_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "beacon00: ";
    Printer<double>::stream(s, indent + "  ", v.beacon00);
    s << indent << "beacon01: ";
    Printer<double>::stream(s, indent + "  ", v.beacon01);
    s << indent << "beacon10: ";
    Printer<double>::stream(s, indent + "  ", v.beacon10);
    s << indent << "beacon11: ";
    Printer<double>::stream(s, indent + "  ", v.beacon11);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MARU2_MSGS_MESSAGE_GPS_H
