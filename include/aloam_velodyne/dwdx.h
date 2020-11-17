// Generated by gencpp from file aloam_velodyne/dwdx.msg
// DO NOT EDIT!


#ifndef ALOAM_VELODYNE_MESSAGE_DWDX_H
#define ALOAM_VELODYNE_MESSAGE_DWDX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace aloam_velodyne
{
template <class ContainerAllocator>
struct dwdx_
{
  typedef dwdx_<ContainerAllocator> Type;

  dwdx_()
    : header()
    , time_stamp(0.0)
    , global_x(0)
    , global_y(0)
    , global_h(0)
    , zone(0)
    , longitude(0)
    , latitude(0)
    , heading(0)
    , pitch(0)
    , roll(0)
    , global_vx(0)
    , global_vy(0)
    , global_vz(0)
    , global_wx(0)
    , global_wy(0)
    , global_wz(0)
    , mileage(0)
    , belief(0)
    , baseline(0.0)
    , NSV1(0)
    , NSV2(0)
    , status(0)
    , accx(0)
    , accy(0)
    , accz(0)
    , tpr(0)  {
    }
  dwdx_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time_stamp(0.0)
    , global_x(0)
    , global_y(0)
    , global_h(0)
    , zone(0)
    , longitude(0)
    , latitude(0)
    , heading(0)
    , pitch(0)
    , roll(0)
    , global_vx(0)
    , global_vy(0)
    , global_vz(0)
    , global_wx(0)
    , global_wy(0)
    , global_wz(0)
    , mileage(0)
    , belief(0)
    , baseline(0.0)
    , NSV1(0)
    , NSV2(0)
    , status(0)
    , accx(0)
    , accy(0)
    , accz(0)
    , tpr(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _time_stamp_type;
  _time_stamp_type time_stamp;

   typedef uint32_t _global_x_type;
  _global_x_type global_x;

   typedef uint32_t _global_y_type;
  _global_y_type global_y;

   typedef uint32_t _global_h_type;
  _global_h_type global_h;

   typedef uint16_t _zone_type;
  _zone_type zone;

   typedef int32_t _longitude_type;
  _longitude_type longitude;

   typedef int32_t _latitude_type;
  _latitude_type latitude;

   typedef uint16_t _heading_type;
  _heading_type heading;

   typedef int16_t _pitch_type;
  _pitch_type pitch;

   typedef int16_t _roll_type;
  _roll_type roll;

   typedef int16_t _global_vx_type;
  _global_vx_type global_vx;

   typedef int16_t _global_vy_type;
  _global_vy_type global_vy;

   typedef int16_t _global_vz_type;
  _global_vz_type global_vz;

   typedef int16_t _global_wx_type;
  _global_wx_type global_wx;

   typedef int16_t _global_wy_type;
  _global_wy_type global_wy;

   typedef int16_t _global_wz_type;
  _global_wz_type global_wz;

   typedef uint32_t _mileage_type;
  _mileage_type mileage;

   typedef uint8_t _belief_type;
  _belief_type belief;

   typedef float _baseline_type;
  _baseline_type baseline;

   typedef uint16_t _NSV1_type;
  _NSV1_type NSV1;

   typedef uint16_t _NSV2_type;
  _NSV2_type NSV2;

   typedef uint8_t _status_type;
  _status_type status;

   typedef int16_t _accx_type;
  _accx_type accx;

   typedef int16_t _accy_type;
  _accy_type accy;

   typedef int16_t _accz_type;
  _accz_type accz;

   typedef int16_t _tpr_type;
  _tpr_type tpr;





  typedef boost::shared_ptr< ::aloam_velodyne::dwdx_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aloam_velodyne::dwdx_<ContainerAllocator> const> ConstPtr;

}; // struct dwdx_

typedef ::aloam_velodyne::dwdx_<std::allocator<void> > dwdx;

typedef boost::shared_ptr< ::aloam_velodyne::dwdx > dwdxPtr;
typedef boost::shared_ptr< ::aloam_velodyne::dwdx const> dwdxConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aloam_velodyne::dwdx_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aloam_velodyne::dwdx_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace aloam_velodyne

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'aloam_velodyne': ['/home/hsc/catkin_ws_aloam/src/A-LOAM/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::aloam_velodyne::dwdx_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aloam_velodyne::dwdx_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aloam_velodyne::dwdx_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aloam_velodyne::dwdx_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aloam_velodyne::dwdx_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aloam_velodyne::dwdx_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aloam_velodyne::dwdx_<ContainerAllocator> >
{
  static const char* value()
  {
    return "884d668d25dbab1186123e045e144d45";
  }

  static const char* value(const ::aloam_velodyne::dwdx_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x884d668d25dbab11ULL;
  static const uint64_t static_value2 = 0x86123e045e144d45ULL;
};

template<class ContainerAllocator>
struct DataType< ::aloam_velodyne::dwdx_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aloam_velodyne/dwdx";
  }

  static const char* value(const ::aloam_velodyne::dwdx_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aloam_velodyne::dwdx_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Header needs to be the first field to refer to std_msgs/Header, other msgs here needs to add the package name such as nav_msgs/Odometry\n\
Header header\n\
float64 time_stamp\n\
uint32 global_x\n\
uint32 global_y\n\
uint32 global_h\n\
uint16 zone\n\
int32 longitude\n\
int32 latitude\n\
uint16 heading\n\
int16 pitch\n\
int16 roll\n\
int16 global_vx\n\
int16 global_vy\n\
int16 global_vz\n\
int16 global_wx\n\
int16 global_wy\n\
int16 global_wz\n\
uint32 mileage\n\
uint8 belief\n\
float32 baseline\n\
uint16 NSV1\n\
uint16 NSV2\n\
uint8 status\n\
int16 accx\n\
int16 accy\n\
int16 accz\n\
int16 tpr\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::aloam_velodyne::dwdx_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aloam_velodyne::dwdx_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time_stamp);
      stream.next(m.global_x);
      stream.next(m.global_y);
      stream.next(m.global_h);
      stream.next(m.zone);
      stream.next(m.longitude);
      stream.next(m.latitude);
      stream.next(m.heading);
      stream.next(m.pitch);
      stream.next(m.roll);
      stream.next(m.global_vx);
      stream.next(m.global_vy);
      stream.next(m.global_vz);
      stream.next(m.global_wx);
      stream.next(m.global_wy);
      stream.next(m.global_wz);
      stream.next(m.mileage);
      stream.next(m.belief);
      stream.next(m.baseline);
      stream.next(m.NSV1);
      stream.next(m.NSV2);
      stream.next(m.status);
      stream.next(m.accx);
      stream.next(m.accy);
      stream.next(m.accz);
      stream.next(m.tpr);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dwdx_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aloam_velodyne::dwdx_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aloam_velodyne::dwdx_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time_stamp: ";
    Printer<double>::stream(s, indent + "  ", v.time_stamp);
    s << indent << "global_x: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.global_x);
    s << indent << "global_y: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.global_y);
    s << indent << "global_h: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.global_h);
    s << indent << "zone: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.zone);
    s << indent << "longitude: ";
    Printer<int32_t>::stream(s, indent + "  ", v.longitude);
    s << indent << "latitude: ";
    Printer<int32_t>::stream(s, indent + "  ", v.latitude);
    s << indent << "heading: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.heading);
    s << indent << "pitch: ";
    Printer<int16_t>::stream(s, indent + "  ", v.pitch);
    s << indent << "roll: ";
    Printer<int16_t>::stream(s, indent + "  ", v.roll);
    s << indent << "global_vx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.global_vx);
    s << indent << "global_vy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.global_vy);
    s << indent << "global_vz: ";
    Printer<int16_t>::stream(s, indent + "  ", v.global_vz);
    s << indent << "global_wx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.global_wx);
    s << indent << "global_wy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.global_wy);
    s << indent << "global_wz: ";
    Printer<int16_t>::stream(s, indent + "  ", v.global_wz);
    s << indent << "mileage: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.mileage);
    s << indent << "belief: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.belief);
    s << indent << "baseline: ";
    Printer<float>::stream(s, indent + "  ", v.baseline);
    s << indent << "NSV1: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.NSV1);
    s << indent << "NSV2: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.NSV2);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
    s << indent << "accx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.accx);
    s << indent << "accy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.accy);
    s << indent << "accz: ";
    Printer<int16_t>::stream(s, indent + "  ", v.accz);
    s << indent << "tpr: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tpr);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ALOAM_VELODYNE_MESSAGE_DWDX_H
