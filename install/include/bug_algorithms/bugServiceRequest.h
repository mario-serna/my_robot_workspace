// Generated by gencpp from file bug_algorithms/bugServiceRequest.msg
// DO NOT EDIT!


#ifndef BUG_ALGORITHMS_MESSAGE_BUGSERVICEREQUEST_H
#define BUG_ALGORITHMS_MESSAGE_BUGSERVICEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bug_algorithms
{
template <class ContainerAllocator>
struct bugServiceRequest_
{
  typedef bugServiceRequest_<ContainerAllocator> Type;

  bugServiceRequest_()
    : algorithm(0)
    , velocity(0.0)
    , initial_x(0.0)
    , initial_y(0.0)
    , desired_x(0.0)
    , desired_y(0.0)
    , simulation(false)
    , reverse(false)
    , choose(false)  {
    }
  bugServiceRequest_(const ContainerAllocator& _alloc)
    : algorithm(0)
    , velocity(0.0)
    , initial_x(0.0)
    , initial_y(0.0)
    , desired_x(0.0)
    , desired_y(0.0)
    , simulation(false)
    , reverse(false)
    , choose(false)  {
  (void)_alloc;
    }



   typedef int32_t _algorithm_type;
  _algorithm_type algorithm;

   typedef float _velocity_type;
  _velocity_type velocity;

   typedef float _initial_x_type;
  _initial_x_type initial_x;

   typedef float _initial_y_type;
  _initial_y_type initial_y;

   typedef float _desired_x_type;
  _desired_x_type desired_x;

   typedef float _desired_y_type;
  _desired_y_type desired_y;

   typedef uint8_t _simulation_type;
  _simulation_type simulation;

   typedef uint8_t _reverse_type;
  _reverse_type reverse;

   typedef uint8_t _choose_type;
  _choose_type choose;





  typedef boost::shared_ptr< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct bugServiceRequest_

typedef ::bug_algorithms::bugServiceRequest_<std::allocator<void> > bugServiceRequest;

typedef boost::shared_ptr< ::bug_algorithms::bugServiceRequest > bugServiceRequestPtr;
typedef boost::shared_ptr< ::bug_algorithms::bugServiceRequest const> bugServiceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bug_algorithms::bugServiceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bug_algorithms

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'bug_algorithms': ['/home/mario/catkin_ws/src/bug_algorithms/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "310f018babea00a829c4f64be9e6a75a";
  }

  static const char* value(const ::bug_algorithms::bugServiceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x310f018babea00a8ULL;
  static const uint64_t static_value2 = 0x29c4f64be9e6a75aULL;
};

template<class ContainerAllocator>
struct DataType< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bug_algorithms/bugServiceRequest";
  }

  static const char* value(const ::bug_algorithms::bugServiceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 algorithm\n\
float32 velocity\n\
float32 initial_x\n\
float32 initial_y\n\
float32 desired_x\n\
float32 desired_y\n\
bool simulation\n\
bool reverse\n\
bool choose\n\
";
  }

  static const char* value(const ::bug_algorithms::bugServiceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.algorithm);
      stream.next(m.velocity);
      stream.next(m.initial_x);
      stream.next(m.initial_y);
      stream.next(m.desired_x);
      stream.next(m.desired_y);
      stream.next(m.simulation);
      stream.next(m.reverse);
      stream.next(m.choose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct bugServiceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bug_algorithms::bugServiceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bug_algorithms::bugServiceRequest_<ContainerAllocator>& v)
  {
    s << indent << "algorithm: ";
    Printer<int32_t>::stream(s, indent + "  ", v.algorithm);
    s << indent << "velocity: ";
    Printer<float>::stream(s, indent + "  ", v.velocity);
    s << indent << "initial_x: ";
    Printer<float>::stream(s, indent + "  ", v.initial_x);
    s << indent << "initial_y: ";
    Printer<float>::stream(s, indent + "  ", v.initial_y);
    s << indent << "desired_x: ";
    Printer<float>::stream(s, indent + "  ", v.desired_x);
    s << indent << "desired_y: ";
    Printer<float>::stream(s, indent + "  ", v.desired_y);
    s << indent << "simulation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.simulation);
    s << indent << "reverse: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reverse);
    s << indent << "choose: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.choose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BUG_ALGORITHMS_MESSAGE_BUGSERVICEREQUEST_H
