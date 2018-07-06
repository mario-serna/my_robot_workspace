// Generated by gencpp from file bug_algorithms/bugSwitchRequest.msg
// DO NOT EDIT!


#ifndef BUG_ALGORITHMS_MESSAGE_BUGSWITCHREQUEST_H
#define BUG_ALGORITHMS_MESSAGE_BUGSWITCHREQUEST_H


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
struct bugSwitchRequest_
{
  typedef bugSwitchRequest_<ContainerAllocator> Type;

  bugSwitchRequest_()
    : algorithm(0)
    , state(0)  {
    }
  bugSwitchRequest_(const ContainerAllocator& _alloc)
    : algorithm(0)
    , state(0)  {
  (void)_alloc;
    }



   typedef int32_t _algorithm_type;
  _algorithm_type algorithm;

   typedef int32_t _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> const> ConstPtr;

}; // struct bugSwitchRequest_

typedef ::bug_algorithms::bugSwitchRequest_<std::allocator<void> > bugSwitchRequest;

typedef boost::shared_ptr< ::bug_algorithms::bugSwitchRequest > bugSwitchRequestPtr;
typedef boost::shared_ptr< ::bug_algorithms::bugSwitchRequest const> bugSwitchRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "412bcf4d0060468078ad60a3377aac77";
  }

  static const char* value(const ::bug_algorithms::bugSwitchRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x412bcf4d00604680ULL;
  static const uint64_t static_value2 = 0x78ad60a3377aac77ULL;
};

template<class ContainerAllocator>
struct DataType< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bug_algorithms/bugSwitchRequest";
  }

  static const char* value(const ::bug_algorithms::bugSwitchRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 algorithm\n\
int32 state\n\
";
  }

  static const char* value(const ::bug_algorithms::bugSwitchRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.algorithm);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct bugSwitchRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bug_algorithms::bugSwitchRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bug_algorithms::bugSwitchRequest_<ContainerAllocator>& v)
  {
    s << indent << "algorithm: ";
    Printer<int32_t>::stream(s, indent + "  ", v.algorithm);
    s << indent << "state: ";
    Printer<int32_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BUG_ALGORITHMS_MESSAGE_BUGSWITCHREQUEST_H
