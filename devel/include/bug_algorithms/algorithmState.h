// Generated by gencpp from file bug_algorithms/algorithmState.msg
// DO NOT EDIT!


#ifndef BUG_ALGORITHMS_MESSAGE_ALGORITHMSTATE_H
#define BUG_ALGORITHMS_MESSAGE_ALGORITHMSTATE_H


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
struct algorithmState_
{
  typedef algorithmState_<ContainerAllocator> Type;

  algorithmState_()
    : algorithm(0)
    , name()
    , pose_x(0.0)
    , pose_y(0.0)
    , yaw(0.0)
    , initial_to_goal_distance(0.0)
    , current_to_goal_distance(0.0)
    , best_distance(0.0)
    , path_length(0.0)
    , time(0.0)  {
    }
  algorithmState_(const ContainerAllocator& _alloc)
    : algorithm(0)
    , name(_alloc)
    , pose_x(0.0)
    , pose_y(0.0)
    , yaw(0.0)
    , initial_to_goal_distance(0.0)
    , current_to_goal_distance(0.0)
    , best_distance(0.0)
    , path_length(0.0)
    , time(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _algorithm_type;
  _algorithm_type algorithm;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef float _pose_x_type;
  _pose_x_type pose_x;

   typedef float _pose_y_type;
  _pose_y_type pose_y;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _initial_to_goal_distance_type;
  _initial_to_goal_distance_type initial_to_goal_distance;

   typedef float _current_to_goal_distance_type;
  _current_to_goal_distance_type current_to_goal_distance;

   typedef float _best_distance_type;
  _best_distance_type best_distance;

   typedef float _path_length_type;
  _path_length_type path_length;

   typedef float _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::bug_algorithms::algorithmState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bug_algorithms::algorithmState_<ContainerAllocator> const> ConstPtr;

}; // struct algorithmState_

typedef ::bug_algorithms::algorithmState_<std::allocator<void> > algorithmState;

typedef boost::shared_ptr< ::bug_algorithms::algorithmState > algorithmStatePtr;
typedef boost::shared_ptr< ::bug_algorithms::algorithmState const> algorithmStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bug_algorithms::algorithmState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bug_algorithms::algorithmState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bug_algorithms

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'bug_algorithms': ['/home/mario/catkin_ws/src/bug_algorithms/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bug_algorithms::algorithmState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bug_algorithms::algorithmState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bug_algorithms::algorithmState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bug_algorithms::algorithmState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bug_algorithms::algorithmState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bug_algorithms::algorithmState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bug_algorithms::algorithmState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f90eba9625cb29111498ea4fb81eca4";
  }

  static const char* value(const ::bug_algorithms::algorithmState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f90eba9625cb291ULL;
  static const uint64_t static_value2 = 0x11498ea4fb81eca4ULL;
};

template<class ContainerAllocator>
struct DataType< ::bug_algorithms::algorithmState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bug_algorithms/algorithmState";
  }

  static const char* value(const ::bug_algorithms::algorithmState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bug_algorithms::algorithmState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 algorithm\n\
string name\n\
float32 pose_x\n\
float32 pose_y\n\
float32 yaw\n\
float32 initial_to_goal_distance\n\
float32 current_to_goal_distance\n\
float32 best_distance\n\
float32 path_length\n\
float32 time\n\
";
  }

  static const char* value(const ::bug_algorithms::algorithmState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bug_algorithms::algorithmState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.algorithm);
      stream.next(m.name);
      stream.next(m.pose_x);
      stream.next(m.pose_y);
      stream.next(m.yaw);
      stream.next(m.initial_to_goal_distance);
      stream.next(m.current_to_goal_distance);
      stream.next(m.best_distance);
      stream.next(m.path_length);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct algorithmState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bug_algorithms::algorithmState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bug_algorithms::algorithmState_<ContainerAllocator>& v)
  {
    s << indent << "algorithm: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.algorithm);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "pose_x: ";
    Printer<float>::stream(s, indent + "  ", v.pose_x);
    s << indent << "pose_y: ";
    Printer<float>::stream(s, indent + "  ", v.pose_y);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "initial_to_goal_distance: ";
    Printer<float>::stream(s, indent + "  ", v.initial_to_goal_distance);
    s << indent << "current_to_goal_distance: ";
    Printer<float>::stream(s, indent + "  ", v.current_to_goal_distance);
    s << indent << "best_distance: ";
    Printer<float>::stream(s, indent + "  ", v.best_distance);
    s << indent << "path_length: ";
    Printer<float>::stream(s, indent + "  ", v.path_length);
    s << indent << "time: ";
    Printer<float>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BUG_ALGORITHMS_MESSAGE_ALGORITHMSTATE_H
