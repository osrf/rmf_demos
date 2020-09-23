#ifndef SRC__RMF_PLUGINS__UTILS_HPP
#define SRC__RMF_PLUGINS__UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <Eigen/Geometry>

namespace rmf_plugins_utils {

/////////////////////////////////////////////////////////////////////////////////////////////
enum Simulator {Ignition, Gazebo};

// Holds an identifier referring to either an Ignition or Gazebo classic entity
// Contains either a uint64_t value or a std::string value depending on `sim_type`
// Enables functions to be written that generically operate on both Ignition or Gazebo entities
struct SimEntity
{
  Simulator sim_type;
  uint64_t entity; // If used for Ignition Gazebo
  std::string name; // If used for Gazebo classic

  SimEntity(uint64_t en)
  : sim_type(Ignition), entity(en)
  {
    name = "";
  }
  SimEntity(std::string nm)
  : sim_type(Gazebo), name(nm)
  {
    entity = 0;
  }

  const std::string& get_name() const
  {
    if (sim_type != Gazebo)
    {
      std::cerr << "SimEntity Ignition object does not hold a name."
                << std::endl;
    }
    return name;
  }

  uint64_t get_entity() const
  {
    if (sim_type != Ignition)
    {
      std::cerr << "SimEntity Gazebo object does not hold a uint64_t entity."
                << std::endl;
    }
    return entity;
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////////
// TODO(MXG): Refactor the use of this function to replace it with
// compute_desired_rate_of_change()
double compute_ds(
  double s_target,
  double v_actual,
  double v_max,
  double accel_nom,
  double accel_max,
  double dt);

struct MotionParams
{
  double v_max = 0.2;
  double a_max = 0.1;
  double a_nom = 0.08;
  double dx_min = 0.01;
  double f_max = 10000000.0;
};

double compute_desired_rate_of_change(
  double _s_target,
  double _v_actual,
  const MotionParams& _motion_params,
  const double _dt);

rclcpp::Time simulation_now(double t);

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename SdfPtrT, typename SdfElementPtrT>
bool get_element_required(
  SdfPtrT& _sdf,
  const std::string& _element_name,
  SdfElementPtrT& _element)
{
  if (!_sdf->HasElement(_element_name))
  {
    std::cerr << "Element [" << _element_name << "] not found" << std::endl;
    return false;
  }
  // using GetElementImpl() because for sdf::v9 GetElement() is not const
  _element = _sdf->GetElementImpl(_element_name);
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename SdfPtrT>
bool get_sdf_attribute_required(SdfPtrT& sdf, const std::string& attribute_name,
  T& value)
{
  if (sdf->HasAttribute(attribute_name))
  {
    if (sdf->GetAttribute(attribute_name)->Get(value))
    {
      std::cout << "Using specified attribute value [" << value
                << "] for property [" << attribute_name << "]"
                << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf attribute for [" << attribute_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cerr << "Attribute [" << attribute_name << "] not found" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename SdfPtrT>
bool get_sdf_param_required(SdfPtrT& sdf, const std::string& parameter_name,
  T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name << "]"
                <<std::endl;
    }
  }
  else
  {
    std::cerr << "Property [" << parameter_name << "] not found" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename SdfPtrT>
void get_sdf_param_if_available(SdfPtrT& sdf, const std::string& parameter_name,
  T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cout << "Using default value [" << value << "] for property ["
              << parameter_name << "]" << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ResultMsgT>
std::shared_ptr<ResultMsgT> make_response(uint8_t status,
  const double sim_time,
  const std::string& request_guid,
  const std::string& guid)
{
  std::shared_ptr<ResultMsgT> response = std::make_shared<ResultMsgT>();
  response->time = simulation_now(sim_time);
  response->request_guid = request_guid;
  response->source_guid = guid;
  response->status = status;
  return response;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Version agnostic conversion functions between Ignition Math and Eigen. Removes need for Ignition
// Math dependencies in rmf_plugins_common

template<typename IgnQuatT>
inline void convert(const Eigen::Quaterniond& _q, IgnQuatT& quat)
{
  quat.W() = _q.w();
  quat.X() = _q.x();
  quat.Y() = _q.y();
  quat.Z() = _q.z();
}

template<typename IgnVec3T>
inline void convert(const Eigen::Vector3d& _v, IgnVec3T& vec)
{
  vec.X() = _v[0];
  vec.Y() = _v[1];
  vec.Z() = _v[2];
}

template<typename IgnVec3T>
inline Eigen::Vector3d convert_vec(const IgnVec3T& _v)
{
  return Eigen::Vector3d(_v[0], _v[1], _v[2]);
}

template<typename IgnQuatT>
inline Eigen::Quaterniond convert_quat(const IgnQuatT& _q)
{
  Eigen::Quaterniond quat;
  quat.w() = _q.W();
  quat.x() = _q.X();
  quat.y() = _q.Y();
  quat.z() = _q.Z();

  return quat;
}

template<typename IgnPoseT>
inline auto convert_to_pose(const Eigen::Isometry3d& _tf)
{
  IgnPoseT pose;
  convert(Eigen::Vector3d(_tf.translation()), pose.Pos());
  convert(Eigen::Quaterniond(_tf.linear()), pose.Rot());

  return pose;
}

template<typename IgnPoseT>
inline Eigen::Isometry3d convert_pose(const IgnPoseT& _pose)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = convert_vec(_pose.Pos());
  tf.linear() = Eigen::Matrix3d(convert_quat(_pose.Rot()));

  return tf;
}

} // namespace rmf_plugins_utils

#endif // SRC__RMF_PLUGINS__UTILS_HPP
