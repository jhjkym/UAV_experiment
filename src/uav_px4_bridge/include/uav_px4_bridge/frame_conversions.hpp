#ifndef UAV_PX4_BRIDGE_FRAME_CONVERSIONS_HPP_
#define UAV_PX4_BRIDGE_FRAME_CONVERSIONS_HPP_

#include <geometry_msgs/Vector3.h>

namespace uav_px4_bridge {

// Convert a vector from ENU world coordinates, in meters or meters per second,
// to NED world coordinates with the same units.
geometry_msgs::Vector3 enuToNed(const geometry_msgs::Vector3& enu);

// Convert a vector from NED world coordinates, in meters or meters per second,
// to ENU world coordinates with the same units.
geometry_msgs::Vector3 nedToEnu(const geometry_msgs::Vector3& ned);

// Convert a vector from FLU body coordinates to FRD body coordinates with the
// same units.
geometry_msgs::Vector3 fluToFrd(const geometry_msgs::Vector3& flu);

// Convert a vector from FRD body coordinates to FLU body coordinates with the
// same units.
geometry_msgs::Vector3 frdToFlu(const geometry_msgs::Vector3& frd);

}  // namespace uav_px4_bridge

#endif  // UAV_PX4_BRIDGE_FRAME_CONVERSIONS_HPP_
