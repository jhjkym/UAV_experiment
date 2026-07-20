#include "uav_px4_bridge/frame_conversions.hpp"

namespace uav_px4_bridge {

geometry_msgs::Vector3 enuToNed(const geometry_msgs::Vector3& enu) {
  geometry_msgs::Vector3 ned;
  ned.x = enu.y;
  ned.y = enu.x;
  ned.z = -enu.z;
  return ned;
}

geometry_msgs::Vector3 nedToEnu(const geometry_msgs::Vector3& ned) {
  geometry_msgs::Vector3 enu;
  enu.x = ned.y;
  enu.y = ned.x;
  enu.z = -ned.z;
  return enu;
}

geometry_msgs::Vector3 fluToFrd(const geometry_msgs::Vector3& flu) {
  geometry_msgs::Vector3 frd;
  frd.x = flu.x;
  frd.y = -flu.y;
  frd.z = -flu.z;
  return frd;
}

geometry_msgs::Vector3 frdToFlu(const geometry_msgs::Vector3& frd) {
  geometry_msgs::Vector3 flu;
  flu.x = frd.x;
  flu.y = -frd.y;
  flu.z = -frd.z;
  return flu;
}

}  // namespace uav_px4_bridge
