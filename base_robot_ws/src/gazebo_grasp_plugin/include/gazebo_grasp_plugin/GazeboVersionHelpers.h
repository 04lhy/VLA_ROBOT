#ifndef GAZEBO_VERSION_HELPERS_H
#define GAZEBO_VERSION_HELPERS_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo {

// Type aliases for Gazebo 11+
typedef ignition::math::Pose3d GzPose3;
typedef ignition::math::Vector3d GzVector3;
typedef ignition::math::Quaterniond GzQuaternion;
typedef ignition::math::Matrix4d GzMatrix4;

inline GzPose3 GetWorldPose(const physics::LinkPtr &link) {
  return link->WorldPose();
}

inline GzVector3 GetWorldVelocity(const physics::LinkPtr &link) {
  return link->WorldLinearVel();
}

inline GzMatrix4 GetMatrix(const GzPose3 &pose) {
  return GzMatrix4(pose);
}

inline GzMatrix4 GetMatrix(const GzVector3 &pos) {
  GzMatrix4 mat = GzMatrix4::Identity;
  mat.SetTranslation(pos);
  return mat;
}

inline double GetLength(const GzVector3 &v) {
  return v.Length();
}

inline GzVector3 GetPos(const GzPose3 &pose) {
  return pose.Pos();
}

inline GzVector3 GetPos(const GzMatrix4 &mat) {
  return mat.Translation();
}

inline physics::PhysicsEnginePtr GetPhysics(const physics::WorldPtr &world) {
  return world->Physics();
}

inline physics::EntityPtr GetEntityByName(
    const physics::WorldPtr &world, const std::string &name) {
  return world->EntityByName(name);
}

template<typename T>
inline std::string GetName(const T& t) {
  return t.Name();
}

} // namespace gazebo
#endif
