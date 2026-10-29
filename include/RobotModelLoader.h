/*
 * RobotModelLoader.h
 *
 * Monta o RigidBodyModel do GravityCompensator a partir de um urdf::Model
 * (o parametro robot_description). Separado do compensador para que este
 * fique sem dependencia de ROS e testavel offline.
 */
#ifndef ROBOT_MODEL_LOADER_H_
#define ROBOT_MODEL_LOADER_H_

#include <urdf/model.h>

#include <Eigen/Geometry>
#include <string>

#include "GravityCompensator.h"

namespace control {
namespace stabilization {

inline bool buildRigidBodyModel(const urdf::Model &urdf, RigidBodyModel &out,
                                std::string &error) {
    out = RigidBodyModel();
    if (!urdf.getRoot()) {
        error = "URDF sem link raiz";
        return false;
    }
    for (const auto &kv : urdf.links_) {
        RigidBodyModel::Link l;
        l.name = kv.first;
        if (kv.second->inertial) {
            l.mass = kv.second->inertial->mass;
            const auto &p = kv.second->inertial->origin.position;
            l.com = Eigen::Vector3d(p.x, p.y, p.z);
        }
        out.links.push_back(l);
    }
    for (const auto &kv : urdf.joints_) {
        const auto &uj = *kv.second;
        RigidBodyModel::Joint j;
        j.name = uj.name;
        j.parentLink = out.linkIndex(uj.parent_link_name);
        j.childLink = out.linkIndex(uj.child_link_name);
        if (j.parentLink < 0 || j.childLink < 0) {
            error = "junta " + uj.name + " com link desconhecido";
            return false;
        }
        const auto &p = uj.parent_to_joint_origin_transform.position;
        const auto &r = uj.parent_to_joint_origin_transform.rotation;
        j.originXyz = Eigen::Vector3d(p.x, p.y, p.z);
        j.originRot = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
        Eigen::Vector3d axis(uj.axis.x, uj.axis.y, uj.axis.z);
        j.movable = (uj.type == urdf::Joint::REVOLUTE || uj.type == urdf::Joint::CONTINUOUS);
        j.axis = axis.norm() > 0.0 ? axis.normalized() : Eigen::Vector3d::UnitX();
        out.joints.push_back(j);
    }
    for (size_t j = 0; j < out.joints.size(); ++j) {
        out.links[out.joints[j].childLink].parentJoint = static_cast<int>(j);
        out.links[out.joints[j].parentLink].childJoints.push_back(static_cast<int>(j));
    }
    out.root = out.linkIndex(urdf.getRoot()->name);
    return out.root >= 0;
}

}  // namespace stabilization
}  // namespace control

#endif  // ROBOT_MODEL_LOADER_H_
