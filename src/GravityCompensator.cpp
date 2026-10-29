/*
 * GravityCompensator.cpp -- ver GravityCompensator.h.
 */
#include "GravityCompensator.h"

#include <Eigen/Geometry>

namespace control {
namespace stabilization {

int RigidBodyModel::linkIndex(const std::string &name) const {
    for (size_t i = 0; i < links.size(); ++i)
        if (links[i].name == name)
            return static_cast<int>(i);
    return -1;
}

int RigidBodyModel::jointIndex(const std::string &name) const {
    for (size_t i = 0; i < joints.size(); ++i)
        if (joints[i].name == name)
            return static_cast<int>(i);
    return -1;
}

GravityCompensator::GravityCompensator(double gravity)
    : g_(0.0, 0.0, -gravity), totalMass_(0.0) {
}

bool GravityCompensator::setModel(const RigidBodyModel &model) {
    model_ = model;
    if (model_.root < 0 || model_.root >= static_cast<int>(model_.links.size()))
        return false;
    totalMass_ = 0.0;
    for (const auto &l : model_.links)
        totalMass_ += l.mass;
    subtreeOfJoint_.assign(model_.joints.size(), {});
    for (size_t j = 0; j < model_.joints.size(); ++j) {
        const int child = model_.joints[j].childLink;
        if (child < 0 || child >= static_cast<int>(model_.links.size()))
            return false;
        collectSubtree(child, subtreeOfJoint_[j]);
    }
    return totalMass_ > 0.0;
}

void GravityCompensator::collectSubtree(int link, std::vector<int> &out) const {
    out.push_back(link);
    for (int j : model_.links[link].childJoints)
        collectSubtree(model_.joints[j].childLink, out);
}

std::vector<int> GravityCompensator::chain(int footLink) const {
    std::vector<int> c;
    int l = footLink;
    while (l >= 0 && l != model_.root) {
        const int j = model_.links[l].parentJoint;
        if (j < 0)
            return {};
        c.push_back(j);
        l = model_.joints[j].parentLink;
    }
    return std::vector<int>(c.rbegin(), c.rend());
}

void GravityCompensator::forwardKinematics(const std::vector<double> &q,
                                           std::vector<Eigen::Vector3d> &linkPos,
                                           std::vector<Eigen::Matrix3d> &linkRot,
                                           std::vector<Eigen::Vector3d> &jointPos,
                                           std::vector<Eigen::Vector3d> &jointAxis) const {
    const size_t nl = model_.links.size(), nj = model_.joints.size();
    linkPos.assign(nl, Eigen::Vector3d::Zero());
    linkRot.assign(nl, Eigen::Matrix3d::Identity());
    jointPos.assign(nj, Eigen::Vector3d::Zero());
    jointAxis.assign(nj, Eigen::Vector3d::UnitX());
    std::vector<int> stack{model_.root};
    while (!stack.empty()) {
        const int l = stack.back();
        stack.pop_back();
        for (int j : model_.links[l].childJoints) {
            const auto &jt = model_.joints[j];
            const Eigen::Matrix3d R = linkRot[l] * jt.originRot;
            const Eigen::Vector3d p = linkPos[l] + linkRot[l] * jt.originXyz;
            jointPos[j] = p;
            jointAxis[j] = R * jt.axis;
            const double angle = (jt.movable && j < static_cast<int>(q.size())) ? q[j] : 0.0;
            linkRot[jt.childLink] = R * Eigen::AngleAxisd(angle, jt.axis).toRotationMatrix();
            linkPos[jt.childLink] = p;
            stack.push_back(jt.childLink);
        }
    }
}

std::map<int, double> GravityCompensator::legTorques(const std::vector<double> &q,
                                                     int footLink, bool support) const {
    std::vector<Eigen::Vector3d> lp, jp, ja;
    std::vector<Eigen::Matrix3d> lr;
    forwardKinematics(q, lp, lr, jp, ja);

    // Primeiros momentos de massa: S = sum m_i r_i
    std::vector<Eigen::Vector3d> r(model_.links.size());
    Eigen::Vector3d sAll = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < model_.links.size(); ++i) {
        r[i] = lp[i] + lr[i] * model_.links[i].com;
        sAll += model_.links[i].mass * r[i];
    }

    std::map<int, double> out;
    for (int j : chain(footLink)) {
        double mSub = 0.0;
        Eigen::Vector3d sSub = Eigen::Vector3d::Zero();
        for (int l : subtreeOfJoint_[j]) {
            mSub += model_.links[l].mass;
            sSub += model_.links[l].mass * r[l];
        }
        const Eigen::Vector3d &p = jp[j];
        double tau;
        if (support) {
            // Momento da gravidade sobre o lado do tronco: (S_B - M_B p) x g
            const Eigen::Vector3d m = (sAll - sSub - (totalMass_ - mSub) * p).cross(g_);
            tau = m.dot(ja[j]);
        } else {
            const Eigen::Vector3d m = (sSub - mSub * p).cross(g_);
            tau = -m.dot(ja[j]);
        }
        out[j] = tau;
    }
    return out;
}

}  // namespace stabilization
}  // namespace control
