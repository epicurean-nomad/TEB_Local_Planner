#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


// ----------------------------------------------------------------------
// g2o vertex/edge types
// ----------------------------------------------------------------------
// Poses are g2o::VertexSE2 directly (proper SE2 manifold: oplusImpl wraps
// theta correctly). Only the scalar time-difference vertex is custom.
// None of these edges override linearizeOplus() -- computeError() alone is
// enough; g2o's BaseFixedSizedEdge/BaseVariableSizedEdge default
// linearizeOplus() falls back to central-difference numeric
// differentiation (delta = 1e-9) automatically. Verified against the
// installed g2o (0~20230806) by compiling and running a standalone test
// exercising every edge shape used here (unary/binary/5-vertex multi-edge,
// mixed VertexSE2 + scalar vertex types, BlockSolverX + LinearSolverEigen +
// OptimizationAlgorithmLevenberg) before writing this into the real header.
 
class VertexTimeDiff : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() override { _estimate = 0.3; }
    virtual void oplusImpl(const double* update) override { _estimate += update[0]; }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 
// Unary: obstacle clearance. Raw hinge residual max(0, minClearance - dist);
// g2o squares it via chi2 = e^T * Omega * e, so it's a proper squared-hinge
// penalty without double-squaring it here. Obstacle data is plain edge
// state (set right after construction), not the g2o "measurement" -- there's
// nothing here that needs (de)serialization.
class EdgeObstacle : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double obstacleX = 0.0, obstacleY = 0.0, minClearance = 1.0;
    void computeError() override {
        const auto* v = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        auto p = v->estimate().translation();
        double dx = p.x() - obstacleX, dy = p.y() - obstacleY;
        double d = std::sqrt(dx * dx + dy * dy);
        _error[0] = std::max(0.0, minClearance - d);
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 


// Unary: pull toward the nearest point on the *original* (pre-optimization)
// reference band -- a small "stay close to plan" bias. 2D residual (dx, dy).
class EdgeViaPoint : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeError() override {
        const auto* v = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        Eigen::Vector2d p = v->estimate().translation();
        _error = p - _measurement;
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 


// Binary: non-holonomic "no sideways slip" residual between consecutive
// poses -- a near-equality soft constraint (large information weight),
// not an inequality hinge like the others.
class EdgeKinematics : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeError() override {
        const auto* va = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        const auto* vb = static_cast<const g2o::VertexSE2*>(_vertices[1]);
        Eigen::Vector2d d = vb->estimate().translation() - va->estimate().translation();
        double thetaA = va->estimate().rotation().angle();
        double thetaB = vb->estimate().rotation().angle();
        double dtheta = g2o::normalize_theta(thetaB - thetaA);
        double avgTheta = thetaA + 0.5 * dtheta;
        double hx = std::cos(avgTheta), hy = std::sin(avgTheta);
        _error[0] = -d.x() * hy + d.y() * hx; // lateral (sideways) component
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 
// Binary: minimum-turning-radius (steering) limit between consecutive poses.
class EdgeCurvature : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double kappaMax = 0.33;
    void computeError() override {
        const auto* va = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        const auto* vb = static_cast<const g2o::VertexSE2*>(_vertices[1]);
        Eigen::Vector2d d = vb->estimate().translation() - va->estimate().translation();
        double dist = d.norm();
        double thetaA = va->estimate().rotation().angle();
        double thetaB = vb->estimate().rotation().angle();
        double dtheta = g2o::normalize_theta(thetaB - thetaA);
        double kappa = dtheta / std::max(dist, 1e-3);
        _error[0] = std::max(0.0, std::fabs(kappa) - kappaMax);
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 
// Multi-edge (3 vertices: poseA, poseB, dt) -- translational + angular
// velocity limits, with the lateral-acceleration-aware dynamic speed cap
// mirroring computeVelocityProfile()'s own v_curv = sqrt(MAX_LAT_ACC/kappa)
// rule, and a gear-aware (not sign-inferred) vmax since the whole band is
// guaranteed single-gear by findHorizonEnd().
class EdgeVelocity : public g2o::BaseMultiEdge<2, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double vmax = 4.0, omegaMax = 1.0, maxLatAcc = 2.0;
    EdgeVelocity() { resize(3); }
    void computeError() override {
        const auto* va  = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        const auto* vb  = static_cast<const g2o::VertexSE2*>(_vertices[1]);
        const auto* vdt = static_cast<const VertexTimeDiff*>(_vertices[2]);
        double dt = std::max(vdt->estimate(), 1e-6);
 
        Eigen::Vector2d d = vb->estimate().translation() - va->estimate().translation();
        double dist = d.norm();
        double thetaA = va->estimate().rotation().angle();
        double thetaB = vb->estimate().rotation().angle();
        double dtheta = g2o::normalize_theta(thetaB - thetaA);
        double avgTheta = thetaA + 0.5 * dtheta;
        double hx = std::cos(avgTheta), hy = std::sin(avgTheta);
        double signed_dist = d.x() * hx + d.y() * hy;
 
        double v     = signed_dist / dt;
        double omega = dtheta / dt;
 
        double vcap = vmax;
        double kappa_for_v = std::fabs(dtheta) / std::max(dist, 1e-3);
        if (kappa_for_v > 1e-6) vcap = std::min(vcap, std::sqrt(maxLatAcc / kappa_for_v));
 
        _error[0] = std::max(0.0, std::fabs(v) - vcap);
        _error[1] = std::max(0.0, std::fabs(omega) - omegaMax);
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 


// Multi-edge (5 vertices: poseA, poseB, poseC, dtAB, dtBC) -- asymmetric
// accel/decel limit across the pair of edges meeting at poseB.
class EdgeAcceleration : public g2o::BaseMultiEdge<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double maxAccel = 3.0, maxDecel = 2.0;
    EdgeAcceleration() { resize(5); }
    static double signedV(const g2o::VertexSE2* X, const g2o::VertexSE2* Y, double dt) {
        Eigen::Vector2d d = Y->estimate().translation() - X->estimate().translation();
        double thetaX = X->estimate().rotation().angle();
        double thetaY = Y->estimate().rotation().angle();
        double dtheta = g2o::normalize_theta(thetaY - thetaX);
        double avgTheta = thetaX + 0.5 * dtheta;
        double hx = std::cos(avgTheta), hy = std::sin(avgTheta);
        return (d.x() * hx + d.y() * hy) / std::max(dt, 1e-6);
    }
    void computeError() override {
        const auto* vA   = static_cast<const g2o::VertexSE2*>(_vertices[0]);
        const auto* vB   = static_cast<const g2o::VertexSE2*>(_vertices[1]);
        const auto* vC   = static_cast<const g2o::VertexSE2*>(_vertices[2]);
        const auto* vDt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
        const auto* vDt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
 
        double dt1 = std::max(vDt1->estimate(), 1e-6);
        double dt2 = std::max(vDt2->estimate(), 1e-6);
        double v1 = signedV(vA, vB, dt1);
        double v2 = signedV(vB, vC, dt2);
        double dtAvg = std::max(0.5 * (dt1 + dt2), 1e-6);
        double a = (v2 - v1) / dtAvg;
 
        double limit = (a >= 0.0) ? maxAccel : maxDecel;
        _error[0] = std::max(0.0, std::fabs(a) - limit);
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
 


// Unary: mild time-optimality pressure on each dt vertex.
class EdgeTimeOptimal : public g2o::BaseUnaryEdge<1, double, VertexTimeDiff> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeError() override {
        const auto* v = static_cast<const VertexTimeDiff*>(_vertices[0]);
        _error[0] = v->estimate();
    }
    virtual bool read(std::istream&) override { return true; }
    virtual bool write(std::ostream&) const override { return true; }
};
