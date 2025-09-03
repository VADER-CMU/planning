
#include <iostream>

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
/**
 * @brief Computes the forward kinematics for the XArm robot.
 * 
 * Calculates the end-effector transformation matrix given the joint positions.
 * 
 * @param joint_positions Array of joint angles (in radians) for each of the 7 joints.
 * @return Matrix4d Homogeneous transformation matrix representing the end-effector pose.
 */
class XArmForwardKinematics {
public:
    using Matrix4d = Eigen::Matrix4d;
    static constexpr size_t N_JOINTS = 7;

    Matrix4d forward_kinematics(const std::array<double, N_JOINTS>& joint_positions) const {
        Matrix4d T = Matrix4d::Identity();
        for (size_t i = 0; i < N_JOINTS; ++i) {
            T = T * joint_transform(i, joint_positions[i]);
        }
        return T;
    }

private:
    struct DHParams {
        double d, a, alpha;
    };

    static constexpr std::array<DHParams, N_JOINTS> dh = {{
        { 0.267, 0.0, -M_PI/2 },
        { 0.000, 0.0,  M_PI/2 },
        { 0.293, 0.0525, M_PI/2 },
        { 0.000, 0.0775, M_PI/2 },
        { 0.3425, 0.0, M_PI/2 },
        { 0.000, 0.076, -M_PI/2 },
        { 0.097, 0.0, 0.0 },
    }};

    static constexpr std::array<double, N_JOINTS> cos_alpha = []{
        std::array<double, N_JOINTS> c = {};
        for (size_t i = 0; i < N_JOINTS; ++i) c[i] = std::cos(dh[i].alpha);
        return c;
    }();

    static constexpr std::array<double, N_JOINTS> sin_alpha = []{
        std::array<double, N_JOINTS> s = {};
        for (size_t i = 0; i < N_JOINTS; ++i) s[i] = std::sin(dh[i].alpha);
        return s;
    }();

    Matrix4d joint_transform(size_t i, double theta) const {
        double ct = std::cos(theta);
        double st = std::sin(theta);

        Matrix4d T;
        T << ct, -st*cos_alpha[i],  st*sin_alpha[i], dh[i].a*ct,
             st,  ct*cos_alpha[i], -ct*sin_alpha[i], dh[i].a*st,
              0,       sin_alpha[i],      cos_alpha[i],    dh[i].d,
              0,           0,                 0,           1;
        return T;
    }
};

class XarmTaskSpaceOptimizationObjective : public ompl::base::OptimizationObjective {
    public:
        XarmTaskSpaceOptimizationObjective(const ompl::base::SpaceInformationPtr& si)
            : ompl::base::OptimizationObjective(si), xarm_fk() {}

        
        ompl::base::Cost stateCost(const ompl::base::State* s) const override {
            // Example: zero cost for all states (customize as needed)
            return ompl::base::Cost(0.0);
        }

        ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override {
            // Compute FK for both states and use L2 norm of end-effector positions as cost
            const auto* r1 = s1->as<ompl::base::RealVectorStateSpace::StateType>();
            const auto* r2 = s2->as<ompl::base::RealVectorStateSpace::StateType>();

            std::array<double, XArmForwardKinematics::N_JOINTS> joints1, joints2;
            for (size_t i = 0; i < XArmForwardKinematics::N_JOINTS; ++i) {
                joints1[i] = r1->values[i];
                joints2[i] = r2->values[i];
            }

            auto T1 = xarm_fk.forward_kinematics(joints1);
            auto T2 = xarm_fk.forward_kinematics(joints2);

            Eigen::Vector3d p1 = T1.block<3,1>(0,3);
            Eigen::Vector3d p2 = T2.block<3,1>(0,3);

            double cost = (p1 - p2).norm();
            return ompl::base::Cost(cost);
            // return identityCost();
        }

        ompl::base::Cost motionCostHeuristic(const ompl::base::State* s1, const ompl::base::State* s2) const override {
            return motionCost(s1, s2);
        }

        ompl::base::Cost motionCostBestEstimate(const ompl::base::State* s1, const ompl::base::State* s2) const {
            return motionCost(s1, s2);
        }

    private:
        XArmForwardKinematics xarm_fk;

};

//TODO move this to our package instead of moveit, ref it there
class VADERCustomObjective2 : public ompl::base::MultiOptimizationObjective
{
public:
  VADERCustomObjective2(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::MultiOptimizationObjective(si)
    {    
    //   ROS_INFO_NAMED("VADERCustomObjective", "Starting");

      addObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si), 10.0);
      addObjective(std::make_shared<ompl::base::MaximizeMinClearanceObjective>(si), 1.0);
    }
  
    // Cost of motion between two states. This is the ONLY method that is called when planning.
    ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override
    {
      ompl::base::Cost c = identityCost();
      for (const auto &component : components_)
      {
          c = ompl::base::Cost(c.value() + component.weight * (component.objective->motionCost(s1, s2).value()));
      }

      // ROS_INFO_NAMED("VADERCustomObjective", "Motion cost: %f", c.value());
      return c;
    }
};



int main() {
    // XArmForwardKinematics xarm;
    // std::array<double, XArmForwardKinematics::N_JOINTS> joint_positions = {0.097,0.823,-0.67,1.758,-0.785,0,-0.384};
    // auto start = std::chrono::high_resolution_clock::now();
    // XArmForwardKinematics::Matrix4d T = xarm.forward_kinematics(joint_positions);
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> elapsed = end - start;
    // std::cout << "forward_kinematics took " << elapsed.count() << " ms\n";
    // std::cout << "End-effector transform:\n" << T << std::endl;

    // Example joint positions
    std::array<double, XArmForwardKinematics::N_JOINTS> joints1 = {0.0, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
    std::array<double, XArmForwardKinematics::N_JOINTS> joints2 = {0.2, -0.1, 0.4, -0.3, 0.6, -0.5, 0.8};

    // Create dummy OMPL state space and states
    // Linker error
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(XArmForwardKinematics::N_JOINTS);
    ompl::base::ScopedState<> state1(space), state2(space);
    for (size_t i = 0; i < XArmForwardKinematics::N_JOINTS; ++i) {
        state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = joints1[i];
        state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = joints2[i];
    }

    // Create SpaceInformation pointer
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);

    // Create the objective
    XarmTaskSpaceOptimizationObjective objective(si);

    // Compute and print the motion cost
    double total_time_ms = 0.0;
    ompl::base::Cost cost;
    for (int i = 0; i < 50; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        cost = objective.motionCost(state1.get(), state2.get());
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        total_time_ms += elapsed.count();
    }
    double avg_time_ms = total_time_ms / 50.0;
    // Avg. duration is 20 microseconds per call
    std::cout << "Average motionCost duration over 50 runs: " << avg_time_ms << " ms" << std::endl;
    std::cout << "Motion cost between joint positions: " << cost.value() << std::endl;
    return 0;
}