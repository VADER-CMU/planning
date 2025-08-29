
#include <iostream>

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
using Matrix4d = Eigen::Matrix4d;

// class VADERCustomObjective : public ompl::base::MultiOptimizationObjective
// {
// public:
//   VADERCustomObjective(const ompl::base::SpaceInformationPtr& si)
//   : ompl::base::MultiOptimizationObjective(si)
//     {
//       addObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si), 10.0);
//       addObjective(std::make_shared<ompl::base::MaximizeMinClearanceObjective>(si), 1.0);
//     }

//   ompl::base::Cost stateCost(const ompl::base::State* s) const override
//   {
//     ompl::base::Cost c = super::stateCost(s);
//     std::cout << "State cost: " << c.value() << std::endl;
//     return c;
//   }
// };


constexpr size_t N_JOINTS = 7;

struct DHParams {
    double d, a, alpha; // only θ is joint-variable in this example
};

// D-H params (see: http://help.ufactory.cc/en/articles/4330809-kinematic-and-dynamic-parameters-of-ufactory-xarm-series)
// Note these are STANDARD D-H parameters. If this is wrong, try the MODIFIED D-H parameters on the same page.

constexpr std::array<DHParams, N_JOINTS> dh = {{
    { /*d1*/ 0.267, /*a1*/ 0.0, /*alpha1*/ -M_PI/2 },
    {        0.000,         0.0,           M_PI/2 },
    {        0.293,         0.0525,        M_PI/2 },
    {        0.000,         0.0775,        M_PI/2 },
    {        0.3425,        0.0,           M_PI/2},
    {        0.000,         0.076,         -M_PI/2},
    {        0.097,         0.0,           0.0},
}};

// Precompute cos(alpha) and sin(alpha) at compile time
constexpr std::array<double, N_JOINTS> cos_alpha = []{
    std::array<double, N_JOINTS> c = {};
    for (size_t i = 0; i < N_JOINTS; ++i) c[i] = std::cos(dh[i].alpha);
    return c;
}();

constexpr std::array<double, N_JOINTS> sin_alpha = []{
    std::array<double, N_JOINTS> s = {};
    for (size_t i = 0; i < N_JOINTS; ++i) s[i] = std::sin(dh[i].alpha);
    return s;
}();

// Runtime: Only cos(theta), sin(theta) are computed and used
Matrix4d joint_transform(size_t i, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);

    Matrix4d T;
    T << ct, -st*cos_alpha[i],  st*sin_alpha[i], dh[i].a*ct,
         st,  ct*cos_alpha[i], -ct*sin_alpha[i], dh[i].a*st,
          0,       sin_alpha[i],      cos_alpha[i],    dh[i].d,
          0,           0,                 0,           1;
    return T;
}

Matrix4d forward_kinematics(const std::array<double, N_JOINTS>& joint_positions) {
    Matrix4d T = Matrix4d::Identity();
    for (size_t i = 0; i < N_JOINTS; ++i) {
        T = T * joint_transform(i, joint_positions[i]);
    }
    return T;
}


using Matrix4d = Eigen::Matrix4d;

int main() {
  std::array<double, N_JOINTS> joint_positions = {0,0,0,0,0,0,0};
  auto start = std::chrono::high_resolution_clock::now();
  Matrix4d T = forward_kinematics(joint_positions);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << "forward_kinematics took " << elapsed.count() << " ms\n";

  std::cout << "End-effector transform:\n" << T << std::endl;
  return 0;
}