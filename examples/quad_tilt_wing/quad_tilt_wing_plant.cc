#include "drake/examples/quad_tilt_wing/quad_tilt_wing_plant.h"

#include <memory>
#include <math.h>
#include <iostream>

#include "drake/common/default_scalars.h"
#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/time_varying_data.h"
#include "drake/systems/primitives/piecewise_polynomial_affine_system.h"

using Eigen::Matrix3d;

namespace drake {
namespace examples {
namespace quad_tilt_wing {

namespace {
Matrix3d default_moment_of_inertia() {
  return (Eigen::Matrix3d() <<  // BR
          0.03504, 0, 0,  // BR
          0, 0.02011, 0,  // BR
          0, 0, 0.05504).finished();
}
}  // namespace

template <typename T>
QuadTiltWingPlant<T>::QuadTiltWingPlant()
    : QuadTiltWingPlant(0.91,    // m (kg)
                     0.209,  // rear_joint_x (m)
                     0.286,  // front_joint_x (m)
                     0.5,    // rear_wing_len (m)
                     0.15,   // rear_wing_wid (m)
                     0.25,   // front_wing_len (m)
                     0.1,    // front_wing_wid (m)
                     0.156,  // front_prop_y (m)
                     0.245,  // rear_prop_y (m)
                     5e-6,    // kProp
                     0.1,  // kLambda
                     default_moment_of_inertia()
                     ) {}

template <typename T>
QuadTiltWingPlant<T>::QuadTiltWingPlant(double m_arg, double rear_joint_x_arg,
                                  double front_joint_x_arg, double rear_wing_len_arg,
                                  double rear_wing_wid_arg, double front_wing_len_arg,
                                  double front_wing_wid_arg, double front_prop_y_arg,
                                  double rear_prop_y_arg, double kProp_arg, double kLambda_arg,
                                  const Matrix3d& I_arg)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<quad_tilt_wing::QuadTiltWingPlant>{}),
      rho_{1.225}, g_{9.81}, J_prop_{5e-5}, m_(m_arg), rear_joint_x_(rear_joint_x_arg), front_joint_x_(front_joint_x_arg),
      rear_wing_len_(rear_wing_len_arg), rear_wing_wid_(rear_wing_wid_arg), front_wing_len_(front_wing_len_arg),
      front_wing_wid_(front_wing_wid_arg), front_prop_y_(front_prop_y_arg), rear_prop_y_(rear_prop_y_arg),
      kProp_(kProp_arg), kLambda_(kLambda_arg), I_(I_arg) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareVectorOutputPort(systems::BasicVector<T>(kStateDimension),
                                &QuadTiltWingPlant::CopyStateOut);
}

template <typename T>
template <typename U>
QuadTiltWingPlant<T>:: QuadTiltWingPlant(const QuadTiltWingPlant<U>& other)
    : QuadTiltWingPlant<T>(other.m_, other.rear_joint_x_, other.front_joint_x_,
                           other.rear_wing_len_, other.rear_wing_wid_,
                           other.front_wing_len_, other.front_wing_wid_,
                           other.front_prop_y_, other.rear_prop_y_,
                           other.kProp_, other.kLambda_, other.I_) {}  // this is just a copy constructor

template <typename T>
QuadTiltWingPlant<T>::~QuadTiltWingPlant() {}

template <typename T>
void QuadTiltWingPlant<T>::CopyStateOut(const systems::Context<T> &context,
                                     systems::BasicVector<T> *output) const {
  // VectorX<T> state = context.get_continuous_state_vector().CopyToVector();    //state variable
//   VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();
//   VectorX<T> combined_vector(kStateDimension+8);
//   combined_vector << state, Eigen::VectorXd::Zero(8);
//   std::cout << "combined output: " << combined_vector << std::endl;
//   output->set_value(combined_vector);
  output->set_value(
      context.get_continuous_state_vector().CopyToVector());            // I guess the output is the output port
}

template <typename T>
void QuadTiltWingPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
        // state = [X,Y,Z,phi,theta,psi,dot_X,dot_Y,dot_Z,dot_phi,dot_theta,dot_psi]
  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();



        // u = [omega_1^2, omega_2^2, omega_3^2, omega_4^2, theta_1, theta_2, theta_3, theta_4]
  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();        // get value from the input port, which is the controller



  // Extract orientation, angular velocities and linear velocities.
  Vector3<T> rpy = state.segment(3, 3);
  Vector3<T> rpy_dot = state.segment(9, 3);
  Vector3<T> xyz_dot = state.segment(6, 3);
  VectorX<T> prop_speed2 = u.segment(0,4);  // omega_1^2, omega_2^2, omega_3^2, omega_4^2
  VectorX<T> tilt_angle = u.segment(4,4);   // theta_1, theta_2, theta_3, theta_4

  // Convert orientation to a rotation matrix.
  Matrix3<T> R = drake::math::rpy2rotmat(rpy);

  /////////// Compute forces /////////////////////
  // Compute the propeller net forces.
  VectorX<T> uF = kProp_ * prop_speed2;
  // Compute force rotational matrix due to wing tilting
  Eigen::Matrix<T, 3, 4> thrust_rot;
  thrust_rot << cos(tilt_angle(0)), cos(tilt_angle(1)), cos(tilt_angle(2)), cos(tilt_angle(3)),
                0, 0, 0, 0,
                -sin(tilt_angle(0)), -sin(tilt_angle(1)), -sin(tilt_angle(2)), -sin(tilt_angle(3));
  Vector3<T> F_th = thrust_rot*uF;

  //// Compute aerodynamic forces
  double front_wing_A = front_wing_len_*front_wing_wid_;
  double rear_wing_A = rear_wing_len_*rear_wing_wid_;
  auto xz_speed2 = pow(xyz_dot(2), 2) + pow(xyz_dot(0), 2);
  // compute angle of attack
  double v_tol = 1e-6;
  auto alpha_wind = atan2(xyz_dot(2), xyz_dot(0));
  if (abs(xyz_dot(0)) < v_tol) { alpha_wind = 0; }
  else { alpha_wind = atan2(xyz_dot(2), xyz_dot(0)); }
  VectorX<T> angle_of_attack = (-tilt_angle.array() - alpha_wind).matrix();  //alpha = -theta - arctan(v_z, v_x)
  VectorX<T> lift_coef = (2*angle_of_attack.array().sin()*angle_of_attack.array().cos()).matrix();
  VectorX<T> drag_coef = 2*(angle_of_attack.array().sin().square()).matrix(); // convert between array and matrix, because only array supports element wise operations
  // compute and add forces from each wing
  /*
  Vector3<T> F_w(0,0,0);
  for (int i = 0; i<4; i++) {
      double A;
      if i<2 { A = front_wing_A; }
      else { A = rear_wing_A }
      Vector3<T> w_coef << drag_coef(i), 0, lift_coef(i);
      Vector3<T> w_rot_rpy(0, angle_of_attack(i)-tilt_angle(i), 0);
      Matrix3d w_rotmat = drake::math::rpy2rotmat(w_rot_rpy);
      F_w += w_rotmat*w_coef*(-0.5)*rho_*A*xz_speed2;
      }
  */
  Vector3<T> w_coef_1(-drag_coef(0), 0, lift_coef(0));
  Vector3<T> w_coef_2(-drag_coef(1), 0, lift_coef(1));
  Vector3<T> w_coef_3(-drag_coef(2), 0, lift_coef(2));
  Vector3<T> w_coef_4(-drag_coef(3), 0, lift_coef(3));

  Vector3<T> w_rot_rpy_1(0, angle_of_attack(0)+tilt_angle(0), 0);
  Matrix3<T> w_rotmat_1 = drake::math::rpy2rotmat(w_rot_rpy_1);
  Vector3<T> w_rot_rpy_2(0, angle_of_attack(1)+tilt_angle(1), 0);
  Matrix3<T> w_rotmat_2 = drake::math::rpy2rotmat(w_rot_rpy_2);
  Vector3<T> w_rot_rpy_3(0, angle_of_attack(2)+tilt_angle(2), 0);
  Matrix3<T> w_rotmat_3 = drake::math::rpy2rotmat(w_rot_rpy_3);
  Vector3<T> w_rot_rpy_4(0, angle_of_attack(3)+tilt_angle(3), 0);
  Matrix3<T> w_rotmat_4 = drake::math::rpy2rotmat(w_rot_rpy_4);

  Vector3<T> F_w_1 = w_rotmat_1*w_coef_1*(0.5)*rho_*front_wing_A*xz_speed2;
  Vector3<T> F_w_2 = w_rotmat_2*w_coef_2*(0.5)*rho_*front_wing_A*xz_speed2;
  Vector3<T> F_w_3 = w_rotmat_3*w_coef_3*(0.5)*rho_*rear_wing_A*xz_speed2;
  Vector3<T> F_w_4 = w_rotmat_4*w_coef_4*(0.5)*rho_*rear_wing_A*xz_speed2;
  Vector3<T> F_w = F_w_1+F_w_2+F_w_3+F_w_4;

  // compute the resultant linear acceleration due to the forces
  Vector3<T> Fg(0, 0, -m_ * g_);
  Vector3<T> xyz_ddot = (1.0 / m_)*(Fg + R*(F_th+F_w));

  //////// Compute moment /////////////////////////
  // moment generated by props
  Eigen::Matrix<T, 3, 4> moment_rot;
  moment_rot(0,0) = -front_prop_y_*sin(tilt_angle(0)) - kLambda_*cos(tilt_angle(0));
  moment_rot(0,1) = front_prop_y_*sin(tilt_angle(1)) + kLambda_*cos(tilt_angle(1));
  moment_rot(0,2) = -rear_prop_y_*sin(tilt_angle(2)) + kLambda_*cos(tilt_angle(2));
  moment_rot(0,3) = rear_prop_y_*sin(tilt_angle(3)) - kLambda_*cos(tilt_angle(3));

  moment_rot(1,0) = front_joint_x_*sin(tilt_angle(0));
  moment_rot(1,1) = front_joint_x_*sin(tilt_angle(1));
  moment_rot(1,2) = -rear_joint_x_*sin(tilt_angle(2));
  moment_rot(1,3) = -rear_joint_x_*sin(tilt_angle(3));

  moment_rot(2,0) = -front_prop_y_*cos(tilt_angle(0)) + kLambda_*sin(tilt_angle(0));
  moment_rot(2,1) = front_prop_y_*cos(tilt_angle(1)) - kLambda_*sin(tilt_angle(1));
  moment_rot(2,2) = -rear_prop_y_*cos(tilt_angle(2)) - kLambda_*sin(tilt_angle(2));
  moment_rot(2,3) = rear_prop_y_*cos(tilt_angle(3)) + kLambda_*sin(tilt_angle(3));

  Vector3<T> M_th = moment_rot * uF;
  // moment generated by aerodynamic forces
  double r_s1 = front_wing_len_/2;
  double r_s2 = r_s1;
  double r_s3 = rear_wing_len_/2;
  double r_s4 = r_s3;
  auto r_l1 = front_joint_x_ - 0.25*front_wing_wid_*cos(tilt_angle(0));
  auto r_l2 = front_joint_x_ - 0.25*front_wing_wid_*cos(tilt_angle(1));
  auto r_l3 = rear_joint_x_ + 0.25*rear_wing_wid_*cos(tilt_angle(2));
  auto r_l4 = rear_joint_x_ + 0.25*rear_wing_wid_*cos(tilt_angle(3));
  Vector3<T> M_w;
  M_w(0) = r_s1*F_w_1(2) - r_s2*F_w_2(2) + r_s3*F_w_3(2) - r_s4*F_w_4(2);
  M_w(1) = -r_l1*F_w_1(2) - r_l2*F_w_2(2) + r_l3*F_w_3(2) + r_l4*F_w_4(2);
  M_w(2) = -r_s1*F_w_1(0) + r_s2*F_w_2(0) - r_s3*F_w_3(0) + r_s4*F_w_4(0);

  //~ // moment generated by gyroscopic effects of the props
  //~ Vector3<T> gyro_1(cos(tilt_angle(0)), 0, -sin(tilt_angle(0)));
  //~ Vector3<T> gyro_2(cos(tilt_angle(1)), 0, -sin(tilt_angle(1)));
  //~ Vector3<T> gyro_3(cos(tilt_angle(2)), 0, -sin(tilt_angle(2)));
  //~ Vector3<T> gyro_4(cos(tilt_angle(3)), 0, -sin(tilt_angle(3)));
  //~ VectorX<T> prop_speed = prop_speed2.cwiseSqrt();
  Matrix3<T> E;
  E << 1, 0, -sin(rpy(1)),
       0, cos(rpy(0)), sin(rpy(0))*cos(rpy(1)),
       0, -sin(rpy(0)), cos(rpy(0))*cos(rpy(1));
  Vector3<T> pqr = E*rpy_dot;
  //~ Vector3<T> M_gyro(0,0,0);
  //~ Vector3<T> gyro(0,0,0);
  //~ double eta;
  //~ for (int i=0; i<4; i++) {
      //~ gyro << cos(tilt_angle(i)), 0, -sin(tilt_angle(i));
      //~ if (i == 0 or i == 3) { eta = 1.0; }
      //~ else { eta = -1.0; }
      //~ M_gyro += J_prop_ * eta * prop_speed(i) * pqr.cross(gyro);
      //~ }
  Vector3<T> M_t = M_th + M_w;

  //~ Vector3<T> pqr;
  //~ rpydot2angularvel(rpy, rpy_dot, pqr);
  //~ pqr = R.adjoint() * pqr;

  // Computing the resultant angular acceleration due to the moments.
  Vector3<T> pqr_dot = I_.ldlt().solve(M_t - pqr.cross(I_ * pqr)); // this is equal to saying I_*pqr_dot = M - pqr.cross(I_ * pqr)
  Matrix3<T> Phi;
  typename drake::math::Gradient<Matrix3<T>, 3>::type dPhi;
  typename drake::math::Gradient<Matrix3<T>, 3, 2>::type* ddPhi = nullptr;
  angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

  MatrixX<T> drpy2drotmat = drake::math::drpy2rotmat(rpy);              // not sure what algorithm is running here??
  VectorX<T> Rdot_vec(9);
  Rdot_vec = drpy2drotmat * rpy_dot;
  Matrix3<T> Rdot = Eigen::Map<Matrix3<T>>(Rdot_vec.data());
  VectorX<T> dPhi_x_rpydot_vec;
  dPhi_x_rpydot_vec = dPhi * rpy_dot;
  Matrix3<T> dPhi_x_rpydot = Eigen::Map<Matrix3<T>>(dPhi_x_rpydot_vec.data());
  Vector3<T> rpy_ddot =
      Phi * R * pqr_dot + dPhi_x_rpydot * R * pqr + Phi * Rdot * pqr;

  // Recomposing the derivatives vector.
  VectorX<T> xdot(12);
  xdot << state.tail(6), xyz_ddot, rpy_ddot;

  //~ std::cout << "inside plant, u is:" << u << std::endl;
  //~ std::cout << "inside plant, x is:" << state << std::endl;
  //~ std::cout << "inside plant, xdot is:" << xdot << std::endl;

  derivatives->SetFromVector(xdot);
}

// Declare storage for our constants.
template <typename T>
constexpr int QuadTiltWingPlant<T>::kStateDimension;
template <typename T>
constexpr int QuadTiltWingPlant<T>::kInputDimension;

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadTiltWingPlant<double>* quad_tilt_wing_plant,
    Eigen::Vector3d nominal_position) {
  auto quad_tilt_wing_context_goal = quad_tilt_wing_plant->CreateDefaultContext();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) = nominal_position;
  x0(3) = 0;

  std::cout << "x0: " << x0 << std::endl;

  // Nominal input corresponds to a hover.
  double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
  std::cout << rear_moment_arm << std::endl;
  double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
  std::cout << front_moment_arm << std::endl;
  double UAV_fg = quad_tilt_wing_plant->m() * quad_tilt_wing_plant->g();
  std::cout << UAV_fg << std::endl;
  double front_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * rear_moment_arm / 2;
  std::cout << front_prop_f << std::endl;
  double rear_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * front_moment_arm / 2;
  std::cout << rear_prop_f << std::endl;
  Eigen::Vector4d u0_prop{front_prop_f/quad_tilt_wing_plant->kProp(), front_prop_f/quad_tilt_wing_plant->kProp(),
      rear_prop_f/quad_tilt_wing_plant->kProp(), rear_prop_f/quad_tilt_wing_plant->kProp()};
  Eigen::VectorXd u0_tilt = Eigen::VectorXd::Constant(4, -M_PI/2);

  std::cout << u0_prop << std::endl;
  std::cout << u0_tilt << std::endl;

  Eigen::VectorXd u0(8);
  u0 << u0_prop, u0_tilt;

  std::cout << "u0: " << u0 << std::endl;

  quad_tilt_wing_context_goal->FixInputPort(0, u0);
  quad_tilt_wing_plant->set_state(quad_tilt_wing_context_goal.get(), x0);              //where is the linearization part?

  // Setup LQR cost matrices (penalize position error 10x more than velocity
  // error).
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
  Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(8, 8);
  R.topLeftCorner<4, 4>() = 1e-8 * Eigen::MatrixXd::Identity(4, 4);
  R.bottomRightCorner<4, 4>() = 1e2 * Eigen::MatrixXd::Identity(4, 4);

  std::cout << "GOT Q and R." << std::endl;

  return systems::controllers::LinearQuadraticRegulator(
      *quad_tilt_wing_plant, *quad_tilt_wing_context_goal, Q, R);
}

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRControllerUpright(
    const QuadTiltWingPlant<double>* quad_tilt_wing_plant,
    Eigen::Vector3d nominal_position) {
  auto quad_tilt_wing_context_goal = quad_tilt_wing_plant->CreateDefaultContext();

  double pitch_angle = 0;
  pitch_angle = - 80.0 / 180.0 * M_PI;
  //~ double pitch_angle = -M_PI/6;

  std::cout << "Pitch angle: " << pitch_angle << std::endl;

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) = nominal_position;
  x0(4) = pitch_angle;

  std::cout << "x0: " << x0 << std::endl;

  // Nominal input corresponds to a hover.
  double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
  std::cout << rear_moment_arm << std::endl;
  double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
  std::cout << front_moment_arm << std::endl;
  double UAV_fg = quad_tilt_wing_plant->m() * quad_tilt_wing_plant->g();
  std::cout << UAV_fg << std::endl;
  double front_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * rear_moment_arm / 2;
  std::cout << front_prop_f << std::endl;
  double rear_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * front_moment_arm / 2;
  std::cout << rear_prop_f << std::endl;
  Eigen::Vector4d u0_prop{front_prop_f/quad_tilt_wing_plant->kProp(), front_prop_f/quad_tilt_wing_plant->kProp(),
      rear_prop_f/quad_tilt_wing_plant->kProp(), rear_prop_f/quad_tilt_wing_plant->kProp()};
  Eigen::VectorXd u0_tilt = Eigen::VectorXd::Constant(4, -(M_PI/2 + pitch_angle));

  std::cout << u0_prop << std::endl;
  std::cout << u0_tilt << std::endl;

  Eigen::VectorXd u0(8);
  u0 << u0_prop, u0_tilt;

  std::cout << "u0: " << u0 << std::endl;

  quad_tilt_wing_context_goal->FixInputPort(0, u0);
  quad_tilt_wing_plant->set_state(quad_tilt_wing_context_goal.get(), x0);              //where is the linearization part?

  // Setup LQR cost matrices
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
  Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(8, 8);
  R.topLeftCorner<4, 4>() = 1e-8 * Eigen::MatrixXd::Identity(4, 4);
  R.bottomRightCorner<4, 4>() = 1e2 * Eigen::MatrixXd::Identity(4, 4);

  std::cout << "GOT Q and R." << std::endl;

  return systems::controllers::LinearQuadraticRegulator(
      *quad_tilt_wing_plant, *quad_tilt_wing_context_goal, Q, R);
}

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRControllerWingtilt(
    const QuadTiltWingPlant<double>* quad_tilt_wing_plant,
    Eigen::Vector3d nominal_position) {
  auto quad_tilt_wing_context_goal = quad_tilt_wing_plant->CreateDefaultContext();
  double front_wing_tilt = -M_PI*2/3;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) = nominal_position;

  std::cout << "x0: " << x0 << std::endl;

  // Nominal input corresponds to a hover.
  double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
  double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
  double UAV_fg = quad_tilt_wing_plant->m() * quad_tilt_wing_plant->g();
  double c_f = front_moment_arm / (front_moment_arm + rear_moment_arm);
  double c_r = rear_moment_arm / (front_moment_arm + rear_moment_arm);

  double rear_wing_tilt = - atan(c_f * tan(front_wing_tilt) / c_r);

  double front_prop_f = UAV_fg * c_r / (-sin(front_wing_tilt)) / 2;
  double rear_prop_f = UAV_fg * c_f / (-sin(rear_wing_tilt)) / 2;

  Eigen::Vector4d u0_prop{front_prop_f/quad_tilt_wing_plant->kProp(), front_prop_f/quad_tilt_wing_plant->kProp(),
      rear_prop_f/quad_tilt_wing_plant->kProp(), rear_prop_f/quad_tilt_wing_plant->kProp()};
  Eigen::Vector4d u0_tilt{front_wing_tilt, front_wing_tilt, rear_wing_tilt, rear_wing_tilt};

  std::cout << u0_prop << std::endl;
  std::cout << u0_tilt << std::endl;

  Eigen::VectorXd u0(8);
  u0 << u0_prop, u0_tilt;

  std::cout << "u0: " << u0 << std::endl;

  quad_tilt_wing_context_goal->FixInputPort(0, u0);
  quad_tilt_wing_plant->set_state(quad_tilt_wing_context_goal.get(), x0);              //where is the linearization part?

  // Setup LQR cost matrices
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
  Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(8, 8);
  R.topLeftCorner<4, 4>() = 1e-8 * Eigen::MatrixXd::Identity(4, 4);
  R.bottomRightCorner<4, 4>() = 1e2 * Eigen::MatrixXd::Identity(4, 4);

  std::cout << "GOT Q and R." << std::endl;

  return systems::controllers::LinearQuadraticRegulator(
      *quad_tilt_wing_plant, *quad_tilt_wing_context_goal, Q, R);
}

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRControllerTrimPoint(
            const QuadTiltWingPlant<double>* quad_tilt_wing_plant,
            Eigen::VectorXd trim_point_state, Eigen::VectorXd trim_point_input) {
      auto quad_tilt_wing_context_goal = quad_tilt_wing_plant->CreateDefaultContext();
      Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
      x0 = trim_point_state;

      std::cout << "x0: " << x0 << std::endl;

      Eigen::VectorXd u0(8);
      u0 = trim_point_input;

      std::cout << "u0: " << u0 << std::endl;

      quad_tilt_wing_context_goal->FixInputPort(0, u0);
      quad_tilt_wing_plant->set_state(quad_tilt_wing_context_goal.get(), x0);              //where is the linearization part?

      // Setup LQR cost matrices
      Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
      Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);
      Q.topLeftCorner<1, 1>() = 0 * 10 * Eigen::MatrixXd::Identity(1, 1);  // do not punish X position

      Eigen::MatrixXd R = Eigen::MatrixXd::Identity(8, 8);
      R.topLeftCorner<4, 4>() = 1e-8 * Eigen::MatrixXd::Identity(4, 4);
      R.bottomRightCorner<4, 4>() = 1e2 * Eigen::MatrixXd::Identity(4, 4);

      std::cout << "GOT Q and R." << std::endl;

      return systems::controllers::LinearQuadraticRegulatorTrim(
              *quad_tilt_wing_plant, *quad_tilt_wing_context_goal, Q, R);
}

std::unique_ptr<systems::TimeVaryingAffineSystem<double>> TimeVaryingLinearQuadraticRegulator(
            const QuadTiltWingPlant<double>* quad_tilt_wing_plant,
            const trajectories::PiecewisePolynomial<double>& u_traj,
            const trajectories::PiecewisePolynomial<double>& x_traj,
            const Eigen::Ref<const Eigen::MatrixXd>& N) {

//      double total_time = x_traj.end_time();
      std::vector<double> segment_times = x_traj.get_segment_times();
      int num_segments = int (segment_times.size());
      std::cout << "num_segments: " << num_segments << std::endl;
      Eigen::VectorXd segment_times_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(segment_times.data(), segment_times.size());
      std::cout << "segment times: " << segment_times_eigen << std::endl;
      auto quad_tilt_wing_context_goal = quad_tilt_wing_plant->CreateDefaultContext();
      // Setup LQR cost matrices
      Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
      Q.topLeftCorner<6, 6>() =  10 * Eigen::MatrixXd::Identity(6, 6);

      Eigen::MatrixXd R = Eigen::MatrixXd::Identity(8, 8);
      R.topLeftCorner<4, 4>() = 1e-8 * Eigen::MatrixXd::Identity(4, 4);
      R.bottomRightCorner<4, 4>() = 1e2 * Eigen::MatrixXd::Identity(4, 4);

      std::cout << "GOT Q and R." << std::endl;
      const int num_inputs = quad_tilt_wing_plant->get_input_port(0).size(),
                num_states = quad_tilt_wing_context_goal->get_num_total_states();
      std::cout << "num_inputs: " << num_inputs << std::endl;
      std::cout << "num_states: " << num_states << std::endl;
      const double equilibrium_check_tolerance = 1e6;
      std::vector<MatrixX<double>> K;
      std::vector<MatrixX<double>> D;
      for (int i = 0; i<num_segments; i++){
        Eigen::VectorXd x0 = x_traj.value(segment_times[i]);
        Eigen::VectorXd u0 = u_traj.value(segment_times[i]);
        quad_tilt_wing_context_goal->FixInputPort(0,u0);
        quad_tilt_wing_plant->set_state(quad_tilt_wing_context_goal.get(), x0);
        auto linear_system = Linearize(*quad_tilt_wing_plant, *quad_tilt_wing_context_goal, 0, -3, equilibrium_check_tolerance);
        systems::controllers::LinearQuadraticRegulatorResult lqr_result =
                (linear_system->time_period() == 0.0)
                ? systems::controllers::LinearQuadraticRegulator(linear_system->A(), linear_system->B(), Q,
                                           R, N)
                : systems::controllers::DiscreteTimeLinearQuadraticRegulator(linear_system->A(),
                                                       linear_system->B(), Q, R);
//        std::cout << "Inside TVLQR, K is: " << lqr_result.K << std::endl;
        K.push_back(lqr_result.K);
        D.push_back(lqr_result.K*(-1.0));
      }

      std::cout << "Finished calculating K's." << std::endl;
    trajectories::PiecewisePolynomial<double> K_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, K);
      std::cout << "Generated K trajectory." << std::endl;

      std::cout << "K test: " << K_traj.value(10.3) << std::endl;

      std::vector<MatrixX<double>> A;
      for (int i = 0; i<num_segments; i++){
          A.push_back(Eigen::Matrix<double, 1, 1>::Zero());
      }
    trajectories::PiecewisePolynomial<double> A_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, A);
    std::cout << "A test: " << A_traj.value(10.3) << std::endl;

      std::vector<MatrixX<double>> B;
      for (int i = 0; i<num_segments; i++){
          B.push_back(Eigen::MatrixXd::Zero(1, num_states));
      }
    trajectories::PiecewisePolynomial<double> B_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, B);
      std::cout << "Generated B trajectory." << std::endl;
    std::cout << "B test: " << B_traj.value(10.3) << std::endl;

    std::vector<MatrixX<double>> f0;
    for (int i = 0; i<num_segments; i++){
        f0.push_back(Eigen::Matrix<double, 1, 1>::Zero());
    }
    trajectories::PiecewisePolynomial<double> f0_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, f0);
    std::cout << "Generated f0 trajectory." << std::endl;
    std::cout << "f0 test: " << f0_traj.value(10.3) << std::endl;

    std::vector<MatrixX<double>> C;
    for (int i = 0; i<num_segments; i++){
        C.push_back(Eigen::MatrixXd::Zero(num_inputs, 1));
    }
    trajectories::PiecewisePolynomial<double> C_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, C);
    std::cout << "Generated C trajectory." << std::endl;
    std::cout << "C test: " << C_traj.value(10.3) << std::endl;

    std::vector<MatrixX<double>> y0;
    for (int i = 0; i<num_segments; i++){
        y0.push_back(u_traj.value(segment_times[i]) + K[i] * x_traj.value(segment_times[i]));
    }

    trajectories::PiecewisePolynomial<double> y0_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, y0);
    std::cout << "Generated y0 trajectory." << std::endl;
    std::cout << "y0 test: " << y0_traj.value(10.3) << std::endl;

    trajectories::PiecewisePolynomial<double> D_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(segment_times, D);

    // test u_0
    std::cout << "x0 is: " << x_traj.value(0) << std::endl;
    std::cout << "K0 is: " << K_traj.value(0) << std::endl;
    std::cout << "y0 is: " << y0_traj.value(0) << std::endl;
    std::cout << "u0 is: " << u_traj.value(0) << std::endl;

    const systems::TimeVaryingData data = systems::TimeVaryingData(A_traj, B_traj, f0_traj, C_traj, D_traj, y0_traj);
    return std::make_unique<systems::PiecewisePolynomialAffineSystem<double>>(data, 0);
}


std::unique_ptr<systems::AffineSystem<double>> ArbitraryController(
    const QuadTiltWingPlant<double>* quad_tilt_wing_plant, Eigen::VectorXd arbitrary_control) {
    int num_inputs = quad_tilt_wing_plant->get_num_states();
    int num_outputs = quad_tilt_wing_plant->get_input_size();
    int num_states = 0;                                            //what does 0 mean here? initiate a matrix with 0 dimension?
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_states, num_states);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_states, num_inputs);
    Eigen::VectorXd f0 = Eigen::VectorXd::Zero(num_states);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);
    //~ Eigen::VectorXd y0 = Eigen::VectorXd::Zero(num_outputs);

    return std::make_unique<systems::AffineSystem<double>>(
          A,   // A
          B,  // B
          f0,   // xDot0
          C,  // C
          D,  // D
          arbitrary_control,                // y0
          0.0);
}

    template <typename T>
    std::vector<T> linspace(T a, T b, size_t N) {
        T h = (b - a) / static_cast<T>(N-1);
        std::vector<T> xs(N);
        typename std::vector<T>::iterator x;
        T val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
            *x = val;
        return xs;
    }



}  // namespace quad_tilt_wing
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::quad_tilt_wing::QuadTiltWingPlant)
