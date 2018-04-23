#include "drake/examples/quad_tilt_wing/quad_tilt_wing_plant.h"

#include <memory>
#include <math.h>

#include "drake/common/default_scalars.h"
#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/util/drakeGeometryUtil.h"

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
                     default_moment_of_inertia(),
                     5e-6,    // kProp
                     0.1  // kLambda
                     ) {}

template <typename T>
QuadTiltWingPlant<T>::QuadTiltWingPlant(double m_arg, double rear_joint_x_arg,
                                  double front_joint_x_arg, double rear_wing_len_arg,
                                  double rear_wing_wid_arg, double front_wing_len_arg,
                                  double front_wing_wid_arg, double front_prop_y_arg,
                                  double rear_prop_y_arg, const Matrix3d& I_arg,
                                  double kProp_arg, double kLambda_arg)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<quad_tilt_wing::QuadTiltWingPlant>{}),
      rho_{1.225}, g_{9.81}, m_(m_arg), rear_joint_x_(rear_joint_x_arg), front_joint_x_(front_joint_x_arg),
      rear_wing_len_(rear_wing_len_arg), rear_wing_wid_(rear_wing_wid_arg), front_wing_len_(front_wing_len_arg),
      front_wing_wid_(front_wing_wid_arg), front_prop_y_(front_prop_y_arg), rear_prop_y_(rear_prop_y_arg),
      kProp_(kProp_arg), kLambda_(kLambda_arg), I_(I_arg), J_prop_{5e-5} {
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
                           other.kProp_, other.kLambda_) {}  // this is just a copy constructor

template <typename T>
QuadTiltWingPlant<T>::~QuadTiltWingPlant() {}

template <typename T>
void QuadTiltWingPlant<T>::CopyStateOut(const systems::Context<T> &context,
                                     systems::BasicVector<T> *output) const {
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
  Eigen::Matrix<double, 3, 4> thrust_rot;
  thrust_rot << cos(tilt_angle(0)), cos(tilt_angle(1)), cos(tilt_angle(2)), cos(tilt_angle(3)),
                0, 0, 0, 0,
                -sin(tilt_angle(0)), -sin(tilt_angle(1)), -sin(tilt_angle(2)), -sin(tilt_angle(3));
  Vector3<T> F_th = thrust_rot*uF;

  //// Compute aerodynamic forces
  double front_wing_A = front_wing_len_*front_wing_wid_;
  double rear_wing_A = rear_wing_len_*rear_wing_wid_;
  double xz_speed2 = pow(xyz_dot(2), 2) + pow(xyz_dot(0), 2);
  // compute angle of attack
  VectorX<T> angle_of_attack = tilt_angle + atan2(xyz_dot(2), xyz_dot(0));  //alpha = theta + arctan(v_z, v_x)
  VectorX<T> lift_coef = 2*angle_of_attack.sin()*angle_of_attack.cos();
  VectorX<T> drag_coef = 2*angle_of_attack.sin().square();
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
  Vector3<T> w_coef_1(drag_coef(0), 0, lift_coef(0));
  Vector3<T> w_coef_2(drag_coef(1), 0, lift_coef(1));
  Vector3<T> w_coef_3(drag_coef(2), 0, lift_coef(2));
  Vector3<T> w_coef_4(drag_coef(3), 0, lift_coef(3));

  Vector3<T> w_rot_rpy_1(0, angle_of_attack(0)-tilt_angle(0), 0);
  Matrix3d w_rotmat_1 = drake::math::rpy2rotmat(w_rot_rpy_1);
  Vector3<T> w_rot_rpy_2(0, angle_of_attack(1)-tilt_angle(1), 0);
  Matrix3d w_rotmat_2 = drake::math::rpy2rotmat(w_rot_rpy_2);
  Vector3<T> w_rot_rpy_3(0, angle_of_attack(2)-tilt_angle(2), 0);
  Matrix3d w_rotmat_3 = drake::math::rpy2rotmat(w_rot_rpy_3);
  Vector3<T> w_rot_rpy_4(0, angle_of_attack(3)-tilt_angle(3), 0);
  Matrix3d w_rotmat_4 = drake::math::rpy2rotmat(w_rot_rpy_4);

  Vector3<T> F_w_1 = w_rotmat_1*w_coef_1*(-0.5)*rho_*A*xz_speed2;
  Vector3<T> F_w_2 = w_rotmat_2*w_coef_2*(-0.5)*rho_*A*xz_speed2;
  Vector3<T> F_w_3 = w_rotmat_3*w_coef_3*(-0.5)*rho_*A*xz_speed2;
  Vector3<T> F_w_4 = w_rotmat_4*w_coef_4*(-0.5)*rho_*A*xz_speed2;
  Vector3<T> F_w = F_w_1+F_w_2+F_w_3+F_w_4;

  // compute the resultant linear acceleration due to the forces
  Vector3<T> Fg(0, 0, -m_ * g_);
  Vector3<T> xyz_ddot = (1.0 / m_)*(Fg + R*(F_th+F_w))

  //////// Compute moment /////////////////////////
  // moment generated by props
  Eigen::Matrix<double, 3, 4> moment_rot;
  moment_rot(0,0) = front_prop_y_*sin(tilt_angle(0)) - kLambda_*cos(tilt_angle(0));
  moment_rot(0,1) = -front_prop_y_*sin(tilt_angle(1)) + kLambda_*cos(tilt_angle(1));
  moment_rot(0,2) = rear_prop_y_*sin(tilt_angle(2)) + kLambda_*cos(tilt_angle(2));
  moment_rot(0,3) = -rear_prop_y_*sin(tilt_angle(3)) - kLambda_*cos(tilt_angle(3));

  moment_rot(1,0) = front_joint_x_*sin(tilt_angle(0));
  moment_rot(1,1) = front_joint_x_*sin(tilt_angle(1));
  moment_rot(1,2) = -rear_joint_x_*sin(tilt_angle(2));
  moment_rot(1,3) = -rear_joint_x_*sin(tilt_angle(3));

  moment_rot(2,0) = front_prop_y_*cos(tilt_angle(0)) + kLambda_*sin(tilt_angle(0));
  moment_rot(2,1) = -front_prop_y_*cos(tilt_angle(1)) - kLambda_*sin(tilt_angle(1));
  moment_rot(2,2) = rear_prop_y_*cos(tilt_angle(2)) - kLambda_*sin(tilt_angle(2));
  moment_rot(2,3) = -rear_prop_y_*cos(tilt_angle(3)) + kLambda_*sin(tilt_angle(3));

  Vector3<T> M_th = moment_rot * uF;
  // moment generated by aerodynamic forces
  double r_s1 = front_wing_len_/2;
  double r_s2 = r_s1;
  double r_s3 = rear_wing_len_/2;
  double r_s4 = r_s3;
  double r_l1 = front_joint_x_ - 0.25*front_wing_wid_;
  double r_l2 = r_l1;
  double r_l3 = rear_joint_x_ + 0.25*rear_wing_wid_;
  double r_l4 = r_l3;
  Vector3<T> M_w;
  M_w(0) = -r_s1*F_w_1(2) + r_s2*F_w_2(2) - r_s3*F_w_3(2) + r_s4*F_w_4(2);
  M_w(1) = -r_l1*F_w_1(2) - r_l2*F_W_2(2) + r_l3*F_w_3(2) + r_l4*F_w_4(2);
  M_w(2) = r_s1*F_w_1(0) - r_s2*F_w_2(0) + r_s3*F_w_3(0) - r_s4*F_w_4(0);

  // moment generated by gyroscopic effects of the props
  Vector3<T> gyro_1(cos(tilt_angle(0)), 0, -sin(tilt_angle(0)));
  Vector3<T> gyro_2(cos(tilt_angle(1)), 0, -sin(tilt_angle(1)));
  Vector3<T> gyro_3(cos(tilt_angle(2)), 0, -sin(tilt_angle(2)));
  Vector3<T> gyro_4(cos(tilt_angle(3)), 0, -sin(tilt_angle(3)));
  VectorX<T> prop_speed = prop_speed2.sqrt();






  VectorX<T> uM = kM_ * u;


  Vector3<T> F(0, 0, uF.sum());
  Vector3<T> M(L_ * (uF(1) - uF(3)), L_ * (uF(2) - uF(0)),
               uM(0) - uM(1) + uM(2) - uM(3));

  // Computing the resultant linear acceleration due to the forces.
  Vector3<T> xyz_ddot = (1.0 / m_) * (Fg + R * F);

  Vector3<T> pqr;
  rpydot2angularvel(rpy, rpy_dot, pqr);
  pqr = R.adjoint() * pqr;

  // Computing the resultant angular acceleration due to the moments.
  Vector3<T> pqr_dot = I_.ldlt().solve(M - pqr.cross(I_ * pqr));
  Matrix3<T> Phi;
  typename drake::math::Gradient<Matrix3<T>, 3>::type dPhi;
  typename drake::math::Gradient<Matrix3<T>, 3, 2>::type* ddPhi = nullptr;
  angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

  MatrixX<T> drpy2drotmat = drake::math::drpy2rotmat(rpy);
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

  // Nominal input corresponds to a hover.
  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(
      4, quad_tilt_wing_plant->m() * quad_tilt_wing_plant->g() / 4);

  quad_tilt_wing_context_goal->FixInputPort(0, u0);
  quad_tilt_wing_plant->set_state(quad_tilt_wing_context_goal.get(), x0);              //where is the linearization part?

  // Setup LQR cost matrices (penalize position error 10x more than velocity
  // error).
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
  Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);

  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();

  return systems::controllers::LinearQuadraticRegulator(
      *quad_tilt_wing_plant, *quad_tilt_wing_context_goal, Q, R);
}

}  // namespace quad_tilt_wing
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::quad_tilt_wing::QuadTiltWingPlant)
