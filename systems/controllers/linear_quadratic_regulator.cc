#include <math.h>
#include <iostream>

#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/time_varying_data.h"
#include "drake/systems/primitives/piecewise_polynomial_affine_system.h"

namespace drake {
namespace systems {
namespace controllers {

LinearQuadraticRegulatorResult LinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  Eigen::Index n = A.rows(), m = B.cols();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(B.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  // N is default to Matrix<double, 0, 0>.
  if (N.rows() != 0) {
    DRAKE_DEMAND(N.rows() == n && N.cols() == m);
  }
  DRAKE_DEMAND(is_approx_equal_abstol(R, R.transpose(), 1e-10));

  LinearQuadraticRegulatorResult ret;

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  if (R_cholesky.info() != Eigen::Success)
    throw std::runtime_error("R must be positive definite");

  if (N.rows() != 0) {
    Eigen::MatrixXd Q1 = Q - N * R_cholesky.solve(N.transpose());
    Eigen::MatrixXd A1 = A - B * R_cholesky.solve(N.transpose());

    ret.S = math::ContinuousAlgebraicRiccatiEquation(A1, B, Q1, R_cholesky);
    ret.K = R_cholesky.solve(B.transpose() * ret.S + N.transpose());
  } else {
    ret.S = math::ContinuousAlgebraicRiccatiEquation(A, B, Q, R_cholesky);
    ret.K = R_cholesky.solve(B.transpose() * ret.S);
  }
  return ret;
}

LinearQuadraticRegulatorResult DiscreteTimeLinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  Eigen::Index n = A.rows(), m = B.cols();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(B.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  DRAKE_DEMAND(is_approx_equal_abstol(R, R.transpose(), 1e-10));

  LinearQuadraticRegulatorResult ret;

  ret.S = math::DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

  Eigen::MatrixXd tmp = B.transpose() * ret.S * B + R;
  ret.K = tmp.llt().solve(B.transpose() * ret.S * A);

  return ret;
}

std::unique_ptr<systems::LinearSystem<double>> LinearQuadraticRegulator(
    const LinearSystem<double>& system,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  // DiscreteTimeLinearQuadraticRegulator does not support N yet.
  DRAKE_DEMAND(system.time_period() == 0.0 || N.rows() == 0);

  const int num_states = system.B().rows(), num_inputs = system.B().cols();

  LinearQuadraticRegulatorResult lqr_result = (system.time_period() == 0.0) ?
    LinearQuadraticRegulator(system.A(), system.B(), Q, R, N) :
    DiscreteTimeLinearQuadraticRegulator(system.A(), system.B(), Q, R);

  // Return the controller: u = -Kx.
  return std::make_unique<systems::LinearSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -lqr_result.K,                         // D
      system.time_period());
}

std::unique_ptr<systems::AffineSystem<double>> LinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  // TODO(russt): accept optional additional argument to return the cost-to-go
  // but note that it will be a full quadratic form (x'S2x + s1'x + s0).

  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  const int num_inputs = system.get_input_port(0).size(),
            num_states = context.get_num_total_states();
  std::cout << "num_inputs inside LQR: " << num_inputs << "num_states inside LQR: " << num_states << std::endl;
  DRAKE_DEMAND(num_states > 0);
  // The Linearize method call below will verify that the system has either
  // continuous-time OR (only simple) discrete-time dyanmics.

  // TODO(russt): Confirm behavior if Q is not PSD.

  // Use first input and no outputs (the output dynamics are irrelevant for
  // LQR design).
  auto linear_system = Linearize(system, context, 0, kNoOutput);
  
  std::cout << "linear_system A: " << linear_system->A() << std::endl;
  std::cout << "linear_system B: " << linear_system->B() << std::endl;

  // DiscreteTimeLinearQuadraticRegulator does not support N yet.
  DRAKE_DEMAND(linear_system->time_period() == 0.0 || N.rows() == 0);

  LinearQuadraticRegulatorResult lqr_result =
      (linear_system->time_period() == 0.0)
          ? LinearQuadraticRegulator(linear_system->A(), linear_system->B(), Q,
                                     R, N)
          : DiscreteTimeLinearQuadraticRegulator(linear_system->A(),
                                                 linear_system->B(), Q, R);

  const Eigen::VectorXd& x0 =
      (linear_system->time_period() == 0.0)
          ? context.get_continuous_state_vector().CopyToVector()
          : context.get_discrete_state(0).CopyToVector();

  const auto& u0 = system.EvalEigenVectorInput(context, 0);
  std::cout << "u0 inside LQR: " << u0 << std::endl;
  
  std::cout << "K inside LQR: " << lqr_result.K << std::endl;

  // Return the affine controller: u = u0 - K(x-x0).
  return std::make_unique<systems::AffineSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // xDot0
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -lqr_result.K,                         // D
      u0 + lqr_result.K * x0,                // y0
      linear_system->time_period());
}

std::unique_ptr<systems::AffineSystem<double>> LinearQuadraticRegulatorTrim(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {

  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  const int num_inputs = system.get_input_port(0).size(),
            num_states = context.get_num_total_states();
  std::cout << "num_inputs inside LQR: " << num_inputs << "num_states inside LQR: " << num_states << std::endl;
  DRAKE_DEMAND(num_states > 0);
  // The Linearize method call below will verify that the system has either
  // continuous-time OR (only simple) discrete-time dyanmics.

  // Added by Hank Yang here so that linearization can be done around a trim point
  double equilibrium_check_tolerance = 1e-6;
  for (int i=0; i<Q.rows(); i++){
	  if (Q(i,i)==0){ equilibrium_check_tolerance = 1e6; }
  }

  // Use first input and no outputs (the output dynamics are irrelevant for
  // LQR design).
  std::cout << "equilibrium_check_tolerance" << equilibrium_check_tolerance << std::endl;
  auto linear_system = Linearize(system, context, 0, kNoOutput, equilibrium_check_tolerance);
  
  std::cout << "linear_system A: " << linear_system->A() << std::endl;
  std::cout << "linear_system B: " << linear_system->B() << std::endl;
  Eigen::MatrixXd A_reduced(num_states-1, num_states-1);
  A_reduced << linear_system->A().bottomRightCorner(num_states-1, num_states-1);
  Eigen::MatrixXd B_reduced(num_states-1, num_inputs);
  B_reduced << linear_system->B().bottomRows(num_states-1);
  Eigen::MatrixXd Q_reduced(num_states-1, num_states-1);
  Q_reduced << Q.bottomRightCorner(num_states-1, num_states-1);
  //~ Eigen::Matrix<double, num_states-1, num_states-1> A_reduced = linear_system->A().bottomRightCorner(num_states-1, num_states-1);
  //~ Eigen::Matrix<double, num_inputs, num_states-1> B_reduced = linear_system->B().bottomRows(num_states-1);
  //~ Eigen::Matrix<double, num_states-1, num_states-1> Q_reduced = Q.bottomRightCorner(num_states-1, num_states-1);

  // DiscreteTimeLinearQuadraticRegulator does not support N yet.
  DRAKE_DEMAND(linear_system->time_period() == 0.0 || N.rows() == 0);

  LinearQuadraticRegulatorResult lqr_result =
      (linear_system->time_period() == 0.0)
          ? LinearQuadraticRegulator(A_reduced, B_reduced, Q_reduced,
                                     R, N)
          : DiscreteTimeLinearQuadraticRegulator(A_reduced,
                                                 B_reduced, Q_reduced, R);
  
  Eigen::MatrixXd K(num_inputs, num_states);
  K.leftCols(1) = Eigen::VectorXd::Zero(num_inputs);
  K.rightCols(num_states-1) = lqr_result.K;
  
  const Eigen::VectorXd& x0 =
      (linear_system->time_period() == 0.0)
          ? context.get_continuous_state_vector().CopyToVector()
          : context.get_discrete_state(0).CopyToVector();

  const auto& u0 = system.EvalEigenVectorInput(context, 0);
  std::cout << "u0 inside LQR: " << u0 << std::endl;
  
  std::cout << "K inside LQR: " << K << std::endl;

  // Return the affine controller: u = u0 - K(x-x0).
  return std::make_unique<systems::AffineSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // xDot0
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -K,                                    // D
      u0 + K * x0,                // y0
      linear_system->time_period());
}


}  // namespace controllers
}  // namespace systems
}  // namespace drake
