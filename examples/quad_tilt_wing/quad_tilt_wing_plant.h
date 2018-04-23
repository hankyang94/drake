#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace quad_tilt_wing {

/// The Quad tilt-wing - an underactuated aerial vehicle. This version of the
/// quad tilt-wing is implemented to match the dynamics of the plant specified in
/// the `quad_tilt_wing.urdf` model file.
template <typename T>
class QuadTiltWingPlant final : public systems::LeafSystem<T> {                // QuadTiltWingPlant derives from systems::LeafSystem
 public:                                                                    // final means cannot override
  QuadTiltWingPlant();            // class constructor
  QuadTiltWingPlant(double m_arg, double rear_joint_x_arg,
                    double front_joint_x_arg, double rear_wing_len_arg, double rear_wing_wid_arg,
                    double front_wing_len_arg, double front_wing_wid_arg, double front_prop_y_arg,
                    double rear_prop_y_arg, const Eigen::Matrix3d& I_arg,
                    double kProp_arg, double kLambda_arg);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit QuadTiltWingPlant(const QuadTiltWingPlant<U>&);

  ~QuadTiltWingPlant() override;        // class destructor

  int get_input_size() const { return kInputDimension; }

  int get_num_states() const { return kStateDimension; }

  void set_state(systems::Context<T>* context, const VectorX<T>& x) const {
    context->get_mutable_continuous_state_vector().SetFromVector(x);
  }

  double m() const { return m_; }
  double g() const { return g_; }

 protected:
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;         // where is autodiff being used?

  /// Declares that the system has no direct feedthrough from any input to any
  /// output.
  ///
  /// The QuadTiltWingPlant is incompatible with the symbolic::Expression scalar
  /// type because it invokes the Cholesky LDLT decomposition, which uses
  /// conditionals in its implementation. Therefore, we must specify sparsity
  /// by hand.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class QuadTiltWingPlant;

  static constexpr int kStateDimension{12};
  static constexpr int kInputDimension{8};

  // TODO(naveenoid): Declare these as parameters in the context.
  const double rho_;         // density of the air 1.225kg/m^3
  const double g_;           // Gravitational acceleration (m/s^2).
  const double m_;           // Mass of the robot (kg).
  const double rear_joint_x_; // x coordinate of the rear rotational joint
  const double front_joint_x_; // x coordinate of the front rotational joing
  const double rear_wing_len_; // rear wing length
  const double rear_wing_wid_; // rear wing width
  const double front_wing_len_; // front wing length
  const double front_wing_wid_; // front wing width
  const double front_prop_y_; // front prop y-directional distance
  const double rear_prop_y_;  // rear prop y-directional distance
  //~ const double L_;           // Length of the arms (m).
  //~ const double kF_;          // Force input constant.
  //~ const double kM_;          // Moment input constant.
  const double kProp_;         // thrust = kProp_*omega^2
  const double kLambda_;       // Torque-to-thrust ratio
  const Eigen::Matrix3d I_;  // Moment of Inertia about the Center of Mass
};

/// Generates an LQR controller to move to @p nominal_position. Internally
/// computes the nominal input corresponding to a hover at position @p x0.
/// @see systems::LinearQuadraticRegulator.
std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadTiltWingPlant<double>* quad_tilt_wing_plant,
    Eigen::Vector3d nominal_position);

}  // namespace quad_tilt_wing
}  // namespace examples
}  // namespace drake
