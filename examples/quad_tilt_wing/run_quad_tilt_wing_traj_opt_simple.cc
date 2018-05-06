#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/examples/quad_tilt_wing/quad_tilt_wing_plant.h"
#include "drake/solvers/ipopt_solver.h"

using drake::solvers::SolutionResult;

namespace drake {
namespace examples {
namespace quad_tilt_wing {

using trajectories::PiecewisePolynomial;
using namespace Eigen;

namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/quad_tilt_wing/quad_tilt_wing.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());
  std::cout << "Finished reading urdf." << std::endl;

//   auto quad_tilt_wing_plant = builder.AddSystem<QuadTiltWingPlant<double>>();
  auto quad_tilt_wing_plant = std::make_unique<QuadTiltWingPlant<double>>();
  quad_tilt_wing_plant->set_name("quad_tilt_wing_plant");

  auto context = quad_tilt_wing_plant.get()->CreateDefaultContext();

  const int kNumTimeSamples = 11;
  const double kMinimumTimeStep = 0.2;
  const double kMaximumTimeStep = 0.2;
  systems::trajectory_optimization::DirectCollocation dirtran(
      quad_tilt_wing_plant.get(), *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);


  // Add bounds to the inputs
//   const double kPI = 3.1415926;
  const solvers::VectorXDecisionVariable& u = dirtran.input();
  const double kProp = quad_tilt_wing_plant -> kProp();
  const double UAV_fg = quad_tilt_wing_plant -> m() * quad_tilt_wing_plant -> g();
  const double kPropSpeedLowerLimit = 100;
  const double kPropSpeedUpperLimit = UAV_fg / kProp;
  std::cout << "M_PI: " << M_PI << std::endl;
  const double kTiltUpperLimit = 0.0;
  const double kTiltLowerLimit = - 0.75 * M_PI;
  dirtran.AddConstraintToAllKnotPoints(u(0) >= kPropSpeedLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(0) <= kPropSpeedUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(1) >= kPropSpeedLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(1) <= kPropSpeedUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(2) >= kPropSpeedLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(2) <= kPropSpeedUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(3) >= kPropSpeedLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(3) <= kPropSpeedUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(4) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(4) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(5) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(5) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(6) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(6) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(7) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(7) <= kTiltUpperLimit);

  // Add constraint to the states
  // Position constraints
  const double kXLowerLimit = -5.0;
  const double kXUpperLimit = 300.0;
  const double kYLimit = 10.0;
  const double kZLowerLimit = -10.0;
  const double kZUpperLimit = 50.0;
  const double kPhiLimit = M_PI/6.0;
  const double kThetaLimit = M_PI/6.0;
  const double kPsiLimit = M_PI/6.0;
  // velocity constriants, very high bound
  const double kXDotLowerLimit = -5.0;
  const double kXDotUpperLimit = 100.0;
  const double kYDotLimit = 50.0;
  const double kZDotLimit = 50.0;
  const double kPhiDotLimit = M_PI;
  const double kThetaDotLimit = M_PI;
  const double kPsiDotLimit = M_PI;
  const solvers::VectorXDecisionVariable& x = dirtran.state();

  std::cout << "x state size: " << x.size() << std::endl;

  dirtran.AddConstraintToAllKnotPoints(x(0) >= kXLowerLimit); // limit X
  dirtran.AddConstraintToAllKnotPoints(x(0) <= kXUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(x(1) >= -kYLimit); // limit Y
  dirtran.AddConstraintToAllKnotPoints(x(1) <= kYLimit);
  dirtran.AddConstraintToAllKnotPoints(x(2) >= kZLowerLimit); // limit Z
  dirtran.AddConstraintToAllKnotPoints(x(2) <= kZUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(x(3) >= -kPhiLimit); // limit phi
  dirtran.AddConstraintToAllKnotPoints(x(3) <= kPhiLimit);
  dirtran.AddConstraintToAllKnotPoints(x(4) >= -kThetaLimit); // limit theta
  dirtran.AddConstraintToAllKnotPoints(x(4) <= kThetaLimit);
  dirtran.AddConstraintToAllKnotPoints(x(5) >= -kPsiLimit); // limit psi
  dirtran.AddConstraintToAllKnotPoints(x(5) <= kPsiLimit);
  dirtran.AddConstraintToAllKnotPoints(x(6) >= kXDotLowerLimit); // limit xdot
  dirtran.AddConstraintToAllKnotPoints(x(6) <= kXDotUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(x(7) >= -kYDotLimit); // limit ydot
  dirtran.AddConstraintToAllKnotPoints(x(7) <= kYDotLimit);
  dirtran.AddConstraintToAllKnotPoints(x(8) >= -kZDotLimit); // limit zdot
  dirtran.AddConstraintToAllKnotPoints(x(8) <= kZDotLimit);
  dirtran.AddConstraintToAllKnotPoints(x(9) >= -kPhiDotLimit); // limit phidot
  dirtran.AddConstraintToAllKnotPoints(x(9) <= kPhiDotLimit);
  dirtran.AddConstraintToAllKnotPoints(x(10) >= -kThetaDotLimit); // limit thetadot
  dirtran.AddConstraintToAllKnotPoints(x(10) <= kThetaDotLimit);
  dirtran.AddConstraintToAllKnotPoints(x(11) >= -kPsiDotLimit); // limit psidot
  dirtran.AddConstraintToAllKnotPoints(x(11) <= kPsiDotLimit);

  // Add initial state constraint
  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(12);
  initial_state(2) = 10.0; // Z_0, start at height 10m, all the other states are 0
  dirtran.AddLinearConstraint(dirtran.initial_state() == initial_state);

  std::cout << "initial state: " << initial_state << std::endl;

  // Add initial input constraint
  const double initial_tilt_angle = -M_PI/2.0; // initial configuration is a quadrotor
  const double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
  const double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
  const double front_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * rear_moment_arm / 2.0;
  const double rear_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * front_moment_arm / 2.0;
  Eigen::Vector4d u0_prop{front_prop_f/kProp, front_prop_f/kProp,
      rear_prop_f/kProp, rear_prop_f/kProp};
  Eigen::VectorXd initial_input = Eigen::VectorXd::Zero(8);
  initial_input.topRows(4) = u0_prop;
  initial_input.bottomRows(4) = Eigen::VectorXd::Constant(4, initial_tilt_angle);
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_0 = dirtran.input(0);
  dirtran.AddLinearConstraint(u_0 == initial_input);


  // Add final state cost
  // Expected final state
  Eigen::VectorXd final_state = Eigen::VectorXd::Zero(12);
  final_state(0) = 0.0;  // for initial guess
  final_state(2) = 15.0; // Z_N, try keeping the same height at final state
  final_state(6) = 0.0; // dot_X, aim for 10m/s speed
  dirtran.AddLinearConstraint(dirtran.final_state() == final_state);


 //  const double R_f_state = 100;
//   dirtran.AddFinalCost(R_f_state * (x(0) - final_state(0)) * (x(0) - final_state(0))); // X close to 0
//   dirtran.AddFinalCost(R_f_state * (x(1) - final_state(1)) * (x(1) - final_state(1))); // Y close to 0
//   dirtran.AddFinalCost(R_f_state * (x(2) - final_state(2)) * (x(2) - final_state(2))); // Z close to 10
//   dirtran.AddFinalCost(R_f_state * (x(3) - final_state(3)) * (x(3) - final_state(3))); // Phi close to 0
//   dirtran.AddFinalCost(R_f_state * (x(4) - final_state(4)) * (x(4) - final_state(4))); // Theta close to 0
//   dirtran.AddFinalCost(R_f_state * (x(5) - final_state(5)) * (x(5) - final_state(5))); // Psi close to 0
//   dirtran.AddFinalCost(R_f_state * (x(6) - final_state(6)) * (x(6) - final_state(6))); // xdot close to 10
//   dirtran.AddFinalCost(R_f_state * (x(7) - final_state(7)) * (x(7) - final_state(7))); // ydot
//   dirtran.AddFinalCost(R_f_state * (x(8) - final_state(8)) * (x(8) - final_state(8))); // zdot
//   dirtran.AddFinalCost(R_f_state * (x(9) - final_state(9)) * (x(9) - final_state(9))); // Phidot close to 0
//   dirtran.AddFinalCost(R_f_state * (x(10) - final_state(10)) * (x(10) - final_state(10))); // Thetadot close to 0
//   dirtran.AddFinalCost(R_f_state * (x(11) - final_state(11)) * (x(11) - final_state(11))); // Psidot close to 0

  std::cout << "final state: " << final_state << std::endl;

  // Add running cost
  const double R_prop = 1e-6;  // Cost on input "effort".
  for (int i = 0; i < 4; i++) {
      dirtran.AddRunningCost(R_prop * u(i) * u(i));
  }
  const double R_tilt = 0;  // Cost on input "effort".
  for (int i = 4; i < 8; i++) {
      dirtran.AddRunningCost(R_tilt * u(i) * u(i));
  }

  std::cout << "Finished adding all constraints and costs." << std::endl;

  // set initial guess
  const double timespan_init = (kNumTimeSamples - 1) * kMinimumTimeStep;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state, final_state});
  auto initial_input_guess = initial_input;
  Eigen::VectorXd final_input_guess(8);
  final_input_guess.topRows(4) = initial_input_guess.topRows(4);
  final_input_guess.bottomRows(4) = initial_input_guess.bottomRows(4);
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_input_guess, final_input_guess});
  dirtran.SetInitialTrajectory(traj_init_u, traj_init_x);

  // Set solver options
  solvers::IpoptSolver solver;
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "print_level", 2);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "print_user_options", "yes");
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "max_iter", 1000);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "constr_viol_tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.constr_viol_tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.constr_viol_tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "acceptable_constr_viol_tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.acceptable_constr_viol_tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "acceptable_tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "tol", 1e-6);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "output_file", "/Users/Hank/solver_output/output_3.txt");

  SolutionResult result = solver.Solve(dirtran);

  std::cout << "Done solving." << std::endl;

  if (result != SolutionResult::kSolutionFound) {
    std::cout << "Failed to solve optimization for the transformation."
              << std::endl;
  }

  // Get solution

  auto t_samples = dirtran.GetSampleTimes();
  auto u_samples = dirtran.GetInputSamples();
  auto x_samples = dirtran.GetStateSamples();

  std::cout << "t_samples: " << t_samples << std::endl;
  std::cout << "u_samples: " << u_samples << std::endl;
  std::cout << "x_samples: " << x_samples << std::endl;

  const PiecewisePolynomial<double> pp_utraj =
      dirtran.ReconstructInputTrajectory();
  const PiecewisePolynomial<double> pp_xtraj =
      dirtran.ReconstructStateTrajectory();

  std::cout << "Retrived solution from solver. " << std::endl;

  // std::cout << "pp_utraj: " << pp_utraj << std::endl;
//   std::cout << "pp_xtraj: " << pp_xtraj << std::endl;

  auto input_trajectory = builder.AddSystem<systems::TrajectorySource>(pp_utraj);
  input_trajectory->set_name("input trajectory");
  auto state_trajectory = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  state_trajectory->set_name("state trajectory");

  // Setup mux and demux for plant, controller, and publiser
  auto plant_demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(12, 6);
  plant_demux->set_name("plant_demux");
  auto controller_demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(8, 4);
  controller_demux->set_name("controller_demux");
  auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(std::vector<int> {6,4,6,4});
  mux->set_name("mux");
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  // Connect solution to visualizer

  // connect dynamics ports
//   builder.Connect(quad_tilt_wing_plant->get_output_port(0), controller->get_input_port());
//   builder.Connect(input_trajectory->get_output_port(), quad_tilt_wing_plant->get_input_port(0));

  builder.Connect(state_trajectory->get_output_port(), plant_demux->get_input_port(0));
  builder.Connect(input_trajectory->get_output_port(), controller_demux->get_input_port(0));
  builder.Connect(plant_demux->get_output_port(0), mux->get_input_port(0));
  builder.Connect(controller_demux->get_output_port(1), mux->get_input_port(1));
  builder.Connect(plant_demux->get_output_port(1), mux->get_input_port(2));
  builder.Connect(controller_demux->get_output_port(1), mux->get_input_port(3));
  builder.Connect(mux->get_output_port(0), publisher->get_input_port(0));

  auto diagram = builder.Build();

  std::cout << "Diagram built." << std::endl;


  Eigen::VectorXd initial_publisher(20);
  initial_publisher << initial_state.head(6), initial_input.tail(4), initial_state.tail(6), initial_input.tail(4);
  systems::Simulator<double> simulator(*diagram);
  std::cout << "size: " << simulator.get_mutable_context().get_mutable_continuous_state_vector().size() << std::endl;
  simulator.get_mutable_context().get_mutable_continuous_state_vector().SetFromVector(initial_publisher);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(pp_xtraj.end_time());

  std::cout << "Simulation Done." << std::endl;

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quad_tilt_wing::DoMain();
}
