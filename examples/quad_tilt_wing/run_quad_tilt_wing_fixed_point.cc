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

DEFINE_double(target_realtime_rate, 0.1,
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

  const int kNumTimeSamples = 2;
  const double kMinimumTimeStep = 0.1;
  const double kMaximumTimeStep = 0.1;
  systems::trajectory_optimization::DirectCollocation dirtran(
      quad_tilt_wing_plant.get(), *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);


  // Add bounds to the inputs
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_0 = dirtran.input(0);
  const double kProp = quad_tilt_wing_plant -> kProp();
  const double UAV_fg = quad_tilt_wing_plant -> m() * quad_tilt_wing_plant -> g();
  const double kPropSpeedLowerLimit = 100;
  const double kPropSpeedUpperLimit = UAV_fg / kProp;
  std::cout << "M_PI: " << M_PI << std::endl;
  const double kTiltUpperLimit = 0.1;
  const double kTiltLowerLimit = - 0.1 * M_PI;
  dirtran.AddLinearConstraint(u_0(0) >= kPropSpeedLowerLimit);
  dirtran.AddLinearConstraint(u_0(0) <= kPropSpeedUpperLimit);
  dirtran.AddLinearConstraint(u_0(1) >= kPropSpeedLowerLimit);
  dirtran.AddLinearConstraint(u_0(1) <= kPropSpeedUpperLimit);
  dirtran.AddLinearConstraint(u_0(2) >= kPropSpeedLowerLimit);
  dirtran.AddLinearConstraint(u_0(2) <= kPropSpeedUpperLimit);
  dirtran.AddLinearConstraint(u_0(3) >= kPropSpeedLowerLimit);
  dirtran.AddLinearConstraint(u_0(3) <= kPropSpeedUpperLimit);
  dirtran.AddLinearConstraint(u_0(4) >= kTiltLowerLimit);
  dirtran.AddLinearConstraint(u_0(4) <= kTiltUpperLimit);
  dirtran.AddLinearConstraint(u_0(5) >= kTiltLowerLimit);
  dirtran.AddLinearConstraint(u_0(5) <= kTiltUpperLimit);
  dirtran.AddLinearConstraint(u_0(6) >= kTiltLowerLimit);
  dirtran.AddLinearConstraint(u_0(6) <= kTiltUpperLimit);
  dirtran.AddLinearConstraint(u_0(7) >= kTiltLowerLimit);
  dirtran.AddLinearConstraint(u_0(7) <= kTiltUpperLimit);
  dirtran.AddLinearConstraint(u_0(0) == u_0(1));
  dirtran.AddLinearConstraint(u_0(2) == u_0(3));
  dirtran.AddLinearConstraint(u_0(4) == u_0(5));
  dirtran.AddLinearConstraint(u_0(6) == u_0(7));
  // All inputs should be same
  for (int i = 1; i < kNumTimeSamples; i++) {
    dirtran.AddLinearConstraint(dirtran.input(i) == u_0);
  }
  // Add constraint to the states
  // Position constraints
  const double kXLowerLimit = -5.0;
  const double kXUpperLimit = 300.0;
  const double kY = 0.0;
  const double kZ = 10;
  const double kPhi = 0.0;
  const double kTheta = 0.0;
  const double kPsi = 0.0;
  // velocity constriants, very high bound
  const double kXDot = 50.0;
  const double kYDot = 0.0;
  const double kZDot = 0.0;
  const double kPhiDot = 0.0;
  const double kThetaDot = 0.0;
  const double kPsiDot = 0.0;
  const double kEpsilon = 1e-4;
  const solvers::VectorXDecisionVariable& x = dirtran.state();

  std::cout << "x state size: " << x.size() << std::endl;

  dirtran.AddConstraintToAllKnotPoints(x(0) >= kXLowerLimit); // limit X
  dirtran.AddConstraintToAllKnotPoints(x(0) <= kXUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(x(1) >= kY - kEpsilon); // limit Y
  dirtran.AddConstraintToAllKnotPoints(x(1) <= kY + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(2) >= kZ - kEpsilon); // limit Z
  dirtran.AddConstraintToAllKnotPoints(x(2) <= kZ + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(3) >= kPhi - kEpsilon); // limit phi
  dirtran.AddConstraintToAllKnotPoints(x(3) <= kPhi + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(4) >= kTheta - kEpsilon); // limit theta
  dirtran.AddConstraintToAllKnotPoints(x(4) <= kTheta + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(5) >= kPsi - kEpsilon); // limit psi
  dirtran.AddConstraintToAllKnotPoints(x(5) <= kPsi + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(6) >= kXDot - kEpsilon); // limit xdot
  dirtran.AddConstraintToAllKnotPoints(x(6) <= kXDot + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(7) >= kYDot - kEpsilon); // limit ydot
  dirtran.AddConstraintToAllKnotPoints(x(7) <= kYDot + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(8) >= kZDot - kEpsilon); // limit zdot
  dirtran.AddConstraintToAllKnotPoints(x(8) <= kZDot + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(9) >= kPhiDot - kEpsilon); // limit phidot
  dirtran.AddConstraintToAllKnotPoints(x(9) <= kPhiDot + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(10) >= kThetaDot - kEpsilon); // limit thetadot
  dirtran.AddConstraintToAllKnotPoints(x(10) <= kThetaDot + kEpsilon);
  dirtran.AddConstraintToAllKnotPoints(x(11) >= kPsiDot - kEpsilon); // limit psidot
  dirtran.AddConstraintToAllKnotPoints(x(11) <= kPsiDot + kEpsilon);

  // Add initial state constraint
  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(12);
  initial_state(2) = kZ; // Z_0, start at height 10m, all the other states are 0
  initial_state(6) = kXDot;
  dirtran.AddLinearConstraint(dirtran.initial_state()(0) == 0);

  std::cout << "initial state: " << initial_state << std::endl;

  // Add running cost
  const solvers::VectorXDecisionVariable& u = dirtran.input();
  const double R_prop = 1e-7;  // Cost on input "effort".
  for (int i = 0; i < 4; i++) {
      dirtran.AddRunningCost(R_prop * u(i) * u(i));
  }
  const double R_tilt = 1;  // Cost on input "effort".
  for (int i = 4; i < 8; i++) {
      dirtran.AddRunningCost(R_tilt * u(i) * u(i));
  }

  std::cout << "Finished adding all constraints and costs." << std::endl;

  // set initial guess
  const double timespan_init = (kNumTimeSamples - 1) * kMinimumTimeStep;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state, initial_state});
  Eigen::VectorXd initial_input_guess(8);
  initial_input_guess.topRows(4) = Eigen::VectorXd::Constant(4, UAV_fg/4);
  initial_input_guess.bottomRows(4) = Eigen::VectorXd::Constant(4, -M_PI/10.0);
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_input_guess, initial_input_guess});
  dirtran.SetInitialTrajectory(traj_init_u, traj_init_x);

  // Set solver options
  const double tol = 1e-6;
  solvers::IpoptSolver solver;
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "print_level", 3);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "print_user_options", "yes");
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "max_iter", 2000);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "constr_viol_tol", tol);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.constr_viol_tol", tol);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.constr_viol_tol", tol);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "acceptable_constr_viol_tol", tol);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.acceptable_constr_viol_tol", tol);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "acceptable_tol", tol);
  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "tol", tol);
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

  publisher->set_publish_period(1.0 / 60.0);
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

  systems::Simulator<double> simulator(*diagram);

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
