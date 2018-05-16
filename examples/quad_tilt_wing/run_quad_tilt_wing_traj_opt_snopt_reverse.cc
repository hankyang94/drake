#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <math.h>
#include <fstream>
#include <limits>

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
#include "drake/solvers/snopt_solver.h"

using drake::solvers::SolutionResult;

namespace drake {
namespace examples {
namespace quad_tilt_wing {

using trajectories::PiecewisePolynomial;
using namespace Eigen;

namespace {

DEFINE_double(target_realtime_rate, 1,
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

  const int kNumTimeSamples = 61;
  const double kMinimumTimeStep = 0.4;
  const double kMaximumTimeStep = 0.8;
  systems::trajectory_optimization::DirectCollocation dirtran(
      quad_tilt_wing_plant.get(), *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);


  // Add bounds to the inputs
//   const double kPI = 3.1415926;
  const solvers::VectorXDecisionVariable& u = dirtran.input();
  const double UAV_fg = quad_tilt_wing_plant -> m() * quad_tilt_wing_plant -> g();
  const double kThrustLowerLimit = 0.0;
  const double kThrustUpperLimit = UAV_fg / 2.0;
  std::cout << "M_PI: " << M_PI << std::endl;
  const double kTiltUpperLimit = 0.1;
  const double kTiltLowerLimit = - 0.51 * M_PI;
  dirtran.AddConstraintToAllKnotPoints(u(0) >= kThrustLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(0) <= kThrustUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(1) >= kThrustLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(1) <= kThrustUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(2) >= kThrustLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(2) <= kThrustUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(3) >= kThrustLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(3) <= kThrustUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(4) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(4) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(5) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(5) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(6) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(6) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(7) >= kTiltLowerLimit);
  dirtran.AddConstraintToAllKnotPoints(u(7) <= kTiltUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(u(0) == u(1));
  dirtran.AddConstraintToAllKnotPoints(u(2) == u(3));
  dirtran.AddConstraintToAllKnotPoints(u(4) == u(5));
  dirtran.AddConstraintToAllKnotPoints(u(6) == u(7));

  // Add constraint to the states
  // Position constraints
  const double kXLowerLimit = -5.0;
  const double kXUpperLimit = 2000.0;
  const double kYLimit = 1e-6;
  const double kZLowerLimit = 10.0-1000;
  const double kZUpperLimit = 10.0+1000;
  const double kPhiLimit = 1e-6;
  const double kThetaLimit = 0.1;
  const double kPsiLimit = 1e-6;
  // velocity constriants, very high bound
  const double kXDotLowerLimit = -5.0;
  const double kXDotUpperLimit = 120.0;
  const double kYDotLimit = 1e-6;
  const double kZDotLimit = 2;
  const double kPhiDotLimit = 1e-6;
  const double kThetaDotLimit = 0.1;
  const double kPsiDotLimit = 1e-6;
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
  initial_state(0) = 0;  // for initial guess
  initial_state(2) = 10; // Z_N, try keeping the same height at final state
  initial_state(6) = 100.0; // dot_X, aim for 100m/s speed
  dirtran.AddLinearConstraint(dirtran.initial_state() == initial_state);
  std::cout << "initial state: " << initial_state << std::endl;


  // Add initial input constraint
  Eigen::VectorXd initial_input(8);
//   final_input << 8297.58, 8297.58, 8288.65, 8288.6, -0.0283086, -0.0283086, -0.00999118, -0.00999118;  // for 50m/s
  initial_input << 0.0104247, 0.0104247, 0.010424, 0.010424, -0.00707774, -0.00707774, -0.00249736, -0.00249737;  // for 100m/s
  const double kSlack_u_N_prop = 1e-4;
  const double kSlack_u_N_tilt = 1e-4;
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_0 = dirtran.input(0);
  for (int i = 0; i < 4; i++) {
    dirtran.AddLinearConstraint(u_0(i) >= initial_input(i) - kSlack_u_N_prop);
    dirtran.AddLinearConstraint(u_0(i) <= initial_input(i) + kSlack_u_N_prop);
  }
  for (int i = 4; i < 8; i++) {
    dirtran.AddLinearConstraint(u_0(i) >= initial_input(i) - kSlack_u_N_tilt);
    dirtran.AddLinearConstraint(u_0(i) <= initial_input(i) + kSlack_u_N_tilt);
  }


  // Add final state constraint
  Eigen::VectorXd final_state = Eigen::VectorXd::Zero(12);
  final_state(0) = 2000; // for initial guess
  final_state(2) = 100.0; // Z_0, start at height 10m, all the other states are 0
  dirtran.AddLinearConstraint(dirtran.final_state().tail(9) == final_state.tail(9));

  std::cout << "final state: " << final_state << std::endl;

  // Add final input constraint
  const double final_tilt_angle = -M_PI/2.0; // final configuration is a quadrotor
  const double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
  const double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
  const double front_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * rear_moment_arm / 2.0;
  const double rear_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * front_moment_arm / 2.0;
  Eigen::Vector4d uN_prop{front_prop_f, front_prop_f, rear_prop_f, rear_prop_f};
  Eigen::VectorXd final_input = Eigen::VectorXd::Zero(8);
  final_input.topRows(4) = uN_prop;
  final_input.bottomRows(4) = Eigen::VectorXd::Constant(4, final_tilt_angle);
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_N = dirtran.input(kNumTimeSamples - 1);
  dirtran.AddLinearConstraint(u_N == final_input);

  // Add tilt-speed limit constraint
  const double kTiltSpeedLimit = M_PI / 30.0; // limit the tilting speed
  for (int i=1; i<(kNumTimeSamples); i++){
    dirtran.AddLinearConstraint(dirtran.input(i).segment(4,1) - dirtran.input(i-1).segment(4,1) >= -kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(4,1) - dirtran.input(i-1).segment(4,1) <= kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(6,1) - dirtran.input(i-1).segment(6,1) >= -kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(6,1) - dirtran.input(i-1).segment(6,1) <= kTiltSpeedLimit*dirtran.timestep(i-1));
  }


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
  auto final_input_guess = final_input;
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_input_guess, final_input_guess});
  dirtran.SetInitialTrajectory(traj_init_u, traj_init_x);

  // Set solver options
  solvers::SnoptSolver solver;
  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Major print level", 4);
  dirtran.SetSolverOption(solvers::SnoptSolver::id(),
                          "Print file", "/Users/Hank/solver_output/transform_100mps_reverse_snopt.txt");
//  dirtran.SetSolverOption(solvers::SnoptSolver::id(),
//                          "Print file", "/home/klytech/solver_output/traj_opt_100mps_reverse_snopt.txt");
//  solvers::IpoptSolver solver;
//  const double tol = 1e-4;
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "print_level", 5);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "print_user_options", "yes");
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "max_iter", 1000);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "constr_viol_tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.constr_viol_tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.constr_viol_tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "acceptable_constr_viol_tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "resto.acceptable_constr_viol_tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "acceptable_tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "tol", tol);
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "output_file", "/home/klytech/solver_output/traj_opt_100mps.txt");
  //~ dirtran.SetSolverOption(solvers::IpoptSolver::id(), "output_file", "/home/hank/solver_output/traj_opt_100mps.txt");
//  dirtran.SetSolverOption(solvers::IpoptSolver::id(), "output_file", "/Users/Hank/solver_output/traj_opt_ipopt_100mps.txt");

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

//  std::ofstream file("/home/klytech/solver_output/traj_opt_sol_100mps_snopt_reverse_py.txt");
  std::ofstream file("/Users/Hank/solver_output/transform_sol_100mps_snopt_reverse_py.txt");
//  std::ofstream file("/home/hank/solver_output/traj_opt_sol_100mps_py.txt");
//  std::ofstream file(
//          "/Users/Hank/Dropbox (MIT)/Courses/6.832 Underactuated Robotics/Final Project/code/solver_output/traj_opt_sol_100mps_py");
  if (file.is_open()) {
    file << "t samples: " << '\n';
    file << t_samples << '\n';
    file << "u_samples: " << '\n';
    file << u_samples << '\n';
    file << "x_samples: " << '\n';
    file << x_samples << '\n';
  }



  const PiecewisePolynomial<double> pp_utraj =
      dirtran.ReconstructInputTrajectory();
  const PiecewisePolynomial<double> pp_xtraj =
      dirtran.ReconstructStateTrajectory();

  std::cout << "Retrived solution from solver. " << std::endl;

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
//   builder.Connect(input_trajectory->get_output_port(), quad_tilt_wing_plant->get_input_port(0))

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
  simulator.StepTo(0.01);
  std::cout << "Pausing for 30 seconds, tune the visualizer please." << std::endl;

  sleep(15);

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
