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

  const int kNumTimeSamples = 101;
  const double kMinimumTimeStep = 0.1;
  const double kMaximumTimeStep = 0.2;
  systems::trajectory_optimization::DirectCollocation dirtran(
      quad_tilt_wing_plant.get(), *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);


  // Add bounds to the inputs
//   const double kPI = 3.1415926;
  const solvers::VectorXDecisionVariable& u = dirtran.input();
  const double kProp = quad_tilt_wing_plant -> kProp();
  const double UAV_fg = quad_tilt_wing_plant -> m() * quad_tilt_wing_plant -> g();
  const double kPropSpeedLowerLimit = 100;
  const double kPropSpeedUpperLimit = UAV_fg / 2.0 / kProp;
  const double kTiltUpperLimit = 0.1;
  const double kTiltLowerLimit = - 0.51 * M_PI;
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
  const double kStateTol = 1e-6;
  const double kXLowerLimit = -5.0;
  const double kXUpperLimit = 2000.0;
  const double kYPosLimit = 200;
  const double kYNegLimit = -kStateTol;
  const double kZLowerLimit = -100-kStateTol;
  const double kZUpperLimit = 100.0+kStateTol;
  const double kPhiNegLimit = -M_PI/2.0;
  const double kPhiPosLimit = M_PI/4.0;
  const double kThetaNegLimit = -M_PI/2.0;
  const double kThetaPosLimit = M_PI/4.0;
  const double kPsiNegLimit = -M_PI/4.0;
  const double kPsiPosLimit = M_PI+kStateTol;
  // velocity constraints, very high bound
  const double kXDotLowerLimit = -120;
  const double kXDotUpperLimit = 120.0;
  const double kYDotLimit = 100;
  const double kZDotLimit = 5;
  const double kPhiDotLimit = M_PI/20;
  const double kThetaDotLimit = M_PI/20;
  const double kPsiDotLimit = M_PI/20;
  const solvers::VectorXDecisionVariable& x = dirtran.state();

  std::cout << "x state size: " << x.size() << std::endl;

  dirtran.AddConstraintToAllKnotPoints(x(0) >= kXLowerLimit); // limit X
  dirtran.AddConstraintToAllKnotPoints(x(0) <= kXUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(x(1) >= kYNegLimit); // limit Y
  dirtran.AddConstraintToAllKnotPoints(x(1) <= kYPosLimit);
  dirtran.AddConstraintToAllKnotPoints(x(2) >= kZLowerLimit); // limit Z
  dirtran.AddConstraintToAllKnotPoints(x(2) <= kZUpperLimit);
  dirtran.AddConstraintToAllKnotPoints(x(3) >= kPhiNegLimit); // limit phi
  dirtran.AddConstraintToAllKnotPoints(x(3) <= kPhiPosLimit);
  dirtran.AddConstraintToAllKnotPoints(x(4) >= kThetaNegLimit); // limit theta
  dirtran.AddConstraintToAllKnotPoints(x(4) <= kThetaPosLimit);
  dirtran.AddConstraintToAllKnotPoints(x(5) >= kPsiNegLimit); // limit psi
  dirtran.AddConstraintToAllKnotPoints(x(5) <= kPsiPosLimit);
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
  initial_state(2) = 10.0; // Z_0, start at height 10m
  initial_state(6) = 100.0; // start at 100m/s
  dirtran.AddLinearConstraint(dirtran.initial_state() == initial_state);

  std::cout << "initial state: " << initial_state << std::endl;

  // Add initial input constraint
  Eigen::VectorXd initial_input = Eigen::VectorXd::Zero(8);
  initial_input << 1970.68, 1970.68, 1970.54, 1970.54, -0.00707841, -0.00707841, -0.00249764, -0.00249764;
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_0 = dirtran.input(0);
  dirtran.AddLinearConstraint(u_0 == initial_input);

  // Add tilt-speed limit constraint
  const double kTiltSpeedLimit = M_PI / 30.0; // limit the tilting speed
  for (int i=1; i<(kNumTimeSamples); i++){
    dirtran.AddLinearConstraint(dirtran.input(i).segment(4,1) - dirtran.input(i-1).segment(4,1) >= -kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(4,1) - dirtran.input(i-1).segment(4,1) <= kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(6,1) - dirtran.input(i-1).segment(6,1) >= -kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(6,1) - dirtran.input(i-1).segment(6,1) <= kTiltSpeedLimit*dirtran.timestep(i-1));
  }

  // Add final state constraint
  Eigen::VectorXd final_state = Eigen::VectorXd::Zero(12);
  final_state(0) = 0;  // for initial guess
  final_state(1) = 100;  // for initial guess
  final_state(2) = 10; // Z_N, try keeping the same height at final state
  final_state(5) = M_PI; // yaw 180 degree to turn back
  final_state(6) = -100.0; // dot_X, aim for 10m/s speed
  dirtran.AddLinearConstraint(dirtran.final_state().tail(10) == final_state.tail(10));

  std::cout << "final state: " << final_state << std::endl;

  // Add final input constraint
  Eigen::VectorXd final_input(8);
  final_input << 1970.68, 1970.68, 1970.54, 1970.54, -0.00707841, -0.00707841, -0.00249764, -0.00249764;  // for 100m/s
  const double kSlack_u_N_prop = 5;
  const double kSlack_u_N_tilt = 1e-4;
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_N = dirtran.input(kNumTimeSamples-1);
  for (int i = 0; i < 4; i++) {
    dirtran.AddLinearConstraint(u_N(i) >= final_input(i) - kSlack_u_N_prop);
    dirtran.AddLinearConstraint(u_N(i) <= final_input(i) + kSlack_u_N_prop);
  }
  for (int i = 4; i < 8; i++) {
    dirtran.AddLinearConstraint(u_N(i) >= final_input(i) - kSlack_u_N_tilt);
    dirtran.AddLinearConstraint(u_N(i) <= final_input(i) + kSlack_u_N_tilt);
  }

  // Add mid point constraint
  // Roll and start to turn
  Eigen::Matrix<int, 1, 3> mid_point_times{(kNumTimeSamples-1)/4, (kNumTimeSamples-1)/4 *2 , (kNumTimeSamples-1)/4 *3};
  std::cout << "Mid point times: " << mid_point_times << std::endl;
  Eigen::VectorXd mid_state_1 = Eigen::VectorXd::Zero(12);
  mid_state_1(0) = 200; // X for initial guess
  mid_state_1(2) = 10; // Z
  const double kRollTurnMax = -M_PI/18.0;
  const double kRollTurnMin = -M_PI/2.0;
  mid_state_1(3) = (kRollTurnMax + kRollTurnMin)/2.0;
  mid_state_1(6) = 100; // X_dot for initial guess
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> x_mid_point_1 = dirtran.state(mid_point_times(0));
  dirtran.AddLinearConstraint(x_mid_point_1.segment(1,2) == mid_state_1.segment(1,2));
  dirtran.AddLinearConstraint(x_mid_point_1(3) >= kRollTurnMin);
  dirtran.AddLinearConstraint(x_mid_point_1(3) <= kRollTurnMax);
  dirtran.AddLinearConstraint(x_mid_point_1.segment(4,2) == mid_state_1.segment(4,2));
  dirtran.AddLinearConstraint(x_mid_point_1.segment(7,5) == mid_state_1.segment(7,5));
  // Turning point
  Eigen::VectorXd mid_state_2 = Eigen::VectorXd::Zero(12);
  mid_state_2(0) = 250; //X for initial guess
  mid_state_2(1) = 50; // Y for initial guess
  mid_state_2(2) = 10; // Z
  mid_state_2(3) = (kRollTurnMax + kRollTurnMin)/2.0;
  mid_state_2(5) = M_PI/2;
  mid_state_2(6) = 0; // X dot
  const double kTurnSpeedMin = 20.0;
  const double kTurnSpeedMax = 100.0;
  mid_state_2(7) = (kTurnSpeedMax + kTurnSpeedMin) / 2.0; // Y dot for initial guess
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> x_mid_point_2 = dirtran.state(mid_point_times(1));
  dirtran.AddLinearConstraint(x_mid_point_2.segment(2,1) == mid_state_2.segment(2,1));
  dirtran.AddLinearConstraint(x_mid_point_2(3) >= kRollTurnMin);
  dirtran.AddLinearConstraint(x_mid_point_2(3) <= kRollTurnMax);
  dirtran.AddLinearConstraint(x_mid_point_2.segment(4,2) == mid_state_2.segment(4,2));
  dirtran.AddLinearConstraint(x_mid_point_2(6) == mid_state_2(6));
  dirtran.AddLinearConstraint(x_mid_point_2(7) >= kTurnSpeedMin);
  dirtran.AddLinearConstraint(x_mid_point_2(7) <= kTurnSpeedMax);
  dirtran.AddLinearConstraint(x_mid_point_2.segment(8,4) == mid_state_2.segment(8,4));
  // Finish turning
  Eigen::VectorXd mid_state_3 = Eigen::VectorXd::Zero(12);
  mid_state_3(0) = 200; // X for initial guess
  mid_state_3(1) = 100; // Y for initial guess
  mid_state_3(2) = 10; // Z
  mid_state_3(3) = (kRollTurnMax + kRollTurnMin)/2.0;;
  mid_state_3(5) = M_PI;
  mid_state_3(6) = -100; // X dot for initial guess
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> x_mid_point_3 = dirtran.state(mid_point_times(2));
  dirtran.AddLinearConstraint(x_mid_point_3.segment(2,1) == mid_state_3.segment(2,1));
  dirtran.AddLinearConstraint(x_mid_point_3(3) >= kRollTurnMin);
  dirtran.AddLinearConstraint(x_mid_point_3(3) <= kRollTurnMax);
  dirtran.AddLinearConstraint(x_mid_point_3.segment(4,2) == mid_state_3.segment(4,2));
  dirtran.AddLinearConstraint(x_mid_point_3.segment(7,5) == mid_state_3.segment(7,5));

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
  const double timespan_init = (kNumTimeSamples - 1) * (kMinimumTimeStep + kMaximumTimeStep) / 2.0;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init/4.0, timespan_init/2.0, timespan_init/4.0*3.0, timespan_init},
              {initial_state, mid_state_1, mid_state_2, mid_state_3, final_state});
  auto initial_input_guess = initial_input;
  auto final_input_guess = final_input;
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_input_guess, final_input_guess});
  dirtran.SetInitialTrajectory(traj_init_u, traj_init_x);

  // Set solver options
//  const double tol = 1e-4;
  solvers::SnoptSolver solver;
  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Major print level", 2);
//  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Iterations limit", 1e4);
//  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Major iterations limit", 1e3);
//  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Minor iterations limit", 1e3);
  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Print file", "/home/klytech/solver_output/traj_opt_100mps_snopt_turn.txt");


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

  std::ofstream file("/home/klytech/solver_output/traj_opt_sol_100mps_snopt_turn_py.txt");

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
