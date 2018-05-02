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
  
  auto quad_tilt_wing_plant = builder.AddSystem<QuadTiltWingPlant<double>>();
  //~ auto quad_tilt_wing_plant = std::make_unique<PendulumPlant<double>>();
  quad_tilt_wing_plant->set_name("quad_tilt_wing_plant");
  
  auto context = quad_tilt_wing_plant->CreateDefaultContext();

  const int kNumTimeSamples = 51;
  const double kMinimumTimeStep = 0.1;
  const double kMaximumTimeStep = 0.5;
  systems::trajectory_optimization::DirectCollocation dirtran(
      quad_tilt_wing_plant, *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);
  
  //~ dirtran.AddTimeIntervalBounds(kMinimumTimeStep, kMaximumTimeStep);
  
  // Add constraint that all tilt angles should be between -3/4*M_PI to M_PI/4 
  const double kTiltUpperLimit = 0;
  const double kTiltLowerLimit = -3/4*M_PI;
  const solvers::VectorXDecisionVariable& u = dirtran.input();
  for (int i = 4; i < 8; i++) {
      dirtran.AddConstraintToAllKnotPoints(kTiltLowerLimit <= u(i));
      dirtran.AddConstraintToAllKnotPoints(kTiltUpperLimit >= u(i));
  }
  
  // Add constraint that all prop speed^2 should be within limit
  const double kPropSpeedLowerLimit = 100;
  const double kProp = quad_tilt_wing_plant -> kProp();
  const double UAV_fg = quad_tilt_wing_plant -> m() * quad_tilt_wing_plant -> g();
  const double kPropSpeedUpperLimit = UAV_fg / kProp; 
  
  std::cout << "Prop speed upperlimit: " << kPropSpeedUpperLimit << std::endl;
  
  for (int i = 0; i < 4; i++) {
      dirtran.AddConstraintToAllKnotPoints(u(i) >= kPropSpeedLowerLimit);
      dirtran.AddConstraintToAllKnotPoints(u(i) <= kPropSpeedUpperLimit);
  }
  
  // Add constraint to the states
  // Position constraints
  const double kZLowerLimit = -10;
  const double kZUpperLimit = 50;
  const double kYLimit = 10;
  const double kXLowerLimit = 0;
  const double kXUpperLimit = 300;
  const double kPhiLimit = M_PI/6;
  const double kThetaLimit = M_PI/6;
  const double kPsiLimit = M_PI/6;
  // velocity constriants, very high bound
  const double kXDotLowerLimit = 0;
  const double kXDotUpperLimit = 100;
  const double kYDotLimit = 50;
  const double kZDotLimit = 50;
  const double kPhiDotLimit = M_PI;
  const double kThetaDotLimit = M_PI;
  const double kPsiDotLimit = M_PI;
  const solvers::VectorXDecisionVariable& x = dirtran.state();
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
  initial_state(2) = 10; // Z_0, start at height 10m, all the other states are 0
  dirtran.AddLinearConstraint(dirtran.initial_state() == initial_state);
  
  std::cout << "initial state: " << initial_state << std::endl;
  
  // Add initial input constraint
  const double initial_tilt_angle = -M_PI/2; // initial configuration is a quadrotor
  Eigen::VectorXd initial_input = Eigen::VectorXd::Zero(8);
  initial_input.bottomRows(4) = Eigen::VectorXd::Constant(4, initial_tilt_angle);
  const solvers::VectorXDecisionVariable& u_0 = dirtran.input(0);
  dirtran.AddLinearConstraint(u_0.bottomRows(4) == initial_input.bottomRows(4));
  
  // Add final input constraint
  const double kFinalTiltLowerLimit = -M_PI/2;
  const double kFinalTiltUpperLimit = 0;
  const solvers::VectorXDecisionVariable& u_N = dirtran.input(kNumTimeSamples-1);
  for (int i=4; i<8; i++) {
      dirtran.AddLinearConstraint(u_N(i) >= kFinalTiltLowerLimit);
      dirtran.AddLinearConstraint(u_N(i) <= kFinalTiltUpperLimit);
  }
  
  //~ // Add final state constraint
  //~ Eigen::VectorXd final_state = Eigen::VectorXd::Zero(12);
  //~ final_state(2) = 10; // Z_N, try keeping the same height at final state
  //~ final_state(6) = 10; // dot_X, aim for 50m/s speed
  //~ dirtran.AddLinearConstraint(
    //~ dirtran.final_state().bottomRows(9) == final_state.bottomRows(9)); // contrain the attitue and velocity of the airplane
  
  // Add final state cost
  // Expected final state
  Eigen::VectorXd final_state = Eigen::VectorXd::Zero(12);
  final_state(0) = 20;  // for initial guess
  final_state(2) = 10; // Z_N, try keeping the same height at final state
  final_state(6) = 10; // dot_X, aim for 10m/s speed
  dirtran.AddFinalCost(x(1) * x(1)); // Y close to 0
  dirtran.AddFinalCost((x(2) - final_state(2)) * (x(2) - final_state(2))); // Z close to 10
  dirtran.AddFinalCost(x(3) * x(3)); // Phi close to 0
  dirtran.AddFinalCost(x(4) * x(4)); // Theta close to 0
  dirtran.AddFinalCost(x(5) * x(5)); // Psi close to 0
  dirtran.AddFinalCost((x(6) - final_state(6)) * (x(6) - final_state(6))); // xdot close to 10
  dirtran.AddFinalCost(x(7) * x(7)); // ydot
  dirtran.AddFinalCost(x(8) * x(8)); // zdot
  dirtran.AddFinalCost(x(9) * x(9)); // Phidot close to 0
  dirtran.AddFinalCost(x(10) * x(10)); // Thetadot close to 0
  dirtran.AddFinalCost(x(11) * x(11)); // Psidot close to 0
  
  std::cout << "final state: " << final_state << std::endl;
  
  // Add running cost
  const double R = 1;  // Cost on input "effort".
  for (int i = 0; i < 4; i++) {
      dirtran.AddRunningCost(R*u(i)*u(i));
  }
  
  std::cout << "Finished adding all constraints and costs." << std::endl;
  
  // set initial guess
  const double timespan_init = 6;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state, final_state});
  dirtran.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  SolutionResult result = dirtran.Solve();
  
  std::cout << "Done solving." << std::endl;
  
  if (result != SolutionResult::kSolutionFound) {
    std::cout << "Failed to solve optimization for the transformation."
              << std::endl;
  }
  
  // Get solution
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
