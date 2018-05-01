/// @file
///
/// This demo sets up a controlled Quadrotor that uses a Linear Quadratic
/// Regulator to (locally) stabilize a nominal hover.

#include <memory>
#include <iostream>
#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/quad_tilt_wing/quad_tilt_wing_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/demultiplexer.h"

DEFINE_int32(simulation_trials, 1, "Number of trials to simulate.");
DEFINE_double(simulation_real_time_rate, 1, "Real time rate");
DEFINE_double(trial_duration, 10, "Duration of execution of each trial");

namespace drake {
using systems::DiagramBuilder;
using systems::Simulator;
using systems::Context;
using systems::ContinuousState;
using systems::VectorBase;
using std::make_unique;

namespace examples {
namespace quad_tilt_wing {
namespace {

int do_main() {
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/quad_tilt_wing/quad_tilt_wing.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());
  std::cout << "Finished reading urdf." << std::endl;

  // The nominal hover position is at (0, 0, 1.0) in world coordinates.
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

  std::cout << "kNominalPosition: " << kNominalPosition << std::endl;

  auto quad_tilt_wing_plant = builder.AddSystem<QuadTiltWingPlant<double>>();
  quad_tilt_wing_plant->set_name("quad_tilt_wing_plant");

  std::cout << "Added quad tilt-wing plant." << std::endl;

  auto controller = builder.AddSystem(StabilizingLQRController(
    quad_tilt_wing_plant, kNominalPosition));
  controller->set_name("controller");

  std::cout << "Added LQR controller." << std::endl;

  auto visualizer =
      builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);
  visualizer->set_name("visualizer");

  std::cout << "Added Visualizer." << std::endl;

  auto plant_demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(12, 6);
  plant_demux->set_name("plant_demux");
  auto controller_demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(8, 4);
  controller_demux->set_name("controller_demux");
  auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(std::vector<int> {6,4,6,4});
  mux->set_name("mux");

  //// check system input and output port sizes
  std::cout << "Plant output size: " << quad_tilt_wing_plant->get_output_port(0).size() << std::endl;
  std::cout << "Controller input size: " << controller->get_input_port().size() << std::endl;
  std::cout << "Controller output size: " << controller->get_output_port().size() << std::endl;
  std::cout << "Plant input size: " << quad_tilt_wing_plant->get_input_port(0).size() << std::endl;
  std::cout << "Plant output size: " << quad_tilt_wing_plant->get_output_port(0).size() << std::endl;
  std::cout << "Visualizer input size: " << visualizer->get_input_port(0).size() << std::endl;
  std::cout << "mux output size: " << mux->get_output_port(0).size() << std::endl;
  // connect dynamics ports
  builder.Connect(quad_tilt_wing_plant->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quad_tilt_wing_plant->get_input_port(0));
  // demux plant into 6 and 6, first 6 is postion, second 6 is velocity
  builder.Connect(quad_tilt_wing_plant->get_output_port(0), plant_demux->get_input_port(0));
  // demux controller into 4 and 4, first 4 is prop speed^2, second 4 is tilting angle
  builder.Connect(controller->get_output_port(), controller_demux->get_input_port(0));
  // mux position, tilting angle, speed, ~ together, the fourth port of mux is not connected, because we dont care
  builder.Connect(plant_demux->get_output_port(0), mux->get_input_port(0));
  builder.Connect(controller_demux->get_output_port(1), mux->get_input_port(1));
  builder.Connect(plant_demux->get_output_port(1), mux->get_input_port(2));
  builder.Connect(controller_demux->get_output_port(1), mux->get_input_port(3));
  // connect mux to visualizer
  builder.Connect(mux->get_output_port(0), visualizer->get_input_port(0));

  auto diagram = builder.Build();

  std::cout << "System Built" << std::endl;

  Simulator<double> simulator(*diagram);
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0(2) = 0.5;  //Z initial height
  x0(3) = M_PI/6;
  x0(4) = M_PI/6;
  x0(5) = M_PI/6;
  x0(6) = 0.5;  // dot_x x direction speed
  x0(8) = 0;   // dot_z z direction speed

  const VectorX<double> kNominalState{((Eigen::VectorXd(12) << kNominalPosition,
  Eigen::VectorXd::Zero(9)).finished())};

  //~ srand(42);

  for (int i = 0; i < FLAGS_simulation_trials; i++) {
    auto diagram_context = diagram->CreateDefaultContext();
    //~ x0 = VectorX<double>::Random(12);

    std::cout << "x0 size:" << x0.size() <<std::endl;
    std::cout << "Continuous state vector size:" << simulator.get_mutable_context().get_mutable_continuous_state_vector().size() <<std::endl;

    //~ auto state_vector = simulator.get_mutable_context().get_mutable_continuous_state_vector().CopyToVector();

    //~ std::cout << "Continuous state vector:" << state_vector << std::endl;

    simulator.get_mutable_context()
        .get_mutable_continuous_state_vector()
        .SetFromVector(x0);

    simulator.Initialize();
    simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);
    simulator.StepTo(FLAGS_trial_duration);

    //~ // Goal state verification.
    //~ const Context<double>& context = simulator.get_context();
    //~ const ContinuousState<double>& state = context.get_continuous_state();
    //~ const VectorX<double>& position_vector = state.CopyToVector();

    //~ if (!is_approx_equal_abstol(
        //~ position_vector, kNominalState, 1e-4)) {
      //~ throw std::runtime_error("Target state is not achieved.");
    //~ }

    simulator.reset_context(std::move(diagram_context));
  }
  return 0;
}

}  // namespace
}  // namespace quad_tilt_wing
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quad_tilt_wing::do_main();
}
