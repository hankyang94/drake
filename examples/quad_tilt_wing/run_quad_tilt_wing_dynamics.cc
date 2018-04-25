/// @file
///
/// This demo sets up a passive quad tilt-wing plant in a world described by the
/// warehouse model. The robot simply passively falls to the floor within the
/// walls of the warehouse, falling from the initial_height command line
/// argument.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using multibody::joints::kFixed;
using multibody::joints::kRollPitchYaw;
using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using parsers::sdf::AddModelInstancesFromSdfFile;
using systems::InputPortDescriptor;

namespace examples {
namespace quad_tilt_wing {
namespace {

DEFINE_double(duration, 1.5, "Total duration of simulation.");
DEFINE_double(initial_height, 10.0, "Initial height of the quad tilt-wing.");
DEFINE_double(realtime_rate, 0.1, "Rate at which to run the simulation, relative to realtime.");

template <typename T>
class QuadTiltWing : public systems::Diagram<T> {
 public:
  QuadTiltWing() {
    this->set_name("QuadTiltWing");

    auto tree = std::make_unique<RigidBodyTree<T>>();
    ModelInstanceIdTable model_id_table = AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/examples/quad_tilt_wing/quad_tilt_wing.urdf"),
        kRollPitchYaw, tree.get());
    const int quad_tilt_wing_id = model_id_table.at("quad_tilt_wing");  // "quad_tilt_wing" is the name defined in urdf file
    AddModelInstancesFromSdfFile(
        FindResourceOrThrow("drake/examples/quad_tilt_wing/warehouse.sdf"),
        kFixed, nullptr /* weld to frame */, tree.get());
    //~ drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<T> builder;

    plant_ =
        builder.template AddSystem<systems::RigidBodyPlant<T>>(std::move(tree));
    plant_->set_name("plant");

    // Verifies that the quad tilt wing has no actuators.
    DRAKE_DEMAND(plant_->get_num_actuators() == 0);
    DRAKE_DEMAND(plant_->get_num_actuators(quad_tilt_wing_id) == 0);

    systems::DrakeVisualizer* publisher =
        builder.template AddSystem<systems::DrakeVisualizer>(
            plant_->get_rigid_body_tree(), &lcm_);

    builder.Connect(plant_->get_output_port(0), publisher->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);                                     // what is DRAKE_DEMAND? nullptr?
    systems::Diagram<T>::SetDefaultState(context, state);
    systems::State<T>& plant_state =
        this->GetMutableSubsystemState(*plant_, state);
    VectorX<T> x0(plant_->get_num_states());
    x0.setZero();
    /* x0 is the initial state where
     * x0(0), x0(1), x0(2) are the quad tilt-wing's x, y, z -states
     * x0(3), x0(4), x0(5) are the quad tilt-wing's Euler angles phi, theta, psi
     */
    x0(2) = FLAGS_initial_height;  // Sets arbitrary z-position.
    plant_->set_state_vector(&plant_state, x0);
  }

 private:
  systems::RigidBodyPlant<T>* plant_{};
  lcm::DrakeLcm lcm_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  QuadTiltWing<double> model;
  systems::Simulator<double> simulator(model);

  // Same as the nominal step size, since we're using a fixed step integrator.
  const double max_step_size = 1e-3;
  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
      model, max_step_size, &simulator.get_mutable_context());
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_duration);
  return 0;
}

}  // namespace
}  // namespace quad_tilt_wing
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::quad_tilt_wing::do_main(argc, argv);
}
