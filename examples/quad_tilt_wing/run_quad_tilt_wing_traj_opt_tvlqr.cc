#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <math.h>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <iterator>
#include <algorithm>

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
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/primitives/signal_logger.h"

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
  quad_tilt_wing_plant->set_name("quad_tilt_wing_plant");

  auto context = quad_tilt_wing_plant->CreateDefaultContext();

  // Add initial state constraint
//  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(12);
//  initial_state(2) = 10.0; // Z_0, start at height 10m, all the other states are 0
//
//  std::cout << "initial state: " << initial_state << std::endl;
//
//  // Add initial input constraint
//  const double UAV_fg = quad_tilt_wing_plant->m() * quad_tilt_wing_plant->g();
//  const double initial_tilt_angle = -M_PI/2.0; // initial configuration is a quadrotor
//  const double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
//  const double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
//  const double front_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * rear_moment_arm / 2.0;
//  const double rear_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * front_moment_arm / 2.0;
//  Eigen::Vector4d u0_prop{front_prop_f, front_prop_f,
//      rear_prop_f, rear_prop_f};
//  Eigen::VectorXd initial_input = Eigen::VectorXd::Zero(8);
//  initial_input.topRows(4) = u0_prop;
//  initial_input.bottomRows(4) = Eigen::VectorXd::Constant(4, initial_tilt_angle);

  // read trajectory optimization results
//  std::fstream infile("/home/klytech/solver_output/traj_opt_sol_100mps_py.txt");
  std::fstream infile("/Users/Hank/solver_output/transform_sol_100mps_0mps_snopt_new_py.txt");
  std::string line;
  int count = 0;
  int tLine = 0;
  int uLine = 0;
  int xLine = 0;
  while (std::getline(infile, line)){
//    std::cout << "line: " << line[0] << std::endl;
    if (line[0] == 't'){ tLine = count; }
    if (line[0] == 'u'){ uLine = count; }
    if (line[0] == 'x'){ xLine = count; }
    count++;
  }
  std::cout << "tLine: " << tLine << std::endl;
  std::cout << "uLine: " << uLine << std::endl;
  std::cout << "xLine: " << xLine << std::endl;
  infile.close();

  Eigen::VectorXd t_samples(uLine-1);
//  std::fstream infile_t("/home/klytech/solver_output/traj_opt_sol_100mps_py.txt");
  std::fstream infile_t("/Users/Hank/solver_output/transform_sol_100mps_0mps_snopt_new_py.txt");
  std::string line_t;
  int lineCount_t = 0;
  int tCount = 0;
  while (std::getline(infile_t, line_t) && lineCount_t < uLine) {
    double t = atof(line_t.c_str());
//    std::cout << "t samples: " << t << std::endl;
    if (!(lineCount_t == tLine)){
      t_samples(tCount) = t;
      tCount ++;
    }
    lineCount_t ++;
  }
  std::cout << "t count: " << tCount << std::endl;
  std::cout << "t samples: " << t_samples << std::endl;
  infile_t.close();

  Eigen::MatrixXd u_samples(8, uLine-1);
//  std::fstream infile_u("/home/klytech/solver_output/traj_opt_sol_100mps_py.txt");
  std::fstream infile_u("/Users/Hank/solver_output/transform_sol_100mps_0mps_snopt_new_py.txt");
  std::string line_u;
  int lineCount_u = 0;
  int uCount = 0;
  while (std::getline(infile_u, line_u) && lineCount_u < xLine) {
    if (lineCount_u>uLine){
      std::vector<double> data;
      std::stringstream lineStream(line_u);
      std::copy(  std::istream_iterator<double>(lineStream),
                  std::istream_iterator<double>(),
                  std::back_inserter(data));
      Map<MatrixXd> temp_u(data.data(), 1, uLine-1);
      u_samples.row(uCount) << temp_u;
      uCount++;
      std::cout << "temp u: " << data.size() << std::endl;
    }
    lineCount_u++;
  }
  std::cout << "u samples: " << u_samples << std::endl;
  infile_u.close();

  Eigen::MatrixXd x_samples(12, uLine-1);
//  std::fstream infile_x("/home/klytech/solver_output/traj_opt_sol_100mps_py.txt");
  std::fstream infile_x("/Users/Hank/solver_output/transform_sol_100mps_0mps_snopt_new_py.txt");
  std::string line_x;
  int lineCount_x = 0;
  int xCount = 0;
  while (std::getline(infile_x, line_x)) {
    if (lineCount_x>xLine){
      std::vector<double> data;
      std::stringstream lineStream(line_x);
      std::copy(  std::istream_iterator<double>(lineStream),
                  std::istream_iterator<double>(),
                  std::back_inserter(data));
      Map<MatrixXd> temp_x(data.data(), 1, uLine-1);
      x_samples.row(xCount) << temp_x;
      xCount++;
      std::cout << "temp x: " << data.size() << std::endl;
    }
    lineCount_x++;
  }
  std::cout << "x samples: " << x_samples << std::endl;
  infile_x.close();

//  std::vector<double> tSamples;
//  tSamples.resize(t_samples.size());
//  Eigen::VectorXd::Map(&tSamples[0], t_samples.size()) = t_samples;
//  std::cout << "std:vector tSamples size " << tSamples[12] << std::endl;

  PiecewisePolynomial<double> u_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          t_samples, u_samples);
  PiecewisePolynomial<double> x_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          t_samples, x_samples);

  std::cout << "test u: " << u_traj.value(10.3) << std::endl;
  std::cout << "test x: " << x_traj.value(33) << std::endl;
  auto initial_state = x_traj.value(0.0);
  auto controller = builder.AddSystem(TimeVaryingLinearQuadraticRegulator(
          quad_tilt_wing_plant, u_traj, x_traj));
  controller->set_name("controller");

//  connect dynamics ports
  builder.Connect(quad_tilt_wing_plant->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quad_tilt_wing_plant->get_input_port(0));

  // Setup mux and demux for plant, controller, and publisher
  auto plant_demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(12, 6);
  plant_demux->set_name("plant_demux");
  auto controller_demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(8, 4);
  controller_demux->set_name("controller_demux");
  auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(std::vector<int> {6,4,6,4});
  mux->set_name("mux");
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  // Demux plant and controller
  builder.Connect(quad_tilt_wing_plant->get_output_port(0), plant_demux->get_input_port(0));
  builder.Connect(controller->get_output_port(), controller_demux->get_input_port(0));
  // Mux demuxed signals to mux
  builder.Connect(plant_demux->get_output_port(0), mux->get_input_port(0));
  builder.Connect(controller_demux->get_output_port(1), mux->get_input_port(1));
  builder.Connect(plant_demux->get_output_port(1), mux->get_input_port(2));
  builder.Connect(controller_demux->get_output_port(1), mux->get_input_port(3));
  // Connect result to publisher
  builder.Connect(mux->get_output_port(0), publisher->get_input_port(0));
  // Add state and control log
  auto state_log =
          builder.AddSystem<drake::systems::SignalLogger<double>>(12);
  auto control_log =
          builder.AddSystem<drake::systems::SignalLogger<double>>(8);

  builder.Connect(quad_tilt_wing_plant->get_output_port(0), state_log->get_input_port());
  builder.Connect(controller->get_output_port(), control_log->get_input_port());

  auto diagram = builder.Build();

  std::cout << "Diagram built." << std::endl;
  auto diagram_context = diagram->CreateDefaultContext();
  systems::Simulator<double> simulator(*diagram);

  Eigen::VectorXd initial_diagram(13);
  initial_diagram << initial_state, 0;
  std::cout << "simulator state size: " << simulator.get_mutable_context().get_mutable_continuous_state_vector().size();
  simulator.get_mutable_context()
          .get_mutable_continuous_state_vector()
          .SetFromVector(initial_diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(0.001);
  std::cout << "Pausing for 20 seconds, tune the visualizer please." << std::endl;

  sleep(10);

  simulator.StepTo(x_traj.end_time());
  std::cout << "Simulation Done." << std::endl;

  std::ofstream state_file("/Users/Hank/solver_output/TVLQR_log_state_100mps_0mps.txt");
  std::ofstream control_file("/Users/Hank/solver_output/TVLQR_log_control_100mps_0mps.txt");
  std::ofstream time_file("/Users/Hank/solver_output/TVLQR_log_time_100mps_0mps.txt");

  if (state_file.is_open()) {
    state_file << state_log->data() << '\n';
  }
  if (control_file.is_open()) {
    control_file << control_log->data() << '\n';
  }
  if (time_file.is_open()) {
    time_file << state_log->sample_times() << '\n';
  }

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
