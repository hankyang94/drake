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

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  /*
  ////////////// read trajectory optimization results 0-10mps ////////////////////
  std::fstream infile("/Users/Hank/solver_output/transform_sol_0mps_10mps_snopt_test_py.txt");
  std::string line;
  int count = 0;
  int tLine = 0;
  int uLine = 0;
  int xLine = 0;
  while (std::getline(infile, line)){
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
  std::fstream infile_t("/Users/Hank/solver_output/transform_sol_0mps_10mps_snopt_test_py.txt");
  std::string line_t;
  int lineCount_t = 0;
  int tCount = 0;
  while (std::getline(infile_t, line_t) && lineCount_t < uLine) {
    double t = atof(line_t.c_str());
    if (!(lineCount_t == tLine)){
      t_samples(tCount) = t;
      tCount ++;
    }
    lineCount_t ++;
  }
  std::cout << "t count: " << tCount << std::endl;
//  std::cout << "t samples: " << t_samples << std::endl;
  infile_t.close();

  Eigen::MatrixXd u_samples(8, uLine-1);
  std::fstream infile_u("/Users/Hank/solver_output/transform_sol_0mps_10mps_snopt_test_py.txt");
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
//  std::cout << "u samples: " << u_samples << std::endl;
  infile_u.close();

  Eigen::MatrixXd x_samples(12, uLine-1);
  std::fstream infile_x("/Users/Hank/solver_output/transform_sol_0mps_10mps_snopt_test_py.txt");
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
//  std::cout << "x samples: " << x_samples << std::endl;
  infile_x.close();

  ////////////// read trajectory optimization results 10-50mps ////////////////////
  std::fstream new_infile("/Users/Hank/solver_output/transform_sol_10mps_50mps_snopt_test_py.txt");
  std::string new_line;
  int new_count = 0;
  int new_tLine = 0;
  int new_uLine = 0;
  int new_xLine = 0;
  while (std::getline(new_infile, new_line)){
    if (new_line[0] == 't'){ new_tLine = new_count; }
    if (new_line[0] == 'u'){ new_uLine = new_count; }
    if (new_line[0] == 'x'){ new_xLine = new_count; }
    new_count++;
  }
  std::cout << "tLine: " << new_tLine << std::endl;
  std::cout << "uLine: " << new_uLine << std::endl;
  std::cout << "xLine: " << new_xLine << std::endl;
  new_infile.close();

  Eigen::VectorXd new_t_samples(new_uLine-1);
  std::fstream new_infile_t("/Users/Hank/solver_output/transform_sol_10mps_50mps_snopt_test_py.txt");
  std::string new_line_t;
  int new_lineCount_t = 0;
  int new_tCount = 0;
  while (std::getline(new_infile_t, new_line_t) && new_lineCount_t < new_uLine) {
    double new_t = atof(new_line_t.c_str());
    if (!(new_lineCount_t == new_tLine)){
      new_t_samples(new_tCount) = new_t;
      new_tCount ++;
    }
    new_lineCount_t ++;
  }
  std::cout << "t count: " << new_tCount << std::endl;
//  std::cout << "t samples: " << new_t_samples << std::endl;
  new_infile_t.close();

  Eigen::MatrixXd new_u_samples(8, uLine-1);
  std::fstream new_infile_u("/Users/Hank/solver_output/transform_sol_10mps_50mps_snopt_test_py.txt");
  std::string new_line_u;
  int new_lineCount_u = 0;
  int new_uCount = 0;
  while (std::getline(new_infile_u, new_line_u) && new_lineCount_u < new_xLine) {
    if (new_lineCount_u>new_uLine){
      std::vector<double> new_data;
      std::stringstream new_lineStream(new_line_u);
      std::copy(  std::istream_iterator<double>(new_lineStream),
                  std::istream_iterator<double>(),
                  std::back_inserter(new_data));
      Map<MatrixXd> new_temp_u(new_data.data(), 1, new_uLine-1);
      new_u_samples.row(new_uCount) << new_temp_u;
      new_uCount++;
      std::cout << "temp u: " << new_data.size() << std::endl;
    }
    new_lineCount_u++;
  }
//  std::cout << "u samples: " << new_u_samples << std::endl;
  new_infile_u.close();

  Eigen::MatrixXd new_x_samples(12, new_uLine-1);
  std::fstream new_infile_x("/Users/Hank/solver_output/transform_sol_10mps_50mps_snopt_test_py.txt");
  std::string new_line_x;
  int new_lineCount_x = 0;
  int new_xCount = 0;
  while (std::getline(new_infile_x, new_line_x)) {
    if (new_lineCount_x>new_xLine){
      std::vector<double> new_data;
      std::stringstream new_lineStream(new_line_x);
      std::copy(  std::istream_iterator<double>(new_lineStream),
                  std::istream_iterator<double>(),
                  std::back_inserter(new_data));
      Map<MatrixXd> new_temp_x(new_data.data(), 1, new_uLine-1);
      new_x_samples.row(new_xCount) << new_temp_x;
      new_xCount++;
      std::cout << "temp x: " << new_data.size() << std::endl;
    }
    new_lineCount_x++;
  }
//  std::cout << "x samples: " << new_x_samples << std::endl;
  new_infile_x.close();

//  std::cout << "0-10mps t: " << t_samples << std::endl;
//  std::cout << "0-10mps u: " << u_samples << std::endl;
//  std::cout << "0-10mps x: " << x_samples << std::endl;
//
//  std::cout << "10-50mps t: " << new_t_samples << std::endl;
//  std::cout << "10-50mps u: " << new_u_samples << std::endl;
//  std::cout << "10-50mps x: " << new_x_samples << std::endl;

  std::cout << u_samples.rightCols(1) << std::endl;
  std::cout << x_samples.rightCols(1) << std::endl;
  std::cout << new_u_samples.leftCols(1) << std::endl;
  std::cout << new_x_samples.leftCols(1) << std::endl;

  const int num_samples_1 = t_samples.size();
  const int num_samples_2 = new_t_samples.size();
  const double t_1_final = t_samples(num_samples_1-1);
  std::cout << t_samples.tail(1) << std::endl;
  for (int i = 0; i<num_samples_2; i++) {new_t_samples(i) += t_1_final;}
  Eigen::VectorXd t_samples_f(num_samples_1 + num_samples_2 - 1);
  t_samples_f << t_samples, new_t_samples.tail(num_samples_2-1);
  std::cout << t_samples_f << std::endl;

  Eigen::MatrixXd u_samples_f(8, num_samples_1 + num_samples_2 - 1);
  u_samples_f.leftCols(num_samples_1-1) = u_samples.leftCols(num_samples_1-1);
  u_samples_f.rightCols(num_samples_2) = new_u_samples;

  auto q_offset = x_samples.col(num_samples_1-1).head(3) - new_x_samples.col(0).head(3);
  std::cout << q_offset << std::endl;
  Eigen::VectorXd x_offset(12);
  x_offset << q_offset, Eigen::VectorXd::Zero(9);
  for (int i = 0; i<num_samples_2; i++) {
    new_x_samples.col(i) += x_offset;
  }
  Eigen::MatrixXd x_samples_f(12, num_samples_1 + num_samples_2 - 1);
  x_samples_f.leftCols(num_samples_1-1) = x_samples.leftCols(num_samples_1-1);
  x_samples_f.rightCols(num_samples_2) = new_x_samples;

  std::ofstream file("/Users/Hank/solver_output/transform_sol_0mps_50mps_combined_test_py.txt");

  if (file.is_open()) {
    file << "t samples: " << '\n';
    file << t_samples_f << '\n';
    file << "u_samples: " << '\n';
    file << u_samples_f << '\n';
    file << "x_samples: " << '\n';
    file << x_samples_f << '\n';
  }
*/


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
  const double kMaximumTimeStep = 1.0;
  systems::trajectory_optimization::DirectCollocation dirtran(
      quad_tilt_wing_plant.get(), *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);


  // Add bounds to the inputs
  const solvers::VectorXDecisionVariable& u = dirtran.input();
  const double UAV_fg = quad_tilt_wing_plant -> m() * quad_tilt_wing_plant -> g();
  const double kThrustLowerLimit = 0;
  const double kThrustUpperLimit = UAV_fg/2.0;
  const double kTiltUpperLimit = 0.1;
  const double kTiltLowerLimit = -0.51*M_PI;
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
//  const double kStateTol = 1e-4;
  const double kXLowerLimit = -5.0;
  const double kXUpperLimit = 2000;
  const double kYLimit = 1;
  const double kZLowerLimit = -2000;
  const double kZUpperLimit = 2000;
  const double kPhiLimit = M_PI/200;
  const double kThetaLimit = M_PI/200;
  const double kPsiLimit = M_PI/200;
  // velocity constriants, very high bound
  const double kXDotLowerLimit = -5.0;
  const double kXDotUpperLimit = 120.0;
  const double kYDotLimit = 1;
  const double kZDotLimit = 100;
  const double kPhiDotLimit = M_PI/400;
  const double kThetaDotLimit = M_PI/400;
  const double kPsiDotLimit = M_PI/400;
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
  initial_state(2) = 10.0; // Z_0, start at height 10m, all the other states are 0
  initial_state(6) = 100.0;
  dirtran.AddLinearConstraint(dirtran.initial_state() == initial_state);

  std::cout << "initial state: " << initial_state << std::endl;

  // Add initial input constraint
  Eigen::VectorXd initial_input(8);
  initial_input << 0.0104247, 0.0104247, 0.010424, 0.010424, -0.00707774, -0.00707774, -0.00249736, -0.00249737;
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_0 = dirtran.input(0);
  const double kSlack_u_N_prop = 1e-4;
  const double kSlack_u_N_tilt = 1e-4;
  for (int i = 0; i < 4; i++) {
    dirtran.AddLinearConstraint(u_0(i) >= initial_input(i) - kSlack_u_N_prop);
    dirtran.AddLinearConstraint(u_0(i) <= initial_input(i) + kSlack_u_N_prop);
  }
  for (int i = 4; i < 8; i++) {
    dirtran.AddLinearConstraint(u_0(i) >= initial_input(i) - kSlack_u_N_tilt);
    dirtran.AddLinearConstraint(u_0(i) <= initial_input(i) + kSlack_u_N_tilt);
  }


  // Add tilt-speed limit constraint
  const double kTiltSpeedLimit = M_PI / 30.0; // limit the tilting speed
  for (int i=1; i<(kNumTimeSamples); i++){
    dirtran.AddLinearConstraint(dirtran.input(i).segment(4,1) - dirtran.input(i-1).segment(4,1) >=
                                -kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(4,1) - dirtran.input(i-1).segment(4,1) <=
                                kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(6,1) - dirtran.input(i-1).segment(6,1) >=
                                -kTiltSpeedLimit*dirtran.timestep(i-1));
    dirtran.AddLinearConstraint(dirtran.input(i).segment(6,1) - dirtran.input(i-1).segment(6,1) <=
                                kTiltSpeedLimit*dirtran.timestep(i-1));
  }


  // Add final state constraint
  Eigen::VectorXd final_state = Eigen::VectorXd::Zero(12);
  final_state(0) = 1000;  // for initial guess
  final_state(2) = 1000; // Z_N, try keeping the same height at final state
  final_state(6) = 0.0;
  dirtran.AddLinearConstraint(dirtran.final_state().tail(9) == final_state.tail(9));


  std::cout << "final state: " << final_state << std::endl;


  // Add final input constraint
  Eigen::VectorXd final_input(8);
  const double final_tilt_angle = -M_PI/2.0; // final configuration is a quadrotor
  const double front_moment_arm = quad_tilt_wing_plant->front_joint_x();
  const double rear_moment_arm = quad_tilt_wing_plant->rear_joint_x();
  const double front_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * rear_moment_arm / 2.0;
  const double rear_prop_f = UAV_fg/(rear_moment_arm + front_moment_arm) * front_moment_arm / 2.0;
  Eigen::Vector4d uN_prop{front_prop_f, front_prop_f, rear_prop_f, rear_prop_f};
  final_input.topRows(4) = uN_prop;
  final_input.bottomRows(4) = Eigen::VectorXd::Constant(4, final_tilt_angle);
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> u_N = dirtran.input(kNumTimeSamples-1);
  for (int i = 0; i < 4; i++) {
    dirtran.AddLinearConstraint(u_N(i) >= final_input(i) - kSlack_u_N_prop);
    dirtran.AddLinearConstraint(u_N(i) <= final_input(i) + kSlack_u_N_prop);
  }
  for (int i = 4; i < 8; i++) {
    dirtran.AddLinearConstraint(u_N(i) >= final_input(i) - kSlack_u_N_tilt);
    dirtran.AddLinearConstraint(u_N(i) <= final_input(i) + kSlack_u_N_tilt);
  }


  // Add running cost
//  dirtran.AddRunningCost(1 * x(10) * x(10));
  const double R_prop = 0;  // Cost on input "effort".
  for (int i = 0; i < 4; i++) {
      dirtran.AddRunningCost(R_prop * u(i) * u(i));
  }
  const double R_tilt = 0;  // Cost on input "effort".
  for (int i = 4; i < 8; i++) {
      dirtran.AddRunningCost(R_tilt * u(i) * u(i));
  }

  std::cout << "Finished adding all constraints and costs." << std::endl;

  // set initial guess
  const double timespan_init = (kNumTimeSamples - 1) * (kMinimumTimeStep+kMaximumTimeStep)/2.0;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state, final_state});
  auto initial_input_guess = initial_input;
  auto final_input_guess = final_input;
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_input_guess, final_input_guess});
  dirtran.SetInitialTrajectory(traj_init_u, traj_init_x);

  // Set solver options
//  const double tol = 1e-4;
  solvers::SnoptSolver solver;
//  dirtran.SetSolverOption(solvers::SnoptSolver::id(),
//                          "Print file", "/home/klytech/solver_output/traj_opt_100mps_snopt.txt");
  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Scale option", 1);
  dirtran.SetSolverOption(solvers::SnoptSolver::id(), "Iteration limit", 64000);
  dirtran.SetSolverOption(solvers::SnoptSolver::id(),
                          "Print file", "/Users/Hank/solver_output/transform_100mps_0mps_snopt_new.txt");

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

//  std::ofstream file("/home/klytech/solver_output/traj_opt_sol_100mps_snopt_py.txt");
  std::ofstream file("/Users/Hank/solver_output/transform_sol_100mps_0mps_snopt_new_py.txt");

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
  simulator.StepTo(0.001);
  std::cout << "Pausing for 15 seconds, tune the visualizer please." << std::endl;

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
