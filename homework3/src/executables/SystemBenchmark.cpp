/**
 * @file   PendulumSystemBenchmark.cpp
 *
 * @date   Apr 1, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include<iostream>
#include<fstream>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
//************ LOCAL DEPENDANCIES ****************//
#include <homework3/PendulumSystem.h>
#include <homework3/CarSystem.h>
#include <homework3/RG_RRT.h>
//***********    NAMESPACES     ****************//
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;

void pendulumWithSimpleSetup()
{
    /// construct the state space we are planning in
    ob::StateSpacePtr space(new homework3::PendulumStateSpace());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-3.14159);
    bounds.setHigh(3.14159);

    space->as<homework3::PendulumStateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new homework3::PendulumControlSpace(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-2.0);
    cbounds.setHigh(2.0);

    cspace->as<homework3::PendulumControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&homework3::pendulumIsStateValid, ss.getSpaceInformation().get(), _1));

    //Build ODE solver
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &homework3::pendulumODE));
    odeSolver->setIntegrationStepSize(0.01);
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &homework3::pendulumPostPropagate));

    /// create a start state
    ob::ScopedState<homework3::PendulumStateSpace> start(space);
    start->setTheta(-1.570795);
    start->setOmega(0);

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<homework3::PendulumStateSpace> goal(space);
    goal->setTheta(1.570795);
    goal->setOmega(0);

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);
    ss.setPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));
    ss.setup();

    std::string benchmark_name("Pendulm_System_Benchmark");
    ot::Benchmark b(ss, benchmark_name);

    ot::Benchmark::Request req;
    req.maxTime         = 10.0;
    req.maxMem          = 200.0;
    req.runCount        = 10;
    req.displayProgress = true;
    req.useThreads      = false;

    b.addPlanner(ob::PlannerPtr(new homework3::RG_RRT(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));
    b.setExperimentName(benchmark_name);
    b.benchmark(req);
    b.saveResultsToFile("Pendulum_System_Results");

    std::string path_file_name("Pendulum_System_Results.txt");
    std::ofstream path_file(path_file_name.c_str());

    ss.getSolutionPath().asGeometric().print(path_file);

}

//void carPostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
//{

//    //std::string data_file_name("solution_data_results.txt");

//    //std::ofstream data_file(data_file_name.c_str());
//    ob::PlannerData data(planner->getSpaceInformation());
//    //planner->getPlannerData(data);
//    planner->getProblemDefinition()->getSolutionPath()->print(path_file);
//}

void carWithSimpleSetup()
{
    /// construct the state space we are planning in
    ob::StateSpacePtr space(new homework3::CarStateSpace());

    //std::cout<<"Setting Space Bounds..."<<std::endl;
    ob::RealVectorBounds bounds(4);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->as<homework3::CarStateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new homework3::CarControlSpace(space));

    //std::cout<<"Setting Control Bounds..."<<std::endl;
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-2.0);
    cbounds.setHigh(2.0);

    cspace->as<homework3::CarControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    //std::cout<<"Setting State Validity Checker..."<<std::endl;
    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&homework3::carIsStateValid, ss.getSpaceInformation().get(), _1));

    //std::cout<<"Setting ODE Solver..."<<std::endl;
    //Build ODE solver
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &homework3::carODE));
    odeSolver->setIntegrationStepSize(0.01);
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &homework3::carPostPropagate));

    //std::cout<<"Setting Start State..."<<std::endl;
    /// create a start state
    ob::ScopedState<homework3::CarStateSpace> start(space);
    start->setX(-4.5);
    start->setY(-4.5);
    start->setV(0);
    start->setTheta(0);

    //start.print(std::cout);

    //std::cout<<"Setting Goal State..."<<std::endl;
    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<homework3::CarStateSpace> goal(space);
    goal->setX(4.5);
    goal->setY(4.5);
    goal->setV(0);
    goal->setTheta(0);

    //goal.print(std::cout);

    //std::cout<<"Setting Start/Goal States..."<<std::endl;
    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);
    ss.setup();

    //std::cout<<"Building Benchmark..."<<std::endl;
    std::string benchmark_name("Car_System_Benchmark");
    ot::Benchmark b(ss, benchmark_name);
    //b.setPostRunEvent(boost::bind(&carPostRunEvent, _1, _2));

    ot::Benchmark::Request req;
    req.maxTime         = 20.0;
    req.maxMem          = 1000.0;
    req.runCount        = 10;
    req.displayProgress = true;
    req.useThreads      = true;

    b.addPlanner(ob::PlannerPtr(new homework3::RG_RRT(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));
    b.setExperimentName(benchmark_name);
    std::cout<<"Running Benchmark..."<<std::endl;
    b.benchmark(req);
    b.saveResultsToFile("Car_System_Results");

    std::string path_file_name("Car_System_Results.txt");
    std::ofstream path_file(path_file_name.c_str());

    ss.getSolutionPath().asGeometric().print(path_file);

}

int main(int argc, char **argv)
{
	std::cout<<"Adam Panzica Homework 3:"<<std::endl<<"Running Pendulum System Benchmark!"<<std::endl;
    pendulumWithSimpleSetup();
    std::cout<<"Running Car System Benchmark!"<<std::endl;
    carWithSimpleSetup();
}
