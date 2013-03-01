/**
 * @file test_rtp_planner.cpp
 *
 * @date   Feb 28, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <boost/bind.hpp>
#include <string>
#include <vector>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
//*****************LOCAL DEPENDANCIES**************************//
#include "RandomTreePlanner.h"
#include "BenchmarkConfigurationPaths.h"
//**********************NAMESPACES*****************************//

using namespace ompl;

base::ValidStateSamplerPtr allocUniformStateSampler(const base::SpaceInformation *si)
{
	return base::ValidStateSamplerPtr(new base::UniformValidStateSampler(si));
}

base::ValidStateSamplerPtr allocGaussianStateSampler(const base::SpaceInformation *si)
{
	return base::ValidStateSamplerPtr(new base::GaussianValidStateSampler(si));
}

base::ValidStateSamplerPtr allocObstacleStateSampler(const base::SpaceInformation *si)
{
	return base::ValidStateSamplerPtr(new base::ObstacleBasedValidStateSampler(si));
}

base::ValidStateSamplerPtr allocMaximizeClearanceStateSampler(const base::SpaceInformation *si)
{
	base::MaximizeClearanceValidStateSampler *s = new base::MaximizeClearanceValidStateSampler(si);
	s->setNrImproveAttempts(5);
	return base::ValidStateSamplerPtr(s);
}

// A function that matches the ompl::base::PlannerAllocator type.
// It will be used later to allocate an instance of EST
base::PlannerPtr configuredRTP(const ompl::base::SpaceInformationPtr &si)
{
	ompl::RandomTreePlanner *rtp = new ompl::RandomTreePlanner(si);
	return ompl::base::PlannerPtr(rtp);
}

void setUpBenchmark(app::SE2RigidBodyPlanning& setup)
{
	std::string robot_fname(CFG_PATH);
	robot_fname += "/SimpleRobot.dae";
	std::string env_fname(CFG_PATH);
	env_fname   += "/SimpleEnvironment.dae";
	setup.setRobotMesh(robot_fname.c_str());
	setup.setEnvironmentMesh(env_fname.c_str());



	base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
	start->setX(5);
	start->setY(5);
	start->setYaw(0.0);

	base::ScopedState<base::SE2StateSpace> goal(setup.getSpaceInformation());
	goal->setX(35);
	goal->setY(35);
	goal->setYaw(0);

	setup.setStartAndGoalStates(start, goal);
	setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
	setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
	setup.setup();

	std::vector<double> cs(2);
	cs[0] = 35; cs[1] = 35;
	setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);
}

int main(int argc, char **argv)
{

	//Set up the test space
	app::SE2RigidBodyPlanning ss;

	setUpBenchmark(ss);

	//Create a benchmark class:
	std::string benchmark_name("random_tree_planner_benchmark");
	tools::Benchmark b(ss, benchmark_name);
	tools::Benchmark::Request req;
	req.maxTime         = 30.0;
	req.maxMem          = 200.0;
	req.runCount        = 200;
	req.displayProgress = true;
	req.useThreads      = false;

	//Add built in planners
	b.addPlanner(base::PlannerPtr(new geometric::RRT(ss.getSpaceInformation())));
	b.addPlanner(base::PlannerPtr(new geometric::PRM(ss.getSpaceInformation())));
	// For planners that we want to configure in specific ways,
	// the ompl::base::PlannerAllocator should be used:
	b.addPlannerAllocator(boost::bind(&configuredRTP, _1));

	//Run the benchmark
	ss.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
	b.setExperimentName(benchmark_name + "_uniform_sampler");
	b.benchmark(req);
	b.saveResultsToFile();

	return 0;
}
