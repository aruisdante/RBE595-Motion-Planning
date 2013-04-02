/**
 * @file   PendulumSystem.cpp
 *
 * @date   Apr 1, 2013
 * @author Adam Panzica
 * @brief  Benchmark/Solver for the Pendulum System Problem
 */

//*********** SYSTEM DEPENDANCIES ****************//

//************ LOCAL DEPENDANCIES ****************//
#include<homework3/PendulumSystem.h>
//***********    NAMESPACES     ****************//

using namespace homework3;

//void pendulumODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
//{
//    //Get control
//    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
//    const double tau = u[0];

//    //Get states
//    const double theta = q[0];
//    const double omega = q[1];

//    //Make sure qdot is the right size
//    qdot.resize(q.size(), 0);

//    //Set theta dot
//    qdot[0] = omega;

//    //Set omega dot;
//    qdot[1] = -9.81*cos(theta)+tau;
//}

//bool pendulumIsStateValid(const oc::SpaceInformation *si, const ob::State *state)
//{
//    /// cast the abstract state type to the type we expect
//    const PendulumStateSpace::StateType *penstate = state->as<PendulumStateSpace::StateType>();

//    /// check validity of state defined by theta/omega and the maximum value of omega
//    return si->satisfiesBounds(state) && (penstate->getOmega()<PS_OMEGA_BOUND);
//}

//void homework3::pendulumPostPropagate(const ob::State* state, const oc::Control* control, const double duration, ob::State* result)
//{
//    ob::SO2StateSpace SO2;
//    // Ensure that the pendulum's theta is between 0-2*Pi
//    PendulumStateSpace::StateType& s = *result->as<PendulumStateSpace::StateType>();
//    SO2.enforceBounds(s[0]);
//}

PendulumControlSpace::PendulumControlSpace(const ob::StateSpacePtr &stateSpace): oc::RealVectorControlSpace(stateSpace, 1)
{
    this->setName("PendulumControl"+getName());
}

void printCtrCSV(const oc::Control* control, std::ofstream& stream)
{
    std::stringstream s;
    s<<control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    stream<<s.str()<<std::endl;
}


PendulumStateSpace::StateType::StateType() : CompoundStateSpace::StateType(){}

double PendulumStateSpace::StateType::getTheta() const
{
    return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
}

double PendulumStateSpace::StateType::getOmega() const
{
    return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
}

void PendulumStateSpace::StateType::setTheta(double theta)
{
    as<ob::RealVectorStateSpace::StateType>(0)->values[0] = theta;
}

void PendulumStateSpace::StateType::setOmega(double omega)
{
    as<ob::RealVectorStateSpace::StateType>(0)->values[1] = omega;
}

PendulumStateSpace::PendulumStateSpace() : CompoundStateSpace()
{
    this->setName("PendulumSpace"+getName());
    this->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
    this->lock();
}

void PendulumStateSpace::setBounds(const ob::RealVectorBounds &bounds)
{
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

const ob::RealVectorBounds& PendulumStateSpace::getBounds() const
{
    return as<ob::RealVectorStateSpace>(0)->getBounds();
}

void PendulumStateSpace::StateType::printCSV(std::ofstream& stream) const
{
    std::stringstream s;
    s<<this->getTheta()<<","<<this->getOmega()<<std::endl;
    s<<s.str();
}
