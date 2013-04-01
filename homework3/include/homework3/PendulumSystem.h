/**
 * @file   PendulumSystem.h
 *
 * @date   Apr 1, 2013
 * @author Adam Panzica
 * @brief  Definitions for the pendulum system
 */

#ifndef PENDULUMSYSTEM_H_
#define PENDULUMSYSTEM_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ODESolver.h>
#include <ompl/config.h>
#include<ompl/base/spaces/SO2StateSpace.h>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

#define PS_PI 3.14159
#define PS_OMEGA_BOUND 3.0*PS_PI

namespace homework3
{
namespace oc = ompl::control;
namespace ob = ompl::base;


class PendulumControlSpace : public oc::RealVectorControlSpace
{
public:

    PendulumControlSpace(const ob::StateSpacePtr &stateSpace);

};

class PendulumStateSpace : public ob::CompoundStateSpace
{
public:

    class StateType : public CompoundStateSpace::StateType
    {
    public:
        StateType();


        double getTheta() const;

        double getOmega() const;

        void setTheta(double theta);

        void setOmega(double omega);
    };


    PendulumStateSpace();

    virtual void setBounds(const ob::RealVectorBounds &bounds);

    virtual const ob::RealVectorBounds& getBounds() const;
};

inline void pendulumODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    //Get control
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double tau = u[0];

    //Get states
    const double theta = q[0];
    const double omega = q[1];

    //Make sure qdot is the right size
    qdot.resize(q.size(), 0);

    //Set theta dot
    qdot[0] = omega;

    //Set omega dot;
    qdot[1] = -9.81*cos(theta)+tau;
}

inline bool pendulumIsStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    /// cast the abstract state type to the type we expect
    const homework3::PendulumStateSpace::StateType *penstate = state->as<PendulumStateSpace::StateType>();

    /// check validity of state defined by theta/omega and the maximum value of omega
    return si->satisfiesBounds(state) && (penstate->getOmega()<PS_OMEGA_BOUND);
}

inline void pendulumPostPropagate(const ob::State* state, const oc::Control* control, const double duration, ob::State* result)
{
    ob::SO2StateSpace SO2;
    // Ensure that the pendulum's theta is between 0-2*Pi
    PendulumStateSpace::StateType& s = *result->as<PendulumStateSpace::StateType>();
    SO2.enforceBounds(s[0]);
}

}

#endif /* PENDULUMSYSTEM_H_ */
