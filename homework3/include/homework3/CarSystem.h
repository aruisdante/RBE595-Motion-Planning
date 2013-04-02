/**
 * @file   CarSystem.h
 *
 * @date   Apr 2, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef CARSYSTEM_H_
#define CARSYSTEM_H_

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
#include<ompl/base/spaces/SE2StateSpace.h>
#include<fstream>
#include<ostream>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

namespace homework3
{
namespace oc = ompl::control;
namespace ob = ompl::base;


class CarControlSpace : public oc::RealVectorControlSpace
{
public:

    CarControlSpace(const ob::StateSpacePtr &stateSpace);

};

class CarStateSpace : public ob::CompoundStateSpace
{
public:

    class StateType : public CompoundStateSpace::StateType
    {
    public:
        StateType();

        double getX() const;

        double getY() const;

        double getTheta() const;

        double getV() const;

        void setX(double x);

        void setY(double y);

        void setTheta(double theta);

        void setV(double v);

        void printCSV(std::ofstream& stream) const;
    };


    CarStateSpace();

    virtual void setBounds(const ob::RealVectorBounds &bounds);

    virtual const ob::RealVectorBounds& getBounds() const;

};



inline void carODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    //Get control
    const double *u = c->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double u0 = u[0];
    const double u1 = u[1];

    //Get states
    const double x = q[0];
    const double y = q[1];
    const double t = q[2];
    const double v = q[3];

    //Make sure qdot is the right size
    qdot.resize(q.size(), 0);

    //Set q dot
    qdot[0] = v*cos(t);
    qdot[1] = v*sin(t);
    qdot[2] = u0;
    qdot[3] = u1;
}

inline bool collideBox(const CarStateSpace::StateType* state, double x_min, double x_max, double y_min, double y_max)
{
    bool lowerX = state->getX() > x_min;
    bool upperX = state->getX() < x_max;
    bool lowerY = state->getY() > y_min;
    bool upperY = state->getY() < y_max;

    return lowerX&&upperX&&lowerY&&upperY;
}


inline bool collisionCheckBottomBox(const CarStateSpace::StateType* state)
{
    double x_min = -5;
    double x_max = 5;
    double y_min = -10;
    double y_max = -6;

    return !collideBox(state, x_min, x_max, y_min, y_max);
}

inline bool collisionCheckLeftBox(const CarStateSpace::StateType* state)
{
    double x_min = -5;
    double x_max = 0;
    double y_min = -4;
    double y_max = 4;

    return !collideBox(state, x_min, x_max, y_min, y_max);
}

inline bool collisionCheckRightBox(const CarStateSpace::StateType* state)
{
    double x_min = 2;
    double x_max = 5;
    double y_min = -4;
    double y_max = 4;

    return !collideBox(state, x_min, x_max, y_min, y_max);
}

inline bool collisionCheckTopBox(const CarStateSpace::StateType* state)
{
    double x_min = -5;
    double x_max = 5;
    double y_min = 6;
    double y_max = 8;

    return !collideBox(state, x_min, x_max, y_min, y_max);
}

inline bool collisionCheckField(const CarStateSpace::StateType* state)
{
    double x_min = -5;
    double x_max = 5;
    double y_min = -10;
    double y_max = 10;

    return collideBox(state, x_min, x_max, y_min, y_max);
}

inline bool carIsStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    const CarStateSpace::StateType* s = state->as<CarStateSpace::StateType>();
    //std::cout<<"Checking State Validity with state: "<<s->getX()<<","<<s->getY()<<","<<s->getTheta()<<","<<s->getV()<<std::endl;

    return si->satisfiesBounds(state)&&collisionCheckBottomBox(s)
                                     &&collisionCheckLeftBox(s)
                                     &&collisionCheckRightBox(s)
                                     &&collisionCheckTopBox(s)
                                     &&collisionCheckField(s)
                                     &&s->getV()<10.0;
}

inline void carPostPropagate(const ob::State* state, const oc::Control* control, const double duration, ob::State* result)
{
    ob::SO2StateSpace SO2;
    // Ensure that the cars's theta is between 0-2*Pi
    //std::cout<<"Propogating State: "<<s.getX()<<","<<s.getY()<<","<<s.getTheta()<<","<<s.getV()<<std::endl;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>());
    //std::cout<<"Bounds Enforced: "<<s.getX()<<","<<s.getY()<<","<<s.getTheta()<<","<<s.getV()<<std::endl;
}

}


#endif /* CARSYSTEM_H_ */
