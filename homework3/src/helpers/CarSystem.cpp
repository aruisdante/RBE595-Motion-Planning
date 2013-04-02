/**
 * @file   CarSystem.cpp
 *
 * @date   Apr 2, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
//************ LOCAL DEPENDANCIES ****************//
#include<homework3/CarSystem.h>
//***********    NAMESPACES     ****************//

using namespace homework3;

CarControlSpace::CarControlSpace(const ob::StateSpacePtr &stateSpace): oc::RealVectorControlSpace(stateSpace, 2)
{
    this->setName("CarControl"+getName());
}

CarStateSpace::StateType::StateType() : CompoundStateSpace::StateType(){}

double CarStateSpace::StateType::getX() const
{
    return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
}

double CarStateSpace::StateType::getY() const
{
    return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
}

double CarStateSpace::StateType::getTheta() const
{
    return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
}

double CarStateSpace::StateType::getV() const
{
    return as<ob::RealVectorStateSpace::StateType>(0)->values[3];
}

void CarStateSpace::StateType::setX(double x)
{
    as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
}

void CarStateSpace::StateType::setY(double y)
{
    as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
}

void CarStateSpace::StateType::setTheta(double theta)
{
    as<ob::RealVectorStateSpace::StateType>(0)->values[2] = theta;
}

void CarStateSpace::StateType::setV(double v)
{
    as<ob::RealVectorStateSpace::StateType>(0)->values[3] = v;
}

CarStateSpace::CarStateSpace() : CompoundStateSpace()
{
    this->setName("CarSpace"+getName());
    this->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    this->lock();
}

void CarStateSpace::setBounds(const ob::RealVectorBounds &bounds)
{
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

const ob::RealVectorBounds& CarStateSpace::getBounds() const
{
    return as<ob::RealVectorStateSpace>(0)->getBounds();
}
