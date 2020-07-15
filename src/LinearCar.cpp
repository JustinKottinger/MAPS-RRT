#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
// std includes
#include <iostream>
#include <valarray>
#include <limits>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
// my includes
#include "../includes/LinearCar.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bg = boost::geometry;
namespace trans = boost::geometry::strategy::transform;


// this function is used for any 2D projection needed
// include but not limited to: obs checking, and path segmenting
std::vector<polygon> TwoLinearCars::GetPolygons()
{
    std::vector<polygon> Vehicles;
    // *********************
    // ***** vehicle 1 *****
    // *********************

    // create the shape of the vehicle centered at (0, 0)
    // point A1 = [-0.5l, -0.5w]
    point BackR1( -0.5 * carLength_, -0.5 * carWidth_);
    // point B1 = [-0.5l, +0.5w]
    point BackL1( (-0.5 * carLength_), (0.5 * carWidth_));
    // point C1 = [+0.5l, +0.5w]
    point FrontL1( (0.5 * carLength_), (0.5 * carWidth_));
    // point B1 = [+0.5l, -0.5w]
    point FrontR1( (0.5 * carLength_), (-0.5 * carWidth_));

    // now, translate the polygon to the state location
    trans::translate_transformer<double, 2, 2> translate1(xyState1_->values[0], xyState1_->values[1]);

    boost::geometry::transform(BackR1, BackR1, translate1);
    boost::geometry::transform(BackL1, BackL1, translate1);
    boost::geometry::transform(FrontL1, FrontL1, translate1);
    boost::geometry::transform(FrontR1, FrontR1, translate1);

    // create instance of polygon
    polygon v1;
    // // add the outer points to the shape
    v1.outer().push_back(BackR1);
    v1.outer().push_back(BackL1);
    v1.outer().push_back(FrontL1);
    v1.outer().push_back(FrontR1);
    v1.outer().push_back(BackR1);

    Vehicles.push_back(v1);

    // // *********************
    // // ***** vehicle 2 *****
    // // *********************

    point BackR2( -0.5 * carLength_, -0.5 * carWidth_);
    // point B1 = [-0.5l, +0.5w]
    point BackL2( (-0.5 * carLength_), (0.5 * carWidth_));
    // point C1 = [+0.5l, +0.5w]
    point FrontL2( (0.5 * carLength_), (0.5 * carWidth_));
    // point B1 = [+0.5l, -0.5w]
    point FrontR2( (0.5 * carLength_), (-0.5 * carWidth_));

    // now, translate the polygon to the state location
    trans::translate_transformer<double, 2, 2> translate2(xyState2_->values[0], xyState2_->values[1]);

    boost::geometry::transform(BackR2, BackR2, translate2);
    boost::geometry::transform(BackL2, BackL2, translate2);
    boost::geometry::transform(FrontL2, FrontL2, translate2);
    boost::geometry::transform(FrontR2, FrontR2, translate2);

    // create instance of polygon
    polygon v2;
    // // add the outer points to the shape
    v2.outer().push_back(BackR2);
    v2.outer().push_back(BackL2);
    v2.outer().push_back(FrontL2);
    v2.outer().push_back(FrontR2);
    v2.outer().push_back(BackR2);

    Vehicles.push_back(v2);

    return Vehicles;
}

std::vector<polygon> ThreeLinearCars::GetPolygons()
{
    std::vector<polygon> Vehicles;
    // *********************
    // ***** vehicle 1 *****
    // *********************

    // create the shape of the vehicle centered at (0, 0)
    // point A1 = [-0.5l, -0.5w]
    point BackR1( -0.5 * carLength_, -0.5 * carWidth_);
    // point B1 = [-0.5l, +0.5w]
    point BackL1( (-0.5 * carLength_), (0.5 * carWidth_));
    // point C1 = [+0.5l, +0.5w]
    point FrontL1( (0.5 * carLength_), (0.5 * carWidth_));
    // point B1 = [+0.5l, -0.5w]
    point FrontR1( (0.5 * carLength_), (-0.5 * carWidth_));

    // now, translate the polygon to the state location
    trans::translate_transformer<double, 2, 2> translate1(xyState1_->values[0], xyState1_->values[1]);

    boost::geometry::transform(BackR1, BackR1, translate1);
    boost::geometry::transform(BackL1, BackL1, translate1);
    boost::geometry::transform(FrontL1, FrontL1, translate1);
    boost::geometry::transform(FrontR1, FrontR1, translate1);

    // create instance of polygon
    polygon v1;
    // // add the outer points to the shape
    v1.outer().push_back(BackR1);
    v1.outer().push_back(BackL1);
    v1.outer().push_back(FrontL1);
    v1.outer().push_back(FrontR1);
    v1.outer().push_back(BackR1);

    Vehicles.push_back(v1);

    // // *********************
    // // ***** vehicle 2 *****
    // // *********************

    point BackR2( -0.5 * carLength_, -0.5 * carWidth_);
    // point B1 = [-0.5l, +0.5w]
    point BackL2( (-0.5 * carLength_), (0.5 * carWidth_));
    // point C1 = [+0.5l, +0.5w]
    point FrontL2( (0.5 * carLength_), (0.5 * carWidth_));
    // point B1 = [+0.5l, -0.5w]
    point FrontR2( (0.5 * carLength_), (-0.5 * carWidth_));

    // now, translate the polygon to the state location
    trans::translate_transformer<double, 2, 2> translate2(xyState2_->values[0], xyState2_->values[1]);

    boost::geometry::transform(BackR2, BackR2, translate2);
    boost::geometry::transform(BackL2, BackL2, translate2);
    boost::geometry::transform(FrontL2, FrontL2, translate2);
    boost::geometry::transform(FrontR2, FrontR2, translate2);

    // create instance of polygon
    polygon v2;
    // // add the outer points to the shape
    v2.outer().push_back(BackR2);
    v2.outer().push_back(BackL2);
    v2.outer().push_back(FrontL2);
    v2.outer().push_back(FrontR2);
    v2.outer().push_back(BackR2);

    Vehicles.push_back(v2);

    // // *********************
    // // ***** vehicle 3 *****
    // // *********************

    point BackR3( -0.5 * carLength_, -0.5 * carWidth_);
    // point B1 = [-0.5l, +0.5w]
    point BackL3( (-0.5 * carLength_), (0.5 * carWidth_));
    // point C1 = [+0.5l, +0.5w]
    point FrontL3( (0.5 * carLength_), (0.5 * carWidth_));
    // point B1 = [+0.5l, -0.5w]
    point FrontR3( (0.5 * carLength_), (-0.5 * carWidth_));

    // now, translate the polygon to the state location
    trans::translate_transformer<double, 2, 2> translate3(xyState3_->values[0], xyState3_->values[1]);

    boost::geometry::transform(BackR3, BackR3, translate3);
    boost::geometry::transform(BackL3, BackL3, translate3);
    boost::geometry::transform(FrontL3, FrontL3, translate3);
    boost::geometry::transform(FrontR3, FrontR3, translate3);

    // create instance of polygon
    polygon v3;
    // // add the outer points to the shape
    v3.outer().push_back(BackR3);
    v3.outer().push_back(BackL3);
    v3.outer().push_back(FrontL3);
    v3.outer().push_back(FrontR3);
    v3.outer().push_back(BackR3);

    Vehicles.push_back(v3);

    return Vehicles;
}

// 2 agent linear ODE function
void TwoLinearCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // std::cout << "here" << std::endl;
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // q = x1, y1, x2, y2
    // c = vx1 vy1 vx2 vy2

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // vehicle 1
    qdot[0] = u[0];
    qdot[1] = u[1];
    // vehicle 2
    qdot[2] = u[2];
    qdot[3] = u[3];
}

// 3 agent linear ODE function
void ThreeLinearCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // q = x1, y1, x2, y2, x3, y3
    // c = vx1 vy1 vx2 vy2, vx3, vy3

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // vehicle 1
    qdot[0] = u[0];
    qdot[1] = u[1];
    // vehicle 2
    qdot[2] = u[2];
    qdot[3] = u[3];
    // vehicle 3
    qdot[4] = u[4];
    qdot[5] = u[5];
}








