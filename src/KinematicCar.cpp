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

namespace ob = ompl::base;
namespace oc = ompl::control;

// Kinematic car model object definition.  This class does NOT use ODESolver to propagate the system.
// class KinematicCarModel : public oc::StatePropagator
// {
//     public:
//         KinematicCarModel(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
//         {
//            space_     = si->getStateSpace();
//            carLength_ = 0.2;
//            timeStep_  = 0.01;
//         }

//         std::string data_;

//          void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override
//         {
//             EulerIntegration(state, control, duration, result);
//         }

//     protected:
//         // Explicit Euler Method for numerical integration.
//         void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
//         {
//             double t = timeStep_;
//             std::valarray<double> dstate;
//             space_->copyState(result, start);
//             while (t < duration + std::numeric_limits<double>::epsilon())
//             {
//                 ode(result, control, dstate);
//                 update(result, timeStep_ * dstate);
//                 t += timeStep_;
//             }
//             if (t + std::numeric_limits<double>::epsilon() > duration)
//             {
//                 ode(result, control, dstate);
//                 update(result, (t - duration) * dstate);
//             }
//         }

//         void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
//         {
//             const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
//             const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

//             dstate.resize(3);
//             dstate[0] = u[0] * cos(theta);
//             dstate[1] = u[0] * sin(theta);
//             dstate[2] = u[0] * tan(u[1]) / carLength_;
//         }

//         void update(ob::State *state, const std::valarray<double> &dstate) const
//         {
//             ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
//             s.setX(s.getX() + dstate[0]);
//             s.setY(s.getY() + dstate[1]);
//             s.setYaw(s.getYaw() + dstate[2]);
//             space_->enforceBounds(state);
//         }

//         ob::StateSpacePtr        space_;
//         double                   carLength_;
//         double                   timeStep_;
        
// };

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;
}

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

// multi agent ODE function
void TwoKinematicCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // q = x1, y1, theta1, x2, y2, theta2
    // c = v1, phi1, v2, phi2 
    const double theta1 = q[2];
    const double theta2 = q[5];
    double carLength = 0.2;

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // vehicle 1
    qdot[0] = u[0] * cos(theta1);
    qdot[1] = u[0] * sin(theta1);
    qdot[2] = u[0] * tan(u[1]) / carLength;
    // vehicle 2
    qdot[3] = u[2] * cos(theta2);
    qdot[4] = u[2] * sin(theta2);
    qdot[5] = u[2] * tan(u[3]) / carLength;
}
// multi agent callback function
// this may not work properly
// need to check somehow
void postProp_TwoKinematicCars(const ob::State *q, const oc::Control *ctl, 
    const double duration, ob::State *qnext)
{
    //pull the angles from both cars
    ob::CompoundState* cs = qnext->as<ob::CompoundState>();

    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace::StateType* angleState2 = cs->as<ob::SO2StateSpace::StateType>(3);

    //use ompl to normalize theta
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
    SO2.enforceBounds(angleState2);

}








