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
 
//         void propagate(const ob::State *state, const oc::Control* control, 
//             const double duration, ob::State *result) const override;

//     protected:
//         // Explicit Euler Method for numerical integration.
//         void EulerIntegration(const ob::State *start, const oc::Control *control, 
//             const double duration, ob::State *result) const;
        
//         void ode(const ob::State *state, const oc::Control *control, 
//             std::valarray<double> &dstate) const;
        

//         void update(ob::State *state, const std::valarray<double> &dstate) const;
        
//         ob::StateSpacePtr        space_;
//         double                   carLength_;
//         double                   timeStep_;
        
// };

// Definition of the ODE for the kinematic car.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, 
    oc::ODESolver::StateType& qdot);


// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, 
const oc::Control* /*control*/, const double /*duration*/, ob::State *result);

// Definition of ODe for 2kinematic car ODE
void TwoKinematicCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);

// This is a callback method invoked after numerical integration.
// normalizes angle for two vehicles
void postProp_TwoKinematicCars(const ob::State *q, const oc::Control *ctl, 
    const double duration, ob::State *qnext);

