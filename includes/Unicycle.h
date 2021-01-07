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
#include <boost/numeric/odeint/util/is_resizeable.hpp>


// #ifndef TwoKinematicCarsModel
// #define TwoKinematicCarsModel


namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::polygon<point> polygon;
typedef boost::geometry::model::segment<point> Segment;
typedef boost::array< double , 6 > state_type;





// this is the class I want to use to project my states into 2Dprojections of vehicles
class ThreeUnicycleModels
{
    public:
        ThreeUnicycleModels(const ob::State *state)
        {
            // get the compound space
            cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
            // gather the xy-position of both vehicles
            xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
            xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
            xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);
            // gather the theta value of both vehicles
            rot1_ = cs_->as<ob::SO2StateSpace::StateType>(1);
            rot2_ = cs_->as<ob::SO2StateSpace::StateType>(3);
            rot3_ = cs_->as<ob::SO2StateSpace::StateType>(5);

            const int NumVs_ = 3;
            carLength_ = 0.01;
            carWidth_ = 0.01;
        }
        // 

        // this function reutrns a vector of polygons in their respective states
        std::vector<polygon> GetPolygons();

    protected:
        const ob::SO2StateSpace::StateType *rot1_;
        const ob::SO2StateSpace::StateType *rot2_;
        const ob::SO2StateSpace::StateType *rot3_;
        const ob::RealVectorStateSpace::StateType *xyState1_;
        const ob::RealVectorStateSpace::StateType *xyState2_;
        const ob::RealVectorStateSpace::StateType *xyState3_;
        const ompl::base::CompoundStateSpace::StateType *cs_;
        // const int NumVs_;         
        double carLength_;
        double carWidth_;
        
};

class Two2ndOrderUnicycleModels
{
    public:
        Two2ndOrderUnicycleModels(const ob::State *state)
        {
            // get the compound space
            cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
            // gather the xy-position of both vehicles
            xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
            xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(3);
            // gather the theta value of both vehicles
            rot1_ = cs_->as<ob::SO2StateSpace::StateType>(1);
            rot2_ = cs_->as<ob::SO2StateSpace::StateType>(4);

            const int NumVs_ = 2;
            carLength_ = 0.2;
            carWidth_ = 0.2;
        }
        // this function reutrns a vector of polygons in their respective states
        std::vector<polygon> GetPolygons();

    protected:
        const ob::SO2StateSpace::StateType *rot1_;
        const ob::SO2StateSpace::StateType *rot2_;
        const ob::RealVectorStateSpace::StateType *xyState1_;
        const ob::RealVectorStateSpace::StateType *xyState2_;
        const ompl::base::CompoundStateSpace::StateType *cs_;
        // const int NumVs_;         
        double carLength_;
        double carWidth_;
        
};

// this is the class I want to use to project my states into 2Dprojections of vehicles
class TwoUnicycleModels
{
    public:
        TwoUnicycleModels(const ob::State *state)
        {
            // get the compound space
            cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
            // gather the xy-position of both vehicles
            xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
            xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
            // gather the theta value of both vehicles
            rot1_ = cs_->as<ob::SO2StateSpace::StateType>(1);
            rot2_ = cs_->as<ob::SO2StateSpace::StateType>(3);

            const int NumVs_ = 2;
            carLength_ = 0.5;
            carWidth_ = 0.5;
        }
        // 

        // this function reutrns a vector of polygons in their respective states
        std::vector<polygon> GetPolygons();

    protected:
        const ob::SO2StateSpace::StateType *rot1_;
        const ob::SO2StateSpace::StateType *rot2_;
        const ob::RealVectorStateSpace::StateType *xyState1_;
        const ob::RealVectorStateSpace::StateType *xyState2_;
        const ompl::base::CompoundStateSpace::StateType *cs_;
        // const int NumVs_;         
        double carLength_;
        double carWidth_;
        
};

// Definition of ODe for 2kinematic car ODE
void ThreeUnicyclesODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);

// This is a callback method invoked after numerical integration.
// normalizes angle for two vehicles
void postProp_ThreeUnicycles(const ob::State *q, const oc::Control *ctl, 
    const double duration, ob::State *qnext);

// Definition of ODe for 2kinematic car ODE
void TwoUnicyclesODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);

// This is a callback method invoked after numerical integration.
// normalizes angle for two vehicles
void postProp_TwoUnicycles(const ob::State *q, const oc::Control *ctl, 
    const double duration, ob::State *qnext);

// Definition of ODE for 2 2nd order unicycle
void Two2ndOrderUnicyclesODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);

// This is a callback method invoked after numerical integration.
// normalizes angle for two vehicles
void postProp_Two2ndOrderUnicycles(const ob::State *q, const oc::Control *ctl, 
    const double duration, ob::State *qnext);




