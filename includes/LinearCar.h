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
class TwoLinearCars
{
    public:
        TwoLinearCars(const ob::State *state)
        {
            // get the compound space
            cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
            // gather the xy-position of both vehicles
            xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
            xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);

            const int NumVs_ = 2;
            carLength_ = 0.4;
            carWidth_ = 0.4;
        }
        // 

        // this function reutrns a vector of polygons in their respective states
        std::vector<polygon> GetPolygons();

    protected:
        const ob::RealVectorStateSpace::StateType *xyState1_;
        const ob::RealVectorStateSpace::StateType *xyState2_;
        const ompl::base::CompoundStateSpace::StateType *cs_;
        double carLength_;
        double carWidth_;
        
};

class ThreeLinearCars
{
    public:
        ThreeLinearCars(const ob::State *state)
        {
            // get the compound space
            cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
            // gather the xy-position of both vehicles
            xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
            xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
            xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

            const int NumVs_ = 3;
            carLength_ = 0.4;
            carWidth_ = 0.4;
        }

        // this function reutrns a vector of polygons in their respective states
        std::vector<polygon> GetPolygons();

    protected:
        const ob::RealVectorStateSpace::StateType *xyState1_;
        const ob::RealVectorStateSpace::StateType *xyState2_;
        const ob::RealVectorStateSpace::StateType *xyState3_;
        const ompl::base::CompoundStateSpace::StateType *cs_;
        // const int NumVs_;         
        double carLength_;
        double carWidth_;
        
};

void TwoLinearCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);

void ThreeLinearCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);

void Two2ndOrderLinearCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);


// #endif
