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
class TwoKinematicCarsModel
{
    public:
        TwoKinematicCarsModel(ob::State *state)
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
            carLength_ = 0.2;
            carWidth_ = 0.2;
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
void list_coordinates(point const& p);

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


// this class is currenlty being used to integrate during planning
// could be integrated into the class above in the future
class TwoKinCarsODE 
{
    const oc::Control* ControlInput;
    const double carLength = 0.2;

public:
    TwoKinCarsODE(const oc::Control* control) : ControlInput(control) { }

    void operator() ( const state_type &q , state_type &qdot , const double /* t */ )
    {
        const double *u = ControlInput->as<oc::RealVectorControlSpace::ControlType>()->values;
        // q = x1, y1, theta1, x2, y2, theta2
        // c = v1, phi1, v2, phi2 
        const double theta1 = q[2];
        const double theta2 = q[5];

        // Zero out qdot
        // qdot.resize (q.size (), 0);
        for (int i = 0; i < qdot.size(); i++)
        {
            qdot[i] = 0;
        }
        // vehicle 1
        qdot[0] = u[0] * cos(theta1);
        qdot[1] = u[0] * sin(theta1);
        qdot[2] = u[0] * tan(u[1]) / carLength;
        // vehicle 2
        qdot[3] = u[2] * cos(theta2);
        qdot[4] = u[2] * sin(theta2);
        qdot[5] = u[2] * tan(u[3]) / carLength;
    }
};


struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
        // std::cout << t << ' ' << x[0] << ' ' << x[1] << ' ' << x[2] << std::endl;
    }
};

// #endif
