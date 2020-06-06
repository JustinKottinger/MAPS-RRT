// ompl includes
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/config.h"
#include <boost/math/constants/constants.hpp>
#include <ompl/control/ODESolver.h>

// my includes
#include "../includes/CreateSimpleSetup.h"
#include "../includes/KinematicCar.h"

namespace ob = ompl::base;
namespace oc = ompl::control;


void CreateSimpleSetup(oc::SimpleSetupPtr& ss, std::vector<double> bndry, 
	std::vector<double> gol, std::vector<double> strt, 
	std::string model, const double toll)
{    
    // we are going to compose our statespace with a collection of subspaces
    // note that it is possible to use build in spaces such as SE2StateSpace
    // however, I ofen deal with multi-agent planning so it is easier to just build a 
    // general statespace from the baseline one

    // initialize space

    ob::CompoundStateSpace *cs = new ob::CompoundStateSpace();

    // create a pointer to that space (name: space)

    ob::StateSpacePtr space(cs);

    // define pi
    const double pi = boost::math::constants::pi<double>();

    if (model == "KinematicCar")
    {
        // ********************************************************************
        // ************************* 1 Kinematic Car *************************
        // ********************************************************************

        // 1st order car consists of SE2StateSpace
        // first, add a RealVectorStateSpace for x, y 
        // addSubspace takes in the space type and weight
        // RealVectorStateSpace takes in the dimension 
        // weight is [0, 1]
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // next, add a SO2StateSpace for theta
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
            
        // create an instance of bounds and set the low and high for x and y
        ob::RealVectorBounds bounds(2);
        bounds.setLow(bndry[0]);
        bounds.setHigh(bndry[3]);
        // set the bounds to the state space
        // do this by indexing which space we want to set the bounds to 
        // RealVectorSpace is indexed 0 since it was first
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);        
        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Kinematic car coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 2 because the controls are velocity and steering rate. 
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

        // set the bounds of the control space
        // in the statespace, we got away with one argument since the bounds of x and y 
        // can be the same
        // Here however, the bounds of v and phi are different. Hence, we need to index 
        // which dimension we are setting bounds to. This will have a direct affect on how we read the
        // outputted text file
        // Control 1: velocity
        // Control 2: phi
        ob::RealVectorBounds Cbounds(2);
        Cbounds.setLow(0, -1);  // v lower bound
        Cbounds.setHigh(0, 1);  // v upper bound
        Cbounds.setLow(1, -pi/6.0);  // phi lower bound
        Cbounds.setHigh(1, pi/6.0);  // phi upper bound
        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);


        // reset the simplesetup instance with new spaces



        ss.reset(new oc::SimpleSetup(cspace));

        //  Tbh, I am not sure what this is doing, may come back to it
        // essnetially I am telling simple setup which function to call to check for collisions
        // i just do not know how yet

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &KinematicCarODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &KinematicCarPostIntegration));
            
        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];

        ob::ScopedState<> goal(space);
        // this has something to do with s and p, as it increase len of gol
        // mk note to figure out why? it is non-intuitive 
        goal[0] = gol[0];
        goal[1] = gol[1];
        goal[2] = gol[2];
        // provide the start, goal and tollorance to simple setup class

        ss->setStartAndGoalStates(start, goal, toll);

    }
    else if (model == "2KinematicCars")
    {
        // ********************************************************************
        // ************************* 2 Kinematic Cars *************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // next, add a SO2StateSpace for theta1
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x2 y2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // next, add a SO2StateSpace for theta2
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(2);
        boundsV1.setLow(0, bndry[0]); // x lower bound
        boundsV1.setHigh(0, bndry[3]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[4]); //y upper bound


        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 
        // create another instance of the bounds
        ob::RealVectorBounds boundsV2(2);
        boundsV2.setLow(0, bndry[0]);  // x lower bound
        boundsV2.setHigh(0, bndry[3]);  // y upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[4]); //y upper bound

        // set the bounds of this space 
        // note the indexing change
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(boundsV2);
        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Kinematic car coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars.
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 4));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(4);
        Cbounds.setLow(0, -1);  // v1 lower bound
        Cbounds.setHigh(0, 1);  // v1 upper bound
        Cbounds.setLow(1, -pi/6.0);  // phi1 lower bound
        Cbounds.setHigh(1, pi/6.0);  // phi1 upper bound
        Cbounds.setLow(2, -1);  // v2 lower bound
        Cbounds.setHigh(2, 1);  // v2 upper bound
        Cbounds.setLow(3, -pi/6.0);  // phi2 lower bound
        Cbounds.setHigh(3, pi/6.0);  // phi2 upper bound


        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &TwoKinematicCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_TwoKinematicCars));


        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];

        ss->setStartState(start);

        // ss->setGoal(goal);

        // ob::ScopedState<> goal(space);
        // goal[0] = gol[0];
        // goal[1] = gol[1];
        // goal[2] = gol[2];
        // goal[3] = gol[3];
        // goal[4] = gol[4];
        // goal[5] = gol[5];
        // provide the start, goal and tollorance to simple setup class

        // ss->setStartAndGoalStates(start, goal, toll);
        // std::cout << "exiting simple setup" << std::endl;

    }
    else
    {
        std::cout << "No matching model. Exiting Program to avoid error" << std::endl;
        exit(1);
    }
    
}







