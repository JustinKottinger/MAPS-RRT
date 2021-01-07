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
#include "../includes/ReadWorld.h"
#include "../includes/KinematicCar.h"
#include "../includes/LinearCar.h"
#include "../includes/Unicycle.h"


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
    else if (model == "mlExample")
    {
        // real vector space for x y z 
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for psi
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for theta v
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);

        // set the bounds for the first vehicle 
        ob::RealVectorBounds bounds1(3);
        bounds1.setLow(0, bndry[0]); // x lower bound
        bounds1.setHigh(0, bndry[6]); // x upper bound
        bounds1.setLow(1, bndry[1]);  // y lower bound
        bounds1.setHigh(1, bndry[7]); //y upper bound
        bounds1.setLow(2, bndry[2]);  // z lower bound
        bounds1.setHigh(2, bndry[8]); // z upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(bounds1);

        // set the bounds for the first vehicle 
        ob::RealVectorBounds bounds2(2);
        bounds2.setLow(0, -pi / 3); // theta lower bound
        bounds2.setHigh(0, pi / 3); // theta upper bound
        bounds2.setLow(1, bndry[5]);  // v lower bound
        bounds2.setHigh(1, bndry[11]); // v upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(bounds2);

        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 3));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(3);
        Cbounds.setLow(0, -pi/6.0);  // omega lower bound
        Cbounds.setHigh(0, pi/6.0);  // omega upper bound
        Cbounds.setLow(1, -pi/6.0);  // phi1 lower bound
        Cbounds.setHigh(1, pi/6.0);  // phi1 upper bound
        Cbounds.setLow(2, -1);  // v2 lower bound
        Cbounds.setHigh(2, 1);  // v2 upper bound

        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &mlExampleODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_mlExample));

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


    }
    else if (model == "3KinematicCars")
    {
        // ********************************************************************
        // ************************* 3 Kinematic Cars *************************
        // ********************************************************************

        // add subspace for 3 vehcicles
        // real vector space for x1 y1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // next, add a SO2StateSpace for theta1
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x2 y2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // next, add a SO2StateSpace for theta2
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x3 y3
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // next, add a SO2StateSpace for theta3
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

        // std::cout << bndry[0] << bndry[3] << std::endl;
        // std::cout << bndry[1] << bndry[4] << std::endl;

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

        // create another instance of the bounds
        ob::RealVectorBounds boundsV3(2);
        boundsV3.setLow(0, bndry[0]);  // x lower bound
        boundsV3.setHigh(0, bndry[3]);  // y upper bound
        boundsV3.setLow(1, bndry[1]);  // y lower bound
        boundsV3.setHigh(1, bndry[4]); //y upper bound

        // set the bounds of this space 
        // note the indexing change
        cs->as<ob::RealVectorStateSpace>(4)->setBounds(boundsV3);

        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Kinematic car coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars.
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 6));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(6);
        Cbounds.setLow(0, -1);  // v1 lower bound
        Cbounds.setHigh(0, 1);  // v1 upper bound
        Cbounds.setLow(1, -pi/6.0);  // phi1 lower bound
        Cbounds.setHigh(1, pi/6.0);  // phi1 upper bound
        Cbounds.setLow(2, -1);  // v2 lower bound
        Cbounds.setHigh(2, 1);  // v2 upper bound
        Cbounds.setLow(3, -pi/6.0);  // phi2 lower bound
        Cbounds.setHigh(3, pi/6.0);  // phi2 upper bound
        Cbounds.setLow(4, -1);  // v2 lower bound
        Cbounds.setHigh(4, 1);  // v2 upper bound
        Cbounds.setLow(5, -pi/6.0);  // phi2 lower bound
        Cbounds.setHigh(5, pi/6.0);  // phi2 upper bound


        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &ThreeKinematicCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_ThreeKinematicCars));


        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];
        start[8] = strt[8];


        ss->setStartState(start);

        std::cout << "exiting Simple Set-Up" << std::endl;


    }
    else if (model == "2Linear")
    {
    	// ********************************************************************
        // ************************* 2 Linear *********************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // real vector space for x2 y2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(2);
        boundsV1.setLow(0, bndry[0]); // x lower bound
        boundsV1.setHigh(0, bndry[2]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[3]); //y upper bound


        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 
        // create another instance of the bounds
        ob::RealVectorBounds boundsV2(2);
        boundsV2.setLow(0, bndry[0]); // x lower bound
        boundsV2.setHigh(0, bndry[2]); // x upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[3]); //y upper bound

        // set the bounds of this space 
        // note the indexing change
        cs->as<ob::RealVectorStateSpace>(1)->setBounds(boundsV2);

        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Linear Dyn coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars, each with two velocity controls
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 4));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(4);
        Cbounds.setLow(0, -1);  // vx1 lower bound
        Cbounds.setHigh(0, 1);  // vx1 upper bound
        Cbounds.setLow(1, -1);  // vy1 lower bound
        Cbounds.setHigh(1, 1);  // vy1 upper bound
		Cbounds.setLow(2, -1);  // vx2 lower bound
        Cbounds.setHigh(2, 1);  // vx2 upper bound
        Cbounds.setLow(3, -1);  // vy2 lower bound
        Cbounds.setHigh(3, 1);  // vy2 upper bound

        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &TwoLinearCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve));

        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];

        ss->setStartState(start);

    }
    else if (model == "3Linear")
    {
    	// ********************************************************************
        // ************************* 3 Linear *********************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // real vector space for x2 y2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
        // real vector space for x3 y3
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        MyPrint(bndry);
        ob::RealVectorBounds boundsV1(2);
        boundsV1.setLow(0, bndry[0]); // x lower bound
        boundsV1.setHigh(0, bndry[2]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[3]); //y upper bound
        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1);


        // create another instance of the bounds
        ob::RealVectorBounds boundsV2(2);
        boundsV2.setLow(0, bndry[0]); // x lower bound
        boundsV2.setHigh(0, bndry[2]); // x upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[3]); //y upper bound
        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(1)->setBounds(boundsV2);

        // create another instance of the bounds
        ob::RealVectorBounds boundsV3(2);
        boundsV3.setLow(0, bndry[0]); // x lower bound
        boundsV3.setHigh(0, bndry[2]); // x upper bound
        boundsV3.setLow(1, bndry[1]);  // y lower bound
        boundsV3.setHigh(1, bndry[3]); //y upper bound
        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(boundsV3);

        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Linear Dyn coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 6 because we have three cars, each with two velocity controls
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 6));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(6);
        Cbounds.setLow(0, -1);  // vx1 lower bound
        Cbounds.setHigh(0, 1);  // vx1 upper bound
        Cbounds.setLow(1, -1);  // vy1 lower bound
        Cbounds.setHigh(1, 1);  // vy1 upper bound
		Cbounds.setLow(2, -1);  // vx2 lower bound
        Cbounds.setHigh(2, 1);  // vx2 upper bound
        Cbounds.setLow(3, -1);  // vy2 lower bound
        Cbounds.setHigh(3, 1);  // vy2 upper bound
        Cbounds.setLow(4, -1);  // vx3 lower bound
        Cbounds.setHigh(4, 1);  // vx3 upper bound
        Cbounds.setLow(5, -1);  // vy3 lower bound
        Cbounds.setHigh(5, 1);  // vy3 upper bound

        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars
        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &ThreeLinearCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve));

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

    }
    else if (model == "3Unicycle")
    {

        // ********************************************************************
        // ************************* 3 Unicycle *******************************
        // ********************************************************************

        // add subspace for 3 vehcicles
        // real vector space for x1 y1 v1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta1
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x2 y2 v2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta2
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x3 y3 v3
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta3
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(3);
        boundsV1.setLow(0, bndry[0]); // x lower bound
        boundsV1.setHigh(0, bndry[4]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[5]); //y upper bound
        boundsV1.setLow(2, bndry[2]);  // y lower bound
        boundsV1.setHigh(2, bndry[6]); //y upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 

        // set the bounds for the second vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV2(3);
        boundsV2.setLow(0, bndry[0]); // x lower bound
        boundsV2.setHigh(0, bndry[4]); // x upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[5]); //y upper bound
        boundsV2.setLow(2, bndry[2]);  // y lower bound
        boundsV2.setHigh(2, bndry[6]); //y upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(boundsV2); 

        // set the bounds for the second vehicle 
        // they will be the same as vehicle 3 but adding this way is more intuitve
        ob::RealVectorBounds boundsV3(3);
        boundsV3.setLow(0, bndry[0]); // x lower bound
        boundsV3.setHigh(0, bndry[4]); // x upper bound
        boundsV3.setLow(1, bndry[1]);  // y lower bound
        boundsV3.setHigh(1, bndry[5]); //y upper bound
        boundsV3.setLow(2, bndry[2]);  // y lower bound
        boundsV3.setHigh(2, bndry[6]); //y upper bound


        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(4)->setBounds(boundsV3);


        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Kinematic car coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars.
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 6));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(6);
        // vehicle 1
        Cbounds.setLow(0, -1);  // 1 sigma_dot lower bound
        Cbounds.setHigh(0, 1);  // 1 sigma_dot upper bound
        Cbounds.setLow(1, -1);  // 1 omega_dot lower bound
        Cbounds.setHigh(1, 1);  // 1 omega_dot upper bound
        // vehicle 2
        Cbounds.setLow(2, -1);  // 2 sigma_dot lower bound
        Cbounds.setHigh(2, 1);  // 2 sigma_dot upper bound
        Cbounds.setLow(3, -1);  // 2 omega_dot lower bound
        Cbounds.setHigh(3, 1);  // 2 omega_dot upper bound
        // vehicle 3
        Cbounds.setLow(4, -1);  // 3 sigma_dot lower bound
        Cbounds.setHigh(4, 1);  // 3 sigma_dot upper bound
        Cbounds.setLow(5, -1);  // 3 omega_dot lower bound
        Cbounds.setHigh(5, 1);  // 3 omega_dot upper bound


        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars
        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &ThreeUnicyclesODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_ThreeUnicycles));

        // set start and goal states
        // analogous to setting the bounds
        std::cout << strt[0] << strt[1] << strt[2] << strt[3] << strt[4] << strt[5] << strt[6]
       << strt[7]<< strt[8] << strt[9] << strt[10] << strt[11] << std::endl;
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];
        start[8] = strt[8];
        start[9] = strt[9];
        start[10] = strt[10];
        start[11] = strt[11];

        ss->setStartState(start);        
    }
    else if (model == "2Unicycle")
    {

        // ********************************************************************
        // ************************* 2 Unicycle *******************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1 v1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta1
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x2 y2 v2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta2
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(3);
        boundsV1.setLow(0, bndry[0]); // x lower bound
        boundsV1.setHigh(0, bndry[4]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[5]); //y upper bound
        boundsV1.setLow(2, bndry[2]);  // y lower bound
        boundsV1.setHigh(2, bndry[6]); //y upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 

        // set the bounds for the second vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV2(3);
        boundsV2.setLow(0, bndry[0]); // x lower bound
        boundsV2.setHigh(0, bndry[4]); // x upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[5]); //y upper bound
        boundsV2.setLow(2, bndry[2]);  // y lower bound
        boundsV2.setHigh(2, bndry[6]); //y upper bound

        // set the bounds of this space 
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
        // vehicle 1
        Cbounds.setLow(0, -1);  // 1 sigma_dot lower bound
        Cbounds.setHigh(0, 1);  // 1 sigma_dot upper bound
        Cbounds.setLow(1, -1);  // 1 omega_dot lower bound
        Cbounds.setHigh(1, 1);  // 1 omega_dot upper bound
        // vehicle 2
        Cbounds.setLow(2, -1);  // 2 sigma_dot lower bound
        Cbounds.setHigh(2, 1);  // 2 sigma_dot upper bound
        Cbounds.setLow(3, -1);  // 2 omega_dot lower bound
        Cbounds.setHigh(3, 1);  // 2 omega_dot upper bound
        


        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars
        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &TwoUnicyclesODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_TwoUnicycles));

        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];

        ss->setStartState(start);        
    }
    else if (model == "2Unicycle2ndOrder")
    {

        // ********************************************************************
        // ******************** 2 2nd Order Unicycle **************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1 v1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta1
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // next, add a SO2StateSpace for omega1 (angular vel) [-1, 1]
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.0);
        // real vector space for x2 y2 v2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.0);
        // next, add a SO2StateSpace for theta2
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // next, add a SO2StateSpace for omega2 (angular vel)
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(3);
        boundsV1.setLow(0, bndry[0]); // x lower bound
        boundsV1.setHigh(0, bndry[5]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[6]); // y upper bound
        boundsV1.setLow(2, bndry[2]);  // v lower bound
        boundsV1.setHigh(2, bndry[7]); //v upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 

        ob::RealVectorBounds boundsOmega(1);
        boundsOmega.setLow(0, -1);
        boundsOmega.setHigh(0, 1);

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(boundsOmega); 
        cs->as<ob::RealVectorStateSpace>(5)->setBounds(boundsOmega);


        // set the bounds for the second vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV2(3);
        boundsV2.setLow(0, bndry[0]); // x lower bound
        boundsV2.setHigh(0, bndry[5]); // x upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[6]); //y upper bound
        boundsV2.setLow(2, bndry[2]);  // y lower bound
        boundsV2.setHigh(2, bndry[7]); //y upper bound

        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(3)->setBounds(boundsV2); 


        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Kinematic car coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars.
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 4));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(4);
        // vehicle 1
        Cbounds.setLow(0, -1);  // 1 sigma_dot lower bound
        Cbounds.setHigh(0, 1);  // 1 sigma_dot upper bound
        Cbounds.setLow(1, -1);  // 1 omega_dot lower bound
        Cbounds.setHigh(1, 1);  // 1 omega_dot upper bound
        // vehicle 2
        Cbounds.setLow(2, -1);  // 2 sigma_dot lower bound
        Cbounds.setHigh(2, 1);  // 2 sigma_dot upper bound
        Cbounds.setLow(3, -1);  // 2 omega_dot lower bound
        Cbounds.setHigh(3, 1);  // 2 omega_dot upper bound
        


        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars
        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &Two2ndOrderUnicyclesODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_Two2ndOrderUnicycles));

        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];
        start[8] = strt[8];
        start[9] = strt[9];

        ss->setStartState(start);        
    }
    else if (model == "2Linear2ndOrder")
    {
        // ********************************************************************
        // ********************* 2 Linear 2nd Order ***************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1 vx1 vy1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
        // real vector space for x2 y2 vx2 vy2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(4);
        boundsV1.setLow(0, bndry[0]);  // x lower bound
        boundsV1.setHigh(0, bndry[4]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[5]); // y upper bound
        boundsV1.setLow(2, bndry[2]);  // vx lower bound
        boundsV1.setHigh(2, bndry[6]); // vx upper bound
        boundsV1.setLow(3, bndry[3]);  // vy lower bound
        boundsV1.setHigh(3, bndry[7]); // vy upper bound


        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 
        // create another instance of the bounds
        ob::RealVectorBounds boundsV2(4);
        boundsV2.setLow(0, bndry[0]);  // x lower bound
        boundsV2.setHigh(0, bndry[4]); // x upper bound
        boundsV2.setLow(1, bndry[1]);  // y lower bound
        boundsV2.setHigh(1, bndry[5]); // y upper bound
        boundsV2.setLow(2, bndry[2]);  // vx lower bound
        boundsV2.setHigh(2, bndry[6]); // vx upper bound
        boundsV2.setLow(3, bndry[3]);  // vy lower bound
        boundsV2.setHigh(3, bndry[7]); // vy upper bound

        // set the bounds of this space 
        // note the indexing change
        cs->as<ob::RealVectorStateSpace>(1)->setBounds(boundsV2);

        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Linear Dyn coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars, each with two velocity controls
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 4));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(4);
        Cbounds.setLow(0, -1);  // ax1 lower bound
        Cbounds.setHigh(0, 1);  // ax1 upper bound
        Cbounds.setLow(1, -1);  // ay1 lower bound
        Cbounds.setHigh(1, 1);  // ay1 upper bound
        Cbounds.setLow(2, -1);  // ax2 lower bound
        Cbounds.setHigh(2, 1);  // ax2 upper bound
        Cbounds.setLow(3, -1);  // ay2 lower bound
        Cbounds.setHigh(3, 1);  // ay2 upper bound

        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &Two2ndOrderLinearCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve));

        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];

        ss->setStartState(start);
    }
    else if (model == "2KinematicCars2ndOrder")
    {
        // ********************************************************************
        // ************************* 2 Kinematic Cars *************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1 v1 phi1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
        // next, add a SO2StateSpace for theta1
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
        // real vector space for x2 y2 v2 phi2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
        // next, add a SO2StateSpace for theta2
        cs->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsVehicles(4);
        boundsVehicles.setLow(0, bndry[0]); //  x lower bound
        boundsVehicles.setHigh(0, bndry[5]); // x upper bound
        boundsVehicles.setLow(1, bndry[1]);  // y lower bound
        boundsVehicles.setHigh(1, bndry[6]); // y upper bound
        boundsVehicles.setLow(2, bndry[2]);  // v lower bound
        boundsVehicles.setHigh(2, bndry[7]); // v upper bound
        boundsVehicles.setLow(3, bndry[3]);  // phi lower bound
        boundsVehicles.setHigh(3, bndry[8]); // phi upper bound


        // set the bounds of this space 
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsVehicles); 

        // set the bounds of this space 
        // note the indexing change
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(boundsVehicles);
        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Kinematic car coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars.
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 4));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(4);
        Cbounds.setLow(0, -1);  // a1 lower bound
        Cbounds.setHigh(0, 1);  // a1 upper bound
        Cbounds.setLow(1, -1);  // phi_ddot1 lower bound
        Cbounds.setHigh(1, 1);  // phi_ddot1 upper bound
        Cbounds.setLow(2, -1);  // a2 lower bound
        Cbounds.setHigh(2, 1);  // a2 upper bound
        Cbounds.setLow(3, -1);  // phi_ddot2 lower bound
        Cbounds.setHigh(3, 1);  // phi_ddot2 upper bound


        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            Two2ndOrderCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve, 
            &postProp_Two2ndOrderCars));


        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];
        start[8] = strt[8];
        start[9] = strt[9];

        ss->setStartState(start);
    }
    else if (model == "3Linear2ndOrder")
    {
        // ********************************************************************
        // ********************* 2 Linear 2nd Order ***************************
        // ********************************************************************

        // add subspace for 2 vehcicles
        // real vector space for x1 y1 vx1 vy1
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
        // real vector space for x2 y2 vx2 vy2
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
        // real vector space for x3 y3 vx3 vy3
        cs->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);

        // set the bounds for the first vehicle 
        // they will be the same as vehicle 2 but adding this way is more intuitve
        ob::RealVectorBounds boundsV1(4);
        boundsV1.setLow(0, bndry[0]);  // x lower bound
        boundsV1.setHigh(0, bndry[4]); // x upper bound
        boundsV1.setLow(1, bndry[1]);  // y lower bound
        boundsV1.setHigh(1, bndry[5]); // y upper bound
        boundsV1.setLow(2, bndry[2]);  // vx lower bound
        boundsV1.setHigh(2, bndry[6]); // vx upper bound
        boundsV1.setLow(3, bndry[3]);  // vy lower bound
        boundsV1.setHigh(3, bndry[7]); // vy upper bound


        // set the bounds of the spaces
        cs->as<ob::RealVectorStateSpace>(0)->setBounds(boundsV1); 
        cs->as<ob::RealVectorStateSpace>(1)->setBounds(boundsV1);
        cs->as<ob::RealVectorStateSpace>(2)->setBounds(boundsV1);

        // now, we need to create a control space
        // the realvectorcontrolspace constructor needs the state space it is in and the dimension
        //      of the control space
        // Note the controls of a Linear Dyn coorespond to the compound state space we defined 
        // above. The constructor actually takes in the pointer so, "space" in my case.
        //  The dimension is also 4 because we have two cars, each with two velocity controls
        oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 6));
        // creat another instance of the bounds
        ob::RealVectorBounds Cbounds(6);
        Cbounds.setLow(0, -1);  // ax1 lower bound
        Cbounds.setHigh(0, 1);  // ax1 upper bound
        Cbounds.setLow(1, -1);  // ay1 lower bound
        Cbounds.setHigh(1, 1);  // ay1 upper bound
        Cbounds.setLow(2, -1);  // ax2 lower bound
        Cbounds.setHigh(2, 1);  // ax2 upper bound
        Cbounds.setLow(3, -1);  // ay2 lower bound
        Cbounds.setHigh(3, 1);  // ay2 upper bound
        Cbounds.setLow(4, -1);  // ax3 lower bound
        Cbounds.setHigh(4, 1);  // ax3 upper bound
        Cbounds.setLow(5, -1);  // ay4 lower bound
        Cbounds.setHigh(5, 1);  // ay4 upper bound

        // Just as before, set the bounds to the proper space, this time, it is cspace and not space
        // notice that () is empty, this is because no indexing is needed because 
        // there is just 1 control space
        cspace->as<oc::RealVectorControlSpace>()->setBounds(Cbounds);

        // update the simple setup instance with the space we set up
        ss.reset(new oc::SimpleSetup(cspace));

        // set state propogator for multiplecars

        auto odeSolve(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), 
            &Three2ndOrderLinearCarsODE));

        ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolve));

        // set start and goal states
        // analogous to setting the bounds
        ob::ScopedState<> start(space);
        start[0] = strt[0];
        start[1] = strt[1];
        start[2] = strt[2];
        start[3] = strt[3];
        start[4] = strt[4];
        start[5] = strt[5];
        start[6] = strt[6];
        start[7] = strt[7];
        start[8] = strt[8];
        start[9] = strt[9];
        start[10] = strt[10];
        start[11] = strt[11];

        ss->setStartState(start);
    }
    else
    {
        std::cout << "No matching model. Exiting Program to avoid error" << std::endl;
        exit(1);
    }
    
}







