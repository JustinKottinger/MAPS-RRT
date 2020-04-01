// ompl includes
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
// #include <ompl/control/planners/rrt/RRT.h>
// std includes
#include <iostream>
#include <valarray>
#include <limits>
// my includes
#include "../includes/ReadWorld.h"
#include "../includes/CreateSimpleSetup.h"
#include "../includes/KinematicCar.h"
#include "../includes/MyPlanner.h"


class KinematicCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    KinematicCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }

        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

        const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

        double x = pos->values[0];
        double y = pos->values[1];
            
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          if (x>(obstacle[i] - buffer_) && x<(obstacle[i] + obstacle[i+3] + buffer_) && 
            y>(obstacle[i+1] - buffer_) && y<(obstacle[i+1]+obstacle[i+4] + buffer_))
          {
            return false;
          }
                
        }    
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};

// used for 2 agent planning
class TwoKinematicCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    TwoKinematicCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }
        const ob::CompoundStateSpace::StateType *cs = state->as<ob::CompoundStateSpace::StateType>();

        // recall these are indexed exactly the same order as they are added in Compound state space
        // see CreateSimpleSetup to understand these indexes

        const auto *xyState1 = cs->as<ob::RealVectorStateSpace::StateType>(0);

        const auto *xyState2 = cs->as<ob::RealVectorStateSpace::StateType>(2);

        // const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

        // const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

        // const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        double x1 = xyState1->values[0];
        double y1 = xyState1->values[1];

        double x2 = xyState2->values[0];
        double y2 = xyState2->values[1];
            
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          if (x1>(obstacle[i] - buffer_) && x1<(obstacle[i] + obstacle[i+3] + buffer_) && 
            y1>(obstacle[i+1] - buffer_) && y1<(obstacle[i+1]+obstacle[i+4] + buffer_))
          {
            return false;
          }
          else if((x2>(obstacle[i] - buffer_) && x2<(obstacle[i] + obstacle[i+3] + buffer_) && 
                 y2>(obstacle[i+1] - buffer_) && y2<(obstacle[i+1]+obstacle[i+4] + buffer_)))
          {
            return false;
          }
          else if (abs(x1 - x2) <= 2*buffer_ && abs(y1 - y2) <= 2*buffer_)
          {
            return false;
          }
                
        }    
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};


void plan(std::vector<double> bndry, std::vector<double> gol, 
    std::vector<double> obs, std::vector<double> start, std::string model, 
    const double planningTime, const double Tollorance, std::vector<std::string> data)
{
    oc::SimpleSetupPtr ss;
    CreateSimpleSetup(ss, bndry, gol, start, model, Tollorance);

    const oc::SpaceInformationPtr si = ss->getSpaceInformation();

    if (model == "KinematicCar")
    {
      ss->setStateValidityChecker(std::make_shared<KinematicCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "2KinematicCars")
    {
      ss->setStateValidityChecker(std::make_shared<TwoKinematicCarStateValidityChecker>(
        si, obs));
    }
    
    
    ob::PlannerPtr planner(new oc::MAPSRRT(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    ss->setup();

    // ss->print(std::cout);

    ob::PlannerStatus solved = ss->solve(planningTime);

    if (solved)
    {
      std::cout << "Found solution:" << std::endl;

      oc::PathControl path = ss->getSolutionPath();
      std::ofstream PathFile;
      PathFile.open("txt/path.txt");
      if (PathFile.fail())
      {
        std::cerr << "ERROR: Could not open path.txt" << std::endl;
        exit(1);
      }
      else
      {
        std::cout << "Writing solution to path.txt" << std::endl;
        // PathFile << data << std::endl;
        path.printAsMatrix(PathFile);
        PathFile.close();
        std::cout << "Computation completed successfully" << std::endl;
        // path.print(std::cout);  // this prints out the solution
      }
    }
}


int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // Define the multi-agent problem from world file
    std::string DynModel;
    int dim;
    int NumVs;
    int NumCs;
    std::vector<std::string> data;
    std::vector<double> bndry;
    std::vector<double> gol;
    std::vector<double> obs;
    std::vector<double> strt;

    readFile("txt/world2agents.txt", bndry, gol, obs, strt, DynModel, dim, NumVs, NumCs, data);
    
    double SolveTime = 5.0;
    double Tollorance = 1.0;  // i am impatient with code

    plan(bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data);


    return 0;
}











