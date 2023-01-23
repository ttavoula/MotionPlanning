///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Theodoros Tavoulareas, Efe Eboreime
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <cmath>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include "ompl/base/spaces/SO2StateSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // The dimension of your projection for the pendulum 
        return 1;
    }

    void project(const ompl::base::State *state , Eigen::Ref<Eigen::VectorXd>  projection ) const override
    {
        // Projection for the pendulum
        projection(0) = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value;
 
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q , const ompl::control::Control *control ,
                 ompl::control::ODESolver::StateType &qdot)
{
    
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];  
 
    // Retrieve the current configuration of the Pendulum.   
    const double theta = q[0];
    const double rotational_velocity =q[1];

    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
    qdot[0] = rotational_velocity;            // theta-dot
    qdot[1] = -9.81*cos(theta) + torque;       // omega-dot
    
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state)
{
     
    // cast the abstract state type to the type we expect
    const auto se2state = state->as<ompl::base::SE2StateSpace::StateType>();
  
    // extract the first component of the state and cast it to what we expect
    const auto *theta = se2state->as<ompl::base::SO2StateSpace::StateType>(0);
         
    // extract the second component of the state and cast it to what we expect
    const auto velocity = se2state->as<ompl::base::RealVectorStateSpace::StateType>(1);
 
    // check validity of state defined by theta & velocity  
    return si->satisfiesBounds(state) && velocity->values[0]>=-10 && velocity->values[0]<=10 &&theta->value>=-M_PI && theta->value<=M_PI;
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // Create and setup the pendulum's state space, control space, validity checker, everything we need for
    // planning.
    // construct the R1 part of the state space we are planning in
    auto r1=std::make_shared<ompl::base::RealVectorStateSpace>(1);
 
    // set the bounds for R1
    ompl::base::RealVectorBounds bounds(1);
    bounds.setLow(-10);
    bounds.setHigh(10);
    r1->setBounds(bounds);
      
    // construct the S02 part of the state space we are planning in
    auto so2=std::make_shared<ompl::base::SO2StateSpace>();
    
    ompl::base::StateSpacePtr space;
    space=so2+r1;

    // create a control space
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 1));
    
    // set the bounds for the control space
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);

    cspace->setBounds(cbounds);

    // define a simple setup class
   
    auto ss= std::make_shared<ompl::control::SimpleSetup>(cspace);
    // set state validity checking for this space
    ompl::control::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si](const ompl::base::State *state) { return isStateValid(si, state); });

  
    // Use the ODESolver to propagate the system.
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    /// create a start state
    ompl::base::ScopedState<> start(space);
    start[0] = -M_PI/2;
    start[1] = 0.0;
   

    /// create a  goal state; use the hard way to set the elements
    ompl::base::ScopedState<> goal(space);
    goal[0] = M_PI/2;
    goal[1] = 0.0;

    /// set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);

    /// we want to have a reasonable value for the propagation step size
    ss->setup();

    
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // Do some motion planning for the pendulum
    // choice is what planner to use.
    switch (choice)
    {
        case 1:
        {
            ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));       
        }
        break;
        case 2:
        {
            auto si = ss->getSpaceInformation();           
            ompl::base::StateSpacePtr space (si->getStateSpace());
            auto planner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));

            space->registerProjection("myProjection", ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space->as<ompl::base::StateSpace>())));
            planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("myProjection");
            ss->setPlanner(planner);           
        }
        break;
        case 3:
        {
            ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));          
        }
        break;
        default:
            std::cerr << "Select a Valid Planner!" << std::endl;
        break;
    }


    /// attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = ss->solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen
        std::ofstream myfile;
        myfile.open("path_pendulum.txt");
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
        ss->getSolutionPath().asGeometric().printAsMatrix(myfile);
        
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // Do some benchmarking for the pendulum
    double runtime_limit=60.0;
    double memory_limit=10000.0;
    int run_count=50;

    // create the benchmark object and add all the planners we'd like to run
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(*ss, "Pendulum Benchmark");

    // Planners for benchmarking

    // RRT
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    
    // KPIECE      
    auto si = ss->getSpaceInformation();
    ompl::base::StateSpacePtr space (si->getStateSpace());
    auto planner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    space->registerProjection("myProjection", ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space->as<ompl::base::StateSpace>())));
    planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("myProjection");
            
    b.addPlanner(planner);
    ss->setPlanner(planner);
    // RG-RRT
    b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}