///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Theodoros Tavoulareas, Efe Eboreime
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl
{
    namespace control
    {
        // Implementation of RG-RRT
        // Mainly based on the ompl (control) implementation of RRT

        class RGRRT : public base::Planner
        {
            public:
                /** \brief Constructor */
                RGRRT(const SpaceInformationPtr &si);

                ~RGRRT() override;

                /** \brief Continue solving for some amount of time. Return true if solution was found. */
                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

                /** \brief Clear datastructures. Call this function if the
                    input data to the planner has changed and you do not
                    want to continue planning */
                void clear() override;

                /** In the process of randomly selecting states in the state
                    space to attempt to go towards, the algorithm may in fact
                    choose the actual goal state, if it knows it, with some
                    probability. This probability is a real number between 0.0
                    and 1.0; its value should usually be around 0.05 and
                    should not be too large. It is probably a good idea to use
                    the default value. */
                void setGoalBias(double goalBias)
                {
                    goalBias_ = goalBias;
                }

                /** \brief Get the goal bias the planner is using */
                double getGoalBias() const
                {
                    return goalBias_;
                }

                /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
                */
                bool getIntermediateStates() const
                {
                    return addIntermediateStates_;
                }

                /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
                *   itself */
                void setIntermediateStates(bool addIntermediateStates)
                {
                    addIntermediateStates_ = addIntermediateStates;
                }

                void getPlannerData(base::PlannerData &data) const override;

                /** \brief Set a different nearest neighbors datastructure */
                template <template <typename T> class NN>
                void setNearestNeighbors()
                {
                    if (nn_ && nn_->size() != 0)
                        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                    clear();
                    nn_ = std::make_shared<NN<Motion *>>();
                    setup();
                }

                void setup() override;

            protected:
                /** \brief Representation of a motion

                    This only contains pointers to parent motions as we
                    only need to go backwards in the tree. */
                class Motion
                {
                public:
                    Motion() = default;

                    /** \brief Constructor that allocates memory for the state and the control */
                    /** also takes care about the reachable states */
                    Motion(const SpaceInformation *si)
                        : state(si->allocState()), control(si->allocControl())
                    {
                    }

                    ~Motion() = default;

                    /** \brief The state contained by the motion */
                    base::State *state{nullptr};

                    /** \brief The control contained by the motion */
                    Control *control{nullptr};

                    /** \brief The number of steps the control is applied for */
                    unsigned int steps{0};

                    /** \brief The parent motion in the exploration tree */
                    Motion *parent{nullptr};

                    /** \brief The vector of reachable states from the motion**/
                    std::vector<Motion *> reachable_states;

                    /** \brief Pick 11 reachable states */
                    double num_reachable_states{11};

                    void setReachableStates(const SpaceInformation *si) {

                        /** take only first control into account: torque (pendulum), u0 (car).
                            Choose a small number of controls and compute the states that were reached after
                            applying them for a short duration. */
                        double highBound = si->getControlSpace()->as<RealVectorControlSpace>()->getBounds().high[0];
                        double lowBound = si->getControlSpace()->as<RealVectorControlSpace>()->getBounds().low[0];
                        auto controlAllocated = (highBound - lowBound) / num_reachable_states;

                        /** pick 11 evenly spaced values for τ between the torque limits (similarly for u0) */
                        for (int i=0; i< num_reachable_states; i++) {

                            // Control to be allocated
                            double control_amount = lowBound + i * controlAllocated;

                            // Allocate that control to a motion and check if reachable
                            Motion* reachable_motion = new Motion(si);
                            reachable_motion->control = si->allocControl();

                            double*& controls_reachable = reachable_motion->control->as<RealVectorControlSpace::ControlType>()->values;
                            controls_reachable[0] = control_amount; // first control only

                            //apply the control to this motion for one timestep and add it as reachable if valid
                            si->propagate(state, reachable_motion->control, 1, reachable_motion->state);
                            if(si->isValid(reachable_motion->state))
                            {
                                reachable_motion->steps = 1;
                                reachable_states.push_back(reachable_motion);
                            } else 
                            {
                                delete reachable_motion;
                            }
                        }
                    }
                };

                /** \brief Free the memory allocated by this planner */
                void freeMemory();

                /** \brief Compute distance between motions (actually distance between contained states) */
                double distanceFunction(const Motion *a, const Motion *b) const
                {
                    return si_->distance(a->state, b->state);
                }

                /** \brief State sampler */
                base::StateSamplerPtr sampler_;

                /** \brief Control sampler */
                DirectedControlSamplerPtr controlSampler_;

                /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
                const SpaceInformation *siC_;

                /** \brief A nearest-neighbors datastructure containing the tree of motions */
                std::shared_ptr<NearestNeighborsLinear<Motion *>> nn_;

                /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
                 * available) */
                double goalBias_{0.05};

                /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
                bool addIntermediateStates_{false};

                /** \brief The random number generator */
                RNG rng_;

                /** \brief The most recent goal motion.  Used for PlannerData computation */
                Motion *lastGoalMotion_{nullptr};

        };

    }  // namespace control 
}  // namespace ompl

#endif