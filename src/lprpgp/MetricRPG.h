/************************************************************************
 * Copyright(C) 2018: C. Piacentini, M. P. Castro, A. A. Cire, J. C. Beck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public license as
 * published by the Free Software Foundation; either of the license, or
 * (at your option) any later version.
 *
 * This planner is currently in BETA, and is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU Lesser General Public license for more details.
 *
 * To contact the development team, email to
 * <chiarap@mie.utoronto.ca>
 *
 * This builds on LPRPG. The following is the original LPRPG license:
 *
 * Copyright 2008, 2009, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Maria Fox, Richard Howey and Derek Long - Code from VAL
 * Stephen Cresswell - PDDL Parser
 * Andrew Coles, Amanda Coles - Code for LPRPG
 *
 * This file is part of LPRPG.
 *
 * LPRPG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * LPRPG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LPRPG.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#ifndef METRICRPG_H
#define METRICRPG_H

#include "RPGBuilder.h"

#include <vector>
#include <map>
#include <list>
#include <limits>

using std::vector;
using std::map;
using std::list;

namespace Planner {

/** @brief Class to store the numeric data from an RPG, following the approach of Metric FF. */
class MetricRPG
{
    protected:
        
        /** @brief List the fluent values and actions available in a given layer. */
        struct LayerData {
            
            /** @brief  Values of the fluent in the layer. */
            vector<double> fluentValues;
            
            /** @brief  Action in the preceding layer with the largest assignment on each fluent. */
            vector<pair<int,double> > biggestAssignmentAction;
            
            /** @brief  For each variable, whether assignment gave the best attained value in this layer. */
            vector<bool> assignmentWasBetter;
            
            /** @brief  Actions in the preceding layer with increase effects on each fluent. */
            vector<list<int> > increasersAvailable;
            
            /** @brief  Constructor for fact layer zero. */
            LayerData(const vector<double> & d, const int & doublePNECount)
                : fluentValues(d),
                  biggestAssignmentAction(doublePNECount, make_pair(-1,std::numeric_limits<double>::signaling_NaN())),
                  assignmentWasBetter(doublePNECount, false),
                  increasersAvailable(doublePNECount)
            {
            }

            /** @brief  Copy constructor, capable of also making subsequent layers.
             *
             *  @param makeSuccessor  If <code>true</code>, the constructor makes a new fluent layer, after that given,
             *                        emptying the relevant member variables.
             */
            
            LayerData(const LayerData & other, const bool makeSuccessor=false)
                : fluentValues(other.fluentValues)
            {
                if (!makeSuccessor) {
                    biggestAssignmentAction = other.biggestAssignmentAction;
                    assignmentWasBetter = other.assignmentWasBetter;
                    increasersAvailable = other.increasersAvailable;
                    return;
                }
                
                const int doublePNECount = other.assignmentWasBetter.size();
                biggestAssignmentAction.resize(doublePNECount, make_pair(-1,std::numeric_limits<double>::signaling_NaN()));
                assignmentWasBetter.resize(doublePNECount,false);
                increasersAvailable.resize(doublePNECount);
                
                for (int v = 0; v < doublePNECount; ++v) {
                    if (other.assignmentWasBetter[v]) {
                        increasersAvailable[v] = other.increasersAvailable[v];
                    }
                }
                
            }
            
            bool getActionsToReach(const int & var, double & currentLevel,
                                   const double & targetLevel, const double & weight,
                                   list<pair<int, double> > & retList);
                                   
        };
        
        friend struct LayerData;
        
        /** @brief How many layers there are in the RPG. */
        int layerCount;
        
        /** @brief Mapping from timestamps to the layers. */
        map<double, int, EpsilonComp> timestampToIndex;
        
        /** @brief The fluent layers. */
        vector<LayerData*> fluentLayers;
        
        /** @brief  The state being evaluated. */
        MinimalState * startingState;
        
        /** @brief  The cost of each action. */
        const vector<double> * const propositionalActionCosts;
        
        /** @brief  Whether each action has appeared yet, or not, in the RPG.
         *
         * If an entry is <code>true</code>, the respective action has appeared.
         */
        vector<bool> actionHasAppeared;
        
        set<int> potentiallyHelpful;
        
        /** @brief Whether the action--effect lookup tables have been defined. */
        static bool doneInitialisation;
        
        /** @brief The increase effects of each action, on the variables defined as keys of the map. */
        static vector<map<int,double> > increaseEffectsOfEachAction;
        
        /** @brief The assignment effects of each action, on the variables defined as keys of the map. */
        static vector<map<int,double> > assignmentEffectsOfEachAction;
        
        /** @brief Max amount needed of each variable. */
        static vector<double> maxNeeded;
        /** @brief Min amount needed of each variable. */
        static vector<double> minNeeded;
        
    public:
        
        /** @brief Create a new metric RPG, for the state given.
         *
         *  @param forKShot       The state being evaluated
         *  @param layerZero      The values of the fluents in layer zero of the RPG (i.e. the state)
         *  @param actionCosts    A pointer to a vector of action costs, later used to tie-break achievers
         */
        MetricRPG(MinimalState * forKShot, const vector<double> & layerZero, const vector<double> * const actionCosts);
        
        /** @brief  Destructor - deletes all the fluent layer pointers. */
        ~MetricRPG();
        
        /** @brief Create a new action layer.
         *
         *  @param propPrec  The number of remaining unsatisfied propositional preconditions on each action
         *  @param numPrec   The number of remaining unsatisfied numeric preconditions on each action
         *  @param newTS     The timestamp of the new layer
         */
        void addApplicableActions(vector<int> * const propPrec, vector<int> * const numPrec, const double & newTS);
        
        /** @brief Find the values of the fluents after applying all the available actions so far. */
        void fillMinMaxFluentTable(vector<double> & toFill);
        
        /** @brief Get actions to achieve a given numeric variable value.
         *
         * @param layer  The layer at which the precondition should be achieved
         * @param var    The variable on which the precondition must hold
         * @param min    If <code>true</code>, the precondition is of the form <code>v <= RHS</code>
         * @param RHS    The right-hand-side of the precondition
         * @param retList  After calling the function, this contains the actions to use.  Each entry is a pair of:
         *                 - an action index
         *                 - how many times to apply it
         */
        void getActionsFor(const double & layer, const int & var, const bool & min, const double & RHS,
                           list<pair<int, double> > & retList);
        
        bool isHelpful(const int & a) const {
            return potentiallyHelpful.find(a) != potentiallyHelpful.end();            
        };
};


};

#endif // METRICRPG_H
