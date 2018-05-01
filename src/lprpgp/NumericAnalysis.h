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

#ifndef NUMERIC_ANALYSIS_H
#define NUMERIC_ANALYSIS_H

#include "RPGBuilder.h"

namespace Planner {

class NumericAnalysis {

private:
    static vector<bool> integral;
    static vector<pair<double,double> > pneBounds;
    static vector<pair<double,double> > pneMinMaxNeeded;
    static bool doneIntegralAnalysis;
    
    /** @brief The resources consumed/produced when a given proposition is added/deleted. */
    static map<int, map<int, double> > resourcesExchangedForProposition;

    /** @brief The resources consumed/produced when a given resource is increased.
     * 
     * Keys of the map are PNEs, each an index into <code>RPGBuilder::pnes</code>.  Associated with each
     * (if the entry is defined) is how much of other resources is consumed in order to produce
     * a unit of that PNE.
     */
    static map<int, map<int,double> > resourceExchangeRateToIncrease;

    /** @brief The resources consumed/produced when a given resource is decreased.
    * 
    * Keys of the map are PNEs, each an index into <code>RPGBuilder::pnes</code>.  Associated with each
    * (if the entry is defined) is how much of other resources is consumed in order to consume
    * a unit of that PNE.
    */
    static map<int, map<int,double> > resourceExchangeRateToDecrease;
    
    /** @brief Get the increase effects of the given action. */
    static void getIncreaseEffects(const int & act, map<int,double> & toFill, const int excludeEffect=-1);
    
public:
    static void identifyIntegralPNEs();
    
    /** @brief  Identify propositions that can only be exchanged with resources.
     *
     *  This function recognises the idiom of a proposition which is controlled
     *  by a pair of actions:
     *  - one adding the proposition, and decreasing some resource by c
     *  - one deleting the proposition, and increasing some resource by c
     *
     *  Conceptually, such propositions can never be used to increase the amount of
     *  the resource available as, to achieve the proposition, the amount out
     *  resource produced by deleting the proposition was, previously, consumed.
     */
    static void identifySimplePropositionNumberPumps();
    
    /** @brief  Identify reversible lossless swaps between resources. */
    static void identifySimpleReversibleNumberPumps();
    
    static void performBoundsAnalysis();
    static const vector<pair<double,double> > & getBounds() {
        return pneBounds;
    }
    static void performMinMaxNeededAnalysis();
    static const vector<pair<double,double> > & getMinMaxNeeded() {
        return pneMinMaxNeeded;
    }
    /** @brief  Obtain the resource changes needed to get a given proposition.
     * 
     * @see identifySimplePropositionNumberPumps
     */
    static const map<int, map<int, double> > & getResourcesExchangedForProposition() {
        return resourcesExchangedForProposition;
    }
    
    static const map<int, map<int, double> > & getResourcesExchangedForIncrease() {
        return resourceExchangeRateToIncrease;
    }
    
    static const map<int, map<int, double> > & getResourcesExchangedForDecrease() {
        return resourceExchangeRateToDecrease;
    }
            
};

};

#endif
