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

#ifndef LITERALANALYSIS_H
#define LITERALANALYSIS_H

#include <vector>
#include <utility>
#include <list>

using std::vector;
using std::pair;
using std::list;

namespace Inst {
    class Literal;
}

namespace Planner {
    
class LiteralAnalysis {

    /** For each literal, a record of whether or not it is static.
     *
     * @see LiteralAnalysis::isStatic()
     */
	static vector<pair<bool,bool> > staticLiterals;

    /** Remove the static preconditions from the list given.
     *
     *  This is an internal function to remove static facts from actions' precondition
     *  lists.
     *
     *  @param  toPrune  The list from which static facts are removed
     *  @param  toDec    The number of preconditions the action has, decremented
     *                   by one for each fact removed from the list
     *
     *  @retval <code>true</code>  All static facts in the list were true
     *  @retval <code>false</code>  At least one static fact in the list was false, and
     *                              hence the action can never be applied.
     */
	static bool pruneStaticPreconditions(list<Inst::Literal*> & toPrune, int & toDec);
public:
    
    /** Import basic static fact information from TIM.
     *
     *  This function calls the TIM API to obtain basic information about which
     *  facts are static, based on their predicate names and parameter types.
     *  It is used before action instantiation to avoid grounding actions and
     *  preferences that are clearly contradictory.
     */
    static void getBasicStaticInformationFromTIM();

    /** Identify static propositions.
     *
     *  This function identifies static facts, i.e. those that are never
     *  added or deleted by any action.  To obtain the results of the
     *  analysis, use <code>LiteralAnalysis::isStatic()</code>.
     */    
    static void findStaticLiterals();

    /** Remove actions' static preconditions.
     *
     *  This removes any static facts from the actions' precondition lists.  If
     *  an action is found to be inapplicable, due to requiring a fact that is
     *  statically false, it is removed.
     */
	static void pruneStaticPreconditions();
    
    /**
     *  Function to return whether the specified literal is static.
     *  Returns a pair of bools:
     *   - first element = true if the literal is static;
     *   - second element = if first = true, second = true iff the literal holds in the initial state.
     */
	static const pair<bool,bool> & isStatic(const Inst::Literal * const l);

};

};

#endif

