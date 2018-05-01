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


#ifndef GurobiIPStateBased_hpp
#define GurobiIPStateBased_hpp

#include <stdio.h>
#include <iostream>
#include "GurobiSolverInterface.hpp"
#include "RPGBuilder.h"

using namespace std;
using namespace Planner;

class GurobiIPStateBased : public GurobiSolverInterface {
public:
    GurobiIPStateBased(MinimalState & state, int horizon, vector<bool> & aL, vector<bool> & fL, vector<bool> & aE, vector<bool> & fE, bool d = false);
    ~GurobiIPStateBased(){

    }
protected:
    virtual void initialiseVariables();
    virtual void buildConstraints();
    virtual void updateConstraints(MinimalState &state, int limitH);
    virtual void printSolution();
    
    map <Literal*, int> indexLiteral;
    map <instantiatedOp*, int> indexAction;
    map <Literal*, int> indexActionNoop;
    map <PNE*, int> indexPNE;
    map <PNE*, int> indexPNENoop;
    // min and max values of variables
    vector<double> maxPossible;
    vector<double> minPossible;
    map <int,int> uc;
    
    //constraints
    void initialStateConstraint();
    void goalStateConstraint();
    void objectiveFunction();
    void actionPreconditions();
    void actionsEffects();
    void mutexRelations();
    void noopPreconditions();
    void noopMutex();
    void numericActionPreconditions();
    void numericPreconditions();
    void numericGoalConditions();
    void numericMutex();
    void numericMutexPrecondition();
    void addLandmarksConstraints();
    void addRelevanceConstraints();
    void variablesBounds();
    void sequentialActions();
    void firstActions();
private:
    int mT;
    int maxUse;
    int B;
    MinimalState &theState;
    vector<bool> actionLandmarks;
    vector<bool> factLandmarks;
    vector<bool> eliminatedActions;
    vector<bool> eliminatedFacts;
    virtual void extractSolution();
};
#endif /* GurobiIPStateBased_hpp */
