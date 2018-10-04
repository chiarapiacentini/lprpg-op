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


#ifndef IPStateChangeModelModel_hpp
#define IPStateChangeModelModel_hpp

#include <stdio.h>
#include "IPModel.hpp"
#include <iostream>
#include "RPGBuilder.h"

using namespace std;
using namespace Planner;

class IPStateChangeModel : public IPModel {
public:
    IPStateChangeModel(MinimalState & state, int horizon, vector<bool> & aL, vector<bool> & fL, vector<bool> & aE, vector<bool> & fE, bool d = false);
    ~IPStateChangeModel(){

    }
    virtual void addMIPStart(list<int> solution);
    virtual void incrementTimeHorizon(MinimalState & state, int increment = 0);

protected:
    virtual void initialiseVariables();
    virtual void buildConstraints();
    virtual void updateConstraints(MinimalState &state);
    virtual void extractSolution();
    virtual int extractSolutionLength();

    void addVariables(int tMin, int tMax);

    map <pair<Literal*,int>, int> indexLiteral;
    map <pair<instantiatedOp*,int>, int> indexAction;
    map <pair<Literal*,int>, int> indexActionNoop;
    map <pair<PNE*,int>, int> indexPNE;
    map <pair<PNE*,int>, int> indexPNENoop;
    // min and max values of variables
    vector<double> maxPossible;
    vector<double> minPossible;
    map <pair<int,int>,int> uc;
    
    map <pair<Literal*,int>, int> indexLiteralPreAdd;
    map <pair<Literal*,int>, int> indexLiteralPreDel;
    map <pair<Literal*,int>, int> indexLiteralAdd;
    map <pair<Literal*,int>, int> indexLiteralMaintain;
    
    //constraints
    void initialStateConstraint();
    void goalStateConstraint();
    void stateChangeConstraint(int tMin, int tMax);
    void conflictingActionsConstraint(int tMin, int tMax);
    void mutexActions(int tMin, int tMax);
    void flowConstraint(int tMin, int tMax);
    void objectiveFunction(int tMin, int tMax);
    void lastLayer(int tMax);

    void numericActionPreconditions(int tMin, int tMax);
    void numericPreconditions(int tMin, int tMax);
    void numericGoalConditions();
    void numericMutex(int tMin, int tMax);
    void numericMutexPrecondition(int tMin, int tMax);
    void addLandmarksConstraints(int tMin, int tMax);
    void addNumericLandmarksConstraints(int tMin, int tMax);
    void addRelevanceConstraints(int tMin, int tMax);
    void addNumericRelevanceConstraints(int tMin, int tMax);
    void variablesBounds(int tMin, int tMax);
    void sequentialActions(int tMin, int tMax);
    void firstActions(int tMin, int tMax);
    
    int mT;
    int maxUse;
    int B;
    MinimalState &theState;
    bool literalInPreNotDel(Literal *l, int action);
    bool literalInAddNotPre(Literal *l, int action);
    bool literalInPreAndDel(Literal *l, int action);
    bool literalInAddAndDel(Literal *l, int action);
    
    vector<bool> actionLandmarks;
    vector<bool> factLandmarks;
    vector<bool> eliminatedActions;
    vector<bool> eliminatedFacts;
    list<int> toErase;
    vector<vector<pair<double,double>>>  minMaxValues;
    vector<vector<pair<double,double>>>  minMaxSteps;
};
#endif /* IPStateChangeModel_hpp */
