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

#ifndef IPStateChangeLinearModel_hpp
#define IPStateChangeLinearModel_hpp

#include <stdio.h>
#include "IPStateChangeModel.hpp"
#include "RPGBuilder.h"

#define EPS 0.0001
using namespace std;
using namespace Planner;

class IPStateChangeLinearModel : public IPStateChangeModel {
public:
    IPStateChangeLinearModel(MinimalState & state, int horizon, vector<bool> & aL, vector<bool> & fL, vector<bool> & aE, vector<bool> & fE, bool d = false) :  IPStateChangeModel(state,horizon, aL, fL, aE, fE, d), delta(1), infinity(10),  minMaxValues(0), minMaxSteps(0){
        infinity = 100000;//
        //infinity = IloInfinity;
        
    }
    
    ~IPStateChangeLinearModel(){
        
    }
    virtual void addMIPStart(list<int> solution){ };
    virtual void incrementTimeHorizon(MinimalState & state, int increment = 0);

protected:
    double delta;
    double infinity;
    virtual void initialiseVariables();
    virtual void buildConstraints();
    void addVariables(int tMin, int tMax);

    void numericPreconditions(int tMin, int tMax);
    void numericEffects(int tMin, int tMax);
    void numericInitialStateConstraint();
    void numericGoalStateConstraint();
    void numericNoopConstraint(int tMin, int tMax);
    void numericNoopEffects(int tMin, int tMax);
    void numericMutex(int tMin, int tMax);
    void numericMutexEffects(int tMin, int tMax);

    vector<vector<pair<double,double>>>  minMaxValues;
    vector<vector<pair<double,double>>>  minMaxSteps;
};

#endif /* IPStateChangeLinearModel_hpp */


