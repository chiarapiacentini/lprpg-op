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

#ifndef IPStateChangeLinear_hpp
#define IPStateChangeLinear_hpp

#include <stdio.h>
#include "IPStateChange.hpp"
#include "CPLEXSolverInterface.hpp"
#include "RPGBuilder.h"


using namespace std;
using namespace Planner;

class IPStateChangeLinear : public IPStateChange {
public:
    IPStateChangeLinear(MinimalState & state, int horizon, vector<bool> & aL, vector<bool> & fL, vector<bool> & aE, vector<bool> & fE, bool d = false) : IPStateChange(state,horizon,aL,fL,aE,fE,d), delta(0.00000001){
        //B = 100000000;
    }
    ~IPStateChangeLinear(){
        IloEnv env = model.getEnv();
        env.end();
    }
    virtual void addMIPStart(list<int> solution){ };
protected:
    double delta;
    virtual void initialiseVariables();
    virtual void buildConstraints();
    
//    virtual void printSolution(){
//        for (int i = 0; i < x.x.getSize(); ++i){
//            if (i%maxT!=(maxT-1)){
//                if(cplex.getValue(x.x[i])>0.000000001) cout << setprecision(7) <<  x.x[i] << " " << cplex.getValue(x.x[i]) << endl;
//            }
//        }
//    }
    //constraints
    void numericPreconditions();
    void numericEffects();
    void numericInitialStateConstraint();
    void numericGoalStateConstraint();
    void numericNoopConstraint();
    void numericNoopEffects();
    void numericMutex();
    
    virtual void extractSolution();
    
    
};
#endif /* IPStateChangeLinear_hpp */
