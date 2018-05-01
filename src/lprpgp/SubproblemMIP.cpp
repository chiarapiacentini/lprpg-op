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


#include "RPGBuilder.h"
#include "TIM.h"

#include "GlobalSchedule.h"

#include "ptree.h"
#include <FlexLexer.h>
#include "instantiation.h"
#include "SimpleEval.h"
#include "DebugWriteController.h"
#include "typecheck.h"
#include "TIM.h"

#include "colours.h"

#include "FuncAnalysis.h"
#include "LiteralAnalysis.h"


#include "FFSolver.h"

#include <assert.h>

#include <algorithm>
#include "MILPRPG.h"
#include "MetricRPG.h"
#include "FFSolver.h"

#include "NNF.h"

#include "NumericAnalysis.h"
#include "PreferenceHandler.h"
#include "landmarksanalysis.h"

#include <sstream>
#include <memory>

#include <chrono>

#include "IPSolver.hpp"
#ifdef FOUNDCPLEX
#include "CPLEXSolver.hpp"
#endif

#ifdef FOUNDGUROBI
#include "GurobiSolver.hpp"
#endif
#include "IPHeuristic.hpp"

using namespace TIM;
using namespace Inst;
using namespace VAL;

using std::ostringstream;
using std::cerr;
using std::endl;

namespace Planner {
    
    // @chiarap : this is the function that we need to modify
    double SubproblemMIP::timeBuildConstraints = 0.;
    double SubproblemMIP::timeSolve = 0.;
    
    SubproblemRPG::EvaluationInfo SubproblemMIP::getRelaxedPlan(MinimalState & theState, const double & maxPrefCostIn, const double & stateTS, const int & nextTIL, set<int> & goals, set<int> & goalFluents/*, set<int> & goalFluents*/, list<pair<int, VAL::time_spec> > & helpfulActions, list<pair<double, list<ActionAndHowManyTimes > > > & relaxedPlan, map<double, list<int> > * justApplied, double tilFrom) {
        // to debug
        chrono::time_point<std::chrono::system_clock> p1, p2, p3;
        const bool debug = GlobalSchedule::globalVerbosity & 64;
        ++RPGBuilder::statesEvaluated;
        if (debug) cout << "in mip relaxation" << endl;
        
        ///
        if (RPGBuilder::blindSearch){
            bool isGoal = FF::checkGoalState(theState);
            if (isGoal) return EvaluationInfo(0, 0.0, true);
            return EvaluationInfo(1, 0.0, false);
        }
        ///
        p1 = std::chrono::system_clock::now();
        
        static IPSolver *solver;
        static IPHeuristic mipdf(theState, debug);
        static bool first = true;
        if (first){
            first = false;
#if defined(FOUNDCPLEX) && defined(FOUNDGUROBI)
            if (Planner::RPGBuilder::ipSolver == Planner::GUROBI)
                solver = new GurobiSolver(debug);
            else if (Planner::RPGBuilder::ipSolver == Planner::CPLEX)
                solver = new CPLEXSolver(debug);
#elif defined(FOUNDCPLEX)
            solver = new CPLEXSolver(debug);
#elif defined(FOUNDGUROBI)
            solver = new GurobiSolver(debug);
#endif
            
            mipdf.setSolver(solver);
        }
        mipdf.start(theState);
        p2 = std::chrono::system_clock::now();
        timeBuildConstraints+= (p2-p1).count();
        if(RPGBuilder::useLPRelaxation && RPGBuilder::statesEvaluated==1)
            mipdf.setLPRelaxation(true);
        double h = -1;
        if (mipdf.solve()){
            h = mipdf.getObjectiveValue();
            map<double,list<pair<int,int> > > rp;// = mipdf.extractActionsHowManyTimes();
            for (auto p : rp){
                list<ActionAndHowManyTimes> action;
                for (auto a : p.second){
                    action.push_back(ActionAndHowManyTimes(RPGBuilder::getInstantiatedOp(a.first),  VAL::E_AT_START, a.second));
                }
                relaxedPlan.push_back(make_pair(p.first,action));

            }
            //if (RPGBuilder::useLPRelaxation)
             //   h = ceil(h);
        }
        p3 = std::chrono::system_clock::now();
        timeSolve+= (p3-p2).count();
        bool isGoal = (h == 0.0);
        if (RPGBuilder::planAnyway) {
            isGoal = FF::checkGoalState(theState);
            if (!isGoal && h<1) h = 1;
        }
        return EvaluationInfo(h, 0.0, isGoal);
        
        


        //return EvaluationInfo();	//just in the case that the problem is found infeasible
    }
    
    
    
}
