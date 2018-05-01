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

#include <iostream>
#include <sys/times.h>
#include <unistd.h>

#include "IPCompilation.hpp"
#include "GlobalSchedule.h"
#include "VariableElimination.hpp"
#include "RPGBuilder.h"
#include "MILPRPG.h"
#include "IPSolver.hpp"

#ifdef FOUNDCPLEX
#include "CPLEXSolver.hpp"
#endif

#ifdef FOUNDGUROBI
#include "GurobiSolver.hpp"
#endif

#include "IPModel.hpp"
#include "IPStateBasedModel.hpp"
#include "IPStateChangeModel.hpp"
#include "IPStateBasedLinearModel.hpp"
#include "IPStateBasedUnified.hpp"
#include "IPStateChangeLinearModel.hpp"
#include "IPStateChangeUnified.hpp"
#include "IPHeuristic.hpp"

using namespace std;

void Compilation::solveModel(int h, ModelType modelType, NumericType numericType){
    cout << "inside compilation" << endl;
    
    const bool debug = GlobalSchedule::globalVerbosity & 64;
    LiteralSet initialState;
    vector<double> initialFluents;
    RPGBuilder::getInitialState(initialState, initialFluents);

    set<int> isLocal;
    LiteralSet::iterator isItr = initialState.begin();
    const LiteralSet::iterator isEnd = initialState.end();
    
    for (; isItr != isEnd; ++isItr) {
        isLocal.insert((*isItr)->getStateID());
    }
 //const map<int, int> & sa, const map<int, int> & ia, const map<int, int> & lc, const vector<AutomatonPosition> & ps, const vector<bool> & ls, const double & ppv, const double & sc)
    
    MinimalState state(isLocal,initialFluents,map<int, int>(),map<int, int>(),map<int, int>(),vector<AutomatonPosition>(),vector<bool>(),0,0);
    // perform landmark and relevance analysis
    vector<bool> actionLandmarks;
    vector<bool> factLandmarks;
    vector<bool> eliminatedActions;
    vector<bool> eliminatedFacts;
    if (RPGBuilder::useExtraConstraints || MILPRPG::addLandmarks){
        int nConditions = RPGBuilder::getNLiterals() + RPGBuilder::getNumericPreTable().size();
        actionLandmarks.assign(RPGBuilder::getNOp(),false);
        factLandmarks.assign(nConditions,false);
        eliminatedActions.assign(RPGBuilder::getNOp(),true);
        eliminatedFacts.assign(nConditions,true);
        set<int> actions;
        vector<vector<bool>> factLandmarksTable;
        for (int i = 0; i<RPGBuilder::getNOp(); ++i) actions.insert(i);
        if (MILPRPG::addLandmarks){
            vector<vector<bool>> factLandmarksTable;
            if (MILPRPG::addLandmarks){
                FukunagaAnalysis::performLandmarkAnalysis(actions,state,true, factLandmarksTable, debug);
                FukunagaAnalysis::extractLandmarkActionsAndFacts(state,factLandmarksTable, factLandmarks,actionLandmarks,true, debug);
            }
            if (RPGBuilder::useExtraConstraints){
                vector<vector<bool>> factLandmarksForActions(RPGBuilder::getNOp(), vector<bool>(nConditions,false));
                vector<bool> dominatedActions(RPGBuilder::getNOp(),false);
                vector<bool> dominatedFacts(nConditions,false);
                vector<vector<bool>> fAdd(RPGBuilder::getNOp(),vector<bool>(nConditions,false));
                for (int i = 0; i < RPGBuilder::getNOp(); ++i){
                    list<Literal *> addList = RPGBuilder::getStartAddEffects()[i];
                    for (auto l : addList){
                        int iL = l->getStateID();
                        fAdd[i][iL] = true;
                    }
                    for (int iL = RPGBuilder::getNLiterals(); iL < nConditions; ++iL){
                        fAdd[i][iL] = true;
                    }
                    
                }
                FukunagaAnalysis::getRelevantActions(eliminatedActions,dominatedActions, fAdd, debug);
                if (debug) cout << "calculate getRelevantPropositions" << endl;
                FukunagaAnalysis::getRelevantPropositions(eliminatedFacts, eliminatedActions, state, debug);
            }
            
        }
    }

    if (h!=-1){
        
        IPSolver *solver;
        
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
        IPModel *model;
        
        if (modelType==StateBased && numericType == Simple){
            model = new IPStateBasedModel(state, h, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
        }
        else if (modelType==StateBased && numericType == Linear)
            model = new IPStateBasedUnified(state, h, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
        else if (modelType==StateChange && numericType == Simple)
            model = new IPStateChangeModel(state, h, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
        else
            model = new IPStateChangeUnified(state, h, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
        
        model->setSolver(solver);
        solver->initialise();
        model->start(state);
        model->solve();
        model->printSolution();
        return;
    }else
        iterativeHorizon(state, modelType, numericType, debug);
}

void Compilation::iterativeHorizon(MinimalState &state,  ModelType modelType,  NumericType numericType, bool debug){
    // calculate minCost
    double minActionCost = DBL_MAX;
    for(int i = 0; i<RPGBuilder::getNOp(); ++i){
        if (minActionCost > RPGBuilder::getActionCost()[i]){
            minActionCost = RPGBuilder::getActionCost()[i];
        }
    }
    if (minActionCost <= 0.0001){
        RPGBuilder::usePlanLength = true;
        minActionCost = 1;
        cout << "\n\n!!!!!!!!!\n\nOptimise plan length instead of metric\n\n!!!!!!!!!\n\n";
    }
    
    cout << "Formulation " << modelType << endl;
    cout << "Numeric " << numericType << endl;
    double totalTime = 1800;
    tms refReturn;
    times(&refReturn);
    double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
    int maxToVerify = 0;
    // solve relaxation first
    cout << "relaxation" << endl;
    bool landmarks = MILPRPG::addLandmarks;
    MILPRPG::addLandmarks = true;
    bool extracConstraints = RPGBuilder::useExtraConstraints;
    extracConstraints = false;
    bool seqConstraints = RPGBuilder::useSEQConstraints;
    RPGBuilder::useSEQConstraints = true;
    double minC = 1;
    double minH = 1;
    double pL = 1;

    IPHeuristic *relaxed = new IPHeuristic(state, debug);
    IPSolver *solverRelaxed;
#if defined(FOUNDCPLEX) && defined(FOUNDGUROBI)
    if (Planner::RPGBuilder::ipSolver == Planner::GUROBI)
        solverRelaxed = new GurobiSolver(debug);
    else if (Planner::RPGBuilder::ipSolver == Planner::CPLEX)
        solverRelaxed = new CPLEXSolver(debug);
#elif defined(FOUNDCPLEX)
    solverRelaxed = new CPLEXSolver(debug);
#elif defined(FOUNDGUROBI)
    solverRelaxed = new GurobiSolver(debug);
#endif
    relaxed->setSolver(solverRelaxed);
    relaxed->start(state);
    relaxed->solve();
    minC = relaxed->getObjectiveValue();
    pL = relaxed->getPlanLength();
    if (RPGBuilder::planAnyway && minC < 1) {
        minC = 1;
        pL = 1;
    }
    //double minH = numericType == RPGBuilder::useStartPlanLength ? relaxed->getPlanLength() + 1 : minC + 1;
    minH = RPGBuilder::useStartPlanLength ? pL + 1 : minC + 1;
    
    

    cout << "relaxed solution found with objective value " << minC << endl;
    cout << "relaxed solution found with plan length     " << pL << endl;
    bool solved = false;
    bool solveOpt = false;
    int nIteration = 0;

    RPGBuilder::useSEQConstraints = seqConstraints;
    RPGBuilder::useExtraConstraints = extracConstraints;
    MILPRPG::addLandmarks = landmarks;
    
    // perform landmark and relevance analysis
    vector<bool> actionLandmarks;
    vector<bool> factLandmarks;
    vector<bool> eliminatedActions;
    vector<bool> eliminatedFacts;

    if (RPGBuilder::useExtraConstraints || MILPRPG::addLandmarks){
        int nConditions = RPGBuilder::getNLiterals() + RPGBuilder::getNumericPreTable().size();
        actionLandmarks.assign(RPGBuilder::getNOp(),false);
        factLandmarks.assign(nConditions,false);
        eliminatedActions.assign(RPGBuilder::getNOp(),true);
        eliminatedFacts.assign(nConditions,true);
        set<int> actions;
        for (int i = 0; i<RPGBuilder::getNOp(); ++i) actions.insert(i);
        vector<vector<bool>> factLandmarksTable;
        if (MILPRPG::addLandmarks){
            FukunagaAnalysis::performLandmarkAnalysis(actions,state,true, factLandmarksTable, debug);
        }else{
            int nConditions = RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size();
            factLandmarksTable.assign(nConditions,vector<bool>(nConditions,false));
            for (int i = 0; i < nConditions; ++i){
                factLandmarksTable[i][i] = true;
            }
        }
        FukunagaAnalysis::extractLandmarkActionsAndFacts(state,factLandmarksTable, factLandmarks,actionLandmarks,true, debug);
        if (RPGBuilder::useExtraConstraints){
            vector<vector<bool>> factLandmarksForActions(RPGBuilder::getNOp(), vector<bool>(nConditions,false));
            vector<bool> dominatedActions(RPGBuilder::getNOp(),false);
            vector<bool> dominatedFacts(nConditions,false);
            vector<vector<bool>> fAdd(RPGBuilder::getNOp(),vector<bool>(nConditions,false));
            for (int i = 0; i < RPGBuilder::getNOp(); ++i){
                list<Literal *> addList = RPGBuilder::getStartAddEffects()[i];
                for (auto l : addList){
                    int iL = l->getStateID();
                    fAdd[i][iL] = true;
                }
                for (int iL = RPGBuilder::getNLiterals(); iL < nConditions; ++iL){
                    fAdd[i][iL] = true;
                }
                
            }
            FukunagaAnalysis::getRelevantActions(eliminatedActions,dominatedActions, fAdd, debug);
            if (debug) cout << "calculate getRelevantPropositions" << endl;
            FukunagaAnalysis::getRelevantPropositions(eliminatedFacts, eliminatedActions, state, debug);
            

        }
    }
    list<int> solution;
    IPSolver *solver;
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
    
    IPModel  *sbf;
    if (modelType==StateBased && numericType == Simple)
        sbf = new IPStateBasedModel(state, minH, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
    else if (modelType==StateBased && numericType == Linear)
        sbf = new IPStateBasedUnified(state, minH, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
    else if (modelType==StateChange && numericType == Simple)
        sbf = new IPStateChangeModel(state, minH, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
    else
        sbf = new IPStateChangeUnified(state, minH, actionLandmarks, factLandmarks, eliminatedActions, eliminatedFacts, debug);
    
    bool first = true;
    while (!solved){
        nIteration++;

        tms timeNow;
        times(&timeNow);
        double secsNow = ((double)timeNow.tms_utime + (double)timeNow.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
        cout << "trying solution with horizon " << minH << " and budget time " << totalTime - secsNow << endl;
        if (first){
            sbf->setSolver(solver);
            sbf->start(state);
            first = false;
        }else{
            sbf->incrementTimeHorizon(state,1);
        }
        sbf->setTimeLimit(totalTime - secsNow);
        solved = sbf->solve();
        if(solved){
            maxToVerify = sbf->getObjectiveValue() ;
            tms timeNow;
            times(&timeNow);
            double secsNow = ((double)timeNow.tms_utime + (double)timeNow.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
            if (maxToVerify <= minC || maxToVerify + 1 <= minH) {
                solveOpt = true;
                cout <<"---------------------------" <<endl;
                cout <<";;;; Solution Found " <<endl;
                cout << "; States evaluated: " << nIteration << endl;
                cout << ";" << endl;
                cout << "; Time : " << secsNow << endl;
                cout << "; Time building     : " << secsNow << endl;
                cout << "; Time solving      : " << secsNow << endl;
                cout << ";" << endl;
                sbf->getSolution();
            }else{
                cout << "need to check optimal solution" << endl;
                solution = sbf->extractSolutionList();
            }
        }else{
            minH++;
        }
    }
    if (solved && !solveOpt){
        nIteration++;
        maxToVerify++;
        maxToVerify = maxToVerify / minActionCost + 1;

        tms timeNow;
        times(&timeNow);
        double secsNow = ((double)timeNow.tms_utime + (double)timeNow.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
        cout << "trying solution with horizon " << maxToVerify << " and budget time " << totalTime - secsNow << endl;
        sbf->checkOptimality();
        sbf->incrementTimeHorizon(state,maxToVerify-minH);
        sbf->setTimeLimit(totalTime - secsNow);
        //sbf->addMIPStart(solution);
        solveOpt = sbf->solve();
        if(solveOpt){
            tms timeNow;
            times(&timeNow);
            double secsNow = ((double)timeNow.tms_utime + (double)timeNow.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
            cout <<"---------------------------" <<endl;
            cout <<";;;; Solution Found " <<endl;
            cout << "; States evaluated: " << nIteration << endl;
            cout << ";" << endl;
            cout << "; Time : " << secsNow << endl;
            cout << "; Time building     : " << secsNow << endl;
            cout << "; Time solving      : " << secsNow << endl;
            cout << ";" << endl;
            sbf->getSolution();
        }
        
        
    }
    return;
}
