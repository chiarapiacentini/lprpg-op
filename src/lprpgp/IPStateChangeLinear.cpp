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

#include "IPStateChangeLinear.hpp"
#include "MILPRPG.h"


void IPStateChangeLinear::buildConstraints(){
    initialStateConstraint();
    goalStateConstraint();
    stateChangeConstraint();
    conflictingActionsConstraint();
    mutexActions();
    flowConstraint();
    objectiveFunction();
    
    numericPreconditions();
    numericEffects();
    numericInitialStateConstraint();
    numericGoalStateConstraint();
    numericNoopConstraint();
    numericNoopEffects();
    numericMutex();
    
    
    if (RPGBuilder::useExtraConstraints){
        addRelevanceConstraints();
        //variablesBounds();
    }
    if (MILPRPG::addLandmarks){
        addLandmarksConstraints();
    }
    if (checkOptimal){
        sequentialActions();
        firstActions();
    }
}

void IPStateChangeLinear::initialiseVariables(){
    //B = 100000000;
    IloEnv env = model.getEnv();
    try {
        if (debug) cout << "initialising variables state change simple conditions" << endl;
        int iTA = 0;
        // actions
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            for (int t = 0; t < maxT; ++t){
                
                stringstream name;
                instantiatedOp *a = RPGBuilder::getInstantiatedOp(iA);
                name << *a << "_" << t;
                //cout << t << " "  << iA << " " << iTA << " " << getRow(iTA) << " " << getCol(iTA) << " " << getIndex(iA,t) << endl;
                if (t==0) indexAction.insert(make_pair(a, iA));
                if (t < maxT-1) x.x.add(IloBoolVar(env, 0, 1, name.str().c_str()));
                else x.x.add(IloBoolVar(env, 0, 0, name.str().c_str()));
                iTA++;
            }
        }
        int iCount=0;
        // literals pre-add
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                Literal *l = RPGBuilder::getLiteral(iL);
                name << *l << "_preadd_" << t;
                if (t==0) indexLiteralPreAdd.insert(make_pair(l, iCount++));
                x.y.add(IloBoolVar(env, 0, 1, name.str().c_str()));
            }
        }
        
        // literals pre-del
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                Literal *l = RPGBuilder::getLiteral(iL);
                name << *l << "_predel_" << t;
                if (t==0) indexLiteralPreDel.insert(make_pair(l, iCount++));
                x.y.add(IloBoolVar(env, 0, 1, name.str().c_str()));
            }
        }
        
        // literals add
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                Literal *l = RPGBuilder::getLiteral(iL);
                name << *l << "_add_" << t;
                if (t==0) indexLiteralAdd.insert(make_pair(l, iCount++));
                x.y.add(IloBoolVar(env, 0, 1, name.str().c_str()));
            }
        }
        
        // literals maintain
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                Literal *l = RPGBuilder::getLiteral(iL);
                name << *l << "_maintain_" << t;
                if (t==0) indexLiteralMaintain.insert(make_pair(l, iCount++));
                x.y.add(IloBoolVar(env, 0, 1, name.str().c_str()));
            }
        }
        
        
        // numeric variables
        for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                PNE *p = RPGBuilder::getPNE(iV);
                name << *p << "_" << t;
                if (t==0) indexPNE.insert(make_pair(p,iV));
                IloNumVar toAdd(env, name.str().c_str());
                toAdd.setLB(minPossible[iV]);
                toAdd.setUB(maxPossible[iV]);
                x.v.add(toAdd);
                
            }
        }
        // numeric variables noop
        for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                PNE *p = RPGBuilder::getPNE(iV);
                name << *p << "_noopt_" << t;
                if (t==0) indexPNENoop.insert(make_pair(p, RPGBuilder::getNOp() +iV));
                if (t < maxT - 1)  x.x.add(IloBoolVar(env, 0, 1, name.str().c_str()));
                else x.x.add(IloBoolVar(env, 1, 1, name.str().c_str()));
            }
        }

    }catch (IloException& ex) {
        env.out() << "Error: " << ex << std::endl;
    }
}

void IPStateChangeLinear::numericPreconditions(){
    if (debug) cout << "adding numericPreconditions" << endl;
    IloEnv env  = model.getEnv();
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> preconditions = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        for (auto precID : preconditions){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[precID];
            if (debug) cout << "action " << *RPGBuilder::getInstantiatedOp(iA) << " has precondition " << prec << endl;
            int lhs = prec.LHSVariable;
            double rhs = prec.RHSConstant;
            for (int t = 0; t < maxT; ++t){
                if (lhs < RPGBuilder::getPNECount()){
                    // x > 0
                    if (debug && t==0) cout << "first case " << lhs << endl;
                    model.add(B * (x.x[getIndex(iA,t)] - 1)<=  -  delta + x.v[getIndex(lhs, t)] - rhs);
                }else if (lhs < 2*RPGBuilder::getPNECount()){
                    // x < 0
                    if (debug && t==0)cout << "second case " << lhs << endl;
                    model.add(B * (x.x[getIndex(iA,t)] - 1) <=  -  delta - x.v[getIndex(lhs, t)] + rhs);
                }else{
                    // get artificial variable
                    if (debug && t==0)cout << "third case " << lhs << endl;
                    RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(lhs);
                    vector<double> weights = av.weights;
                    vector<int> fluents = av.fluents;
                    double constant = av.constant;
                    IloExprArray toSum(env);
                    for (int i = 0; i<weights.size(); ++i){
                        int index = fluents[i];
                        int w = weights[i];
                        if (fluents[i] >= RPGBuilder::getPNECount()){
                            w = -1*w;
                            index-=RPGBuilder::getPNECount();
                        }else{
                        }
                        toSum.add( w * x.v[getIndex(index,t)]);
                    }
                    // if (debug && t==0) cout << bigM <<" * " << x.x[getIndex(iA,t)] << " -1 <= + " << epsilon << " + " << IloSum(toSum) << " + " << constant << " - " << rhs << endl;
                    // TODO check this
                    model.add(B * (x.x[getIndex(iA,t)] - 1) <=  delta + IloSum(toSum) + (constant - rhs));
                    
                }
            }
        }
    }
}

void IPStateChangeLinear::numericEffects(){
    if (debug) cout << "adding numericEffects" << endl;
    IloEnv env  = model.getEnv();
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        for (int idEff : numEff){
            RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[idEff];
            int indexFluent = eff.fluentIndex;
            bool isAssignment = eff.isAssignment;
            vector<double> weights = eff.weights;
            vector<int> variables = eff.variables;
            double constant = eff.constant;
            for (int t = 1; t < maxT; ++t){
                IloExprArray toSum(env);
                toSum.add(x.v[getIndex(indexFluent,t)]);
                for (int i = 0; i<weights.size(); ++i){
                    toSum.add(-1* weights[i] * x.v[getIndex(variables[i],t-1)]);
                }
                if (!isAssignment)
                    toSum.add(-1 * x.v[getIndex(indexFluent,t-1)]);
                model.add(IloSum(toSum) <= constant + maxPossible[indexFluent] *(1-x.x[getIndex(iA,t-1)]));
                model.add(IloSum(toSum) >= constant * x.x[getIndex(iA,t-1)] + minPossible[indexFluent] *(1-x.x[getIndex(iA,t-1)]));
            }
        }
    }
}

void IPStateChangeLinear::numericInitialStateConstraint(){
    if (debug) cout << "adding numericInitialStateConstraint" << endl;
    // get variables from rpg
    for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
        stringstream name;
        PNE *p = RPGBuilder::getPNE(iV);
        model.add(x.v[getIndex(indexPNE[p],0)] == theState.second[iV]);
    }
}

void IPStateChangeLinear::numericGoalStateConstraint(){
    if (debug) cout << "adding numericGoalStateConstraint" << endl;
    IloEnv env  = model.getEnv();
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto goal : numericGoals){
        if (goal.first!=-1){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[goal.first];
            int lhs = prec.LHSVariable;
            double rhs = prec.RHSConstant;
            if (lhs < RPGBuilder::getPNECount()){
                // x > 0
                if (debug) cout << "first case " << lhs << endl;
                model.add( x.v[getIndex(lhs, maxT-1)] >= rhs);
            }else if (lhs < 2*RPGBuilder::getPNECount()){
                // x < 0
                if (debug) cout << "second case " << lhs << endl;
                model.add( x.v[getIndex(lhs, maxT-1)] <= rhs);
            }else{
                // get artificial variable
                if (debug)cout << "third case " << lhs << endl;
                RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(lhs);
                if (debug) cout << av << " " << rhs << endl;
                vector<double> weights = av.weights;
                vector<int> fluents = av.fluents;
                double constant = av.constant;
                IloExprArray toSum(env);
                for (int i = 0; i<weights.size(); ++i){
                    //cout << i << " " << weights[i] << " " << fluents[i] << endl;
                    int index = fluents[i];
                    int w = weights[i];
                    if (fluents[i] >= RPGBuilder::getPNECount()){
                        w = -1*w;
                        index-=RPGBuilder::getPNECount();
                    }
                    toSum.add( w * x.v[getIndex(index,maxT-1)]);
                }
                if (debug) cout << IloSum(toSum) << " >= " << rhs - constant << endl;
                model.add(IloSum(toSum) >= rhs - constant);
            }
        }
    }
}

void IPStateChangeLinear::numericNoopConstraint(){
    if (debug) cout << "adding numericNoopConstraint" << endl;
    map<int,list<int> > indexFluentActions;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        for (int idEff : numEff){
            int fluentIndex = RPGBuilder::getNumericEff()[idEff].fluentIndex;
            if (indexFluentActions.find(fluentIndex)!= indexFluentActions.end()){
                indexFluentActions[fluentIndex].push_back(iA);
            }else{
                list<int> toPush;
                toPush.push_back(iA);
                indexFluentActions.insert(make_pair(fluentIndex,toPush));
            }
            for (int t = 0; t<maxT; ++t){
                model.add(x.x[getIndex(indexPNENoop[RPGBuilder::getPNE(fluentIndex)],t)] + x.x[getIndex(iA,t)] <= 1);
            }
        }
    }
    // fluent actions
    IloEnv env  = model.getEnv();
    for (int t = 0; t < maxT - 1; ++t){
        for (auto index : indexFluentActions){
            int indexFluent = index.first;
            list<int> actions = index.second;
            IloExprArray toSum(env);
            toSum.add(x.x[getIndex(indexPNENoop[RPGBuilder::getPNE(indexFluent)],t)]);
            for (auto iA : actions){
                toSum.add(x.x[getIndex(iA,t)]);
            }
            model.add(IloSum(toSum) == 1);
        }
    }
}

void IPStateChangeLinear::numericNoopEffects(){
    if (debug) cout << "adding numericNoopEffects" << endl;
    for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
        for (int t = 1; t < maxT; ++t){
            stringstream name;
            PNE *p = RPGBuilder::getPNE(iV);
            model.add(x.v[getIndex(iV, t)] - x.v[getIndex(iV, t-1)] <= maxPossible[iV] *(1-x.x[getIndex(indexPNENoop[p],t-1)]));
            model.add(x.v[getIndex(iV, t)] - x.v[getIndex(iV, t-1)] >= minPossible[iV] *(1-x.x[getIndex(indexPNENoop[p],t-1)]));
        }
    }
}

void IPStateChangeLinear::numericMutex(){
    if (debug) cout << "adding numericMutex" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        for (int jA = 0; jA < RPGBuilder::getNOp(); ++jA){
            // indicate that iA and jA are mutex
            bool isMutex = false;
            if (iA == jA)
                continue;
            // TODO: check
            list<int> numPrec = RPGBuilder::getActionsToRPGNumericStartPreconditions()[jA];
            for (auto nEff : numEff){
                int indexFluent = RPGBuilder::getNumericEff()[nEff].fluentIndex;
                for (auto precID : numPrec){
                    RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[precID];
                    int lhs = prec.LHSVariable;
                    if (lhs < RPGBuilder::getPNECount()){
                        if (lhs == indexFluent){
                            isMutex = true;
                            break; // from numeric precondition
                        }
                    }else if (lhs < 2*RPGBuilder::getPNECount()){
                        if (lhs - RPGBuilder::getPNECount() == indexFluent){
                            isMutex = true;
                            break; // from numeric precondition
                        }
                    }else{
                        RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(lhs);
                        vector<int> fluents = av.fluents;
                        if(find(fluents.begin(), fluents.end(), indexFluent) != fluents.end()){
                            isMutex = true;
                            break;
                        }
                    }
                } // end numeric precondition
                if (isMutex) break; // no need to check other effects
            } // end numeric effect
            if (isMutex){
                if (debug) cout << "actions " << *RPGBuilder::getInstantiatedOp(iA) << " and " << *RPGBuilder::getInstantiatedOp(jA) << " are mutex"<<  endl;
                for (int t = 0; t < maxT; ++t){
                    model.add(x.x[getIndex(iA, t)] +  x.x[getIndex(jA, t)] <= 1);
                }
            }
        }
    }
}

void IPStateChangeLinear::extractSolution(){
    
}
