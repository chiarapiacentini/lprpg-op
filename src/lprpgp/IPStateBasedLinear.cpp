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

#include "IPStateBasedLinear.hpp"
#include "MILPRPG.h"


void IPStateBasedLinear::buildConstraints(){
    initialStateConstraint();
    goalStateConstraint();
    objectiveFunction();
    actionPreconditions();
    actionsEffects();
    mutexRelations();
    noopPreconditions();
    noopMutex();
    forcePropositionsNotSelected();
    
    //noopForcedIfNoActionsAdd();
    //noopMutexAdd();
    
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

void IPStateBasedLinear::initialiseVariables(){
    IloEnv env = model.getEnv();
    try {
        if (debug) cout << "initialising variables 1" << endl;
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
        // literals
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                Literal *l = RPGBuilder::getLiteral(iL);
                name << *l << "_" << t;
                if (t==0) indexLiteral.insert(make_pair(l, iL));
                x.y.add(IloBoolVar(env, 0, 1, name.str().c_str()));
            }
        }
        // no-op literals
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                Literal *l = RPGBuilder::getLiteral(iL);
                name << *l << "_noopt_" << t;
                if (t==0) indexActionNoop.insert(make_pair(l, RPGBuilder::getNOp()+iL));
                if (t < maxT-1) x.x.add(IloBoolVar(env, 0, 1, name.str().c_str()));
                else x.x.add(IloBoolVar(env, 0, 0, name.str().c_str()));
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
                //toAdd.setLB(-infinity);
                //toAdd.setUB(infinity);
                x.v.add(toAdd);
                
            }
        }
        // numeric variables noop
        for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
            for (int t = 0; t < maxT; ++t){
                stringstream name;
                PNE *p = RPGBuilder::getPNE(iV);
                name << *p << "_noopt_" << t;
                if (t==0) indexPNENoop.insert(make_pair(p, RPGBuilder::getNOp() + RPGBuilder::getNLiterals() +iV));
                if (t < maxT - 1)  x.x.add(IloIntVar(env, 0, 1, name.str().c_str()));
                else x.x.add(IloIntVar(env, 1, 1, name.str().c_str()));
            }
        }
        
    }catch (IloException& ex) {
        env.out() << "Error: " << ex << std::endl;
    }
}

void IPStateBasedLinear::numericPreconditions(){
    if (debug) cout << "adding numericPreconditions" << endl;
    IloEnv env  = model.getEnv();
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[iA];
        for (auto precID : preconditions){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[precID];
            if (debug) cout << "action " << *RPGBuilder::getInstantiatedOp(iA) << " has precondition " << prec << endl;
            int lhs = prec.LHSVariable;
            double rhs = prec.RHSConstant;
            for (int t = 0; t < maxT; ++t){
                if (lhs < RPGBuilder::getPNECount()){
                    // x > 0
                    if (debug && t==0) cout << "first case " << lhs << endl;
                    double epsilon = 0;
                    if (prec.op == VAL::E_GREATER){
                        epsilon-=0.000000001;
                    }
                    model.add(infinity * x.x[getIndex(iA,t)] <=  - epsilon + x.v[getIndex(lhs, t)] - rhs + infinity);
                }else if (lhs < 2*RPGBuilder::getPNECount()){
                    // x < 0
                    double epsilon = 0;
                    if (prec.op == VAL::E_GREATER){
                        epsilon-=0.000000001;
                    }
                    if (debug && t==0)cout << "second case " << lhs << endl;
                    model.add(infinity * x.x[getIndex(iA,t)]  <=  - epsilon - x.v[getIndex(lhs, t)] + rhs + infinity);
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
                            //cout << index << " " << *RPGBuilder::getPNE(index) << " " << x.v[getIndex(index,t)] << endl;
                        }
                        toSum.add( w * x.v[getIndex(index,t)]);
                    }
                   // if (debug && t==0) cout << bigM <<" * " << x.x[getIndex(iA,t)] << " -1 <= + " << epsilon << " + " << IloSum(toSum) << " + " << constant << " - " << rhs << endl;
                    // TODO check this
                    double epsilon = 0;
                    if (prec.op == VAL::E_GREATER){
                        epsilon-=0.000000001;
                    }
                    model.add(infinity * x.x[getIndex(iA,t)]  <= epsilon + IloSum(toSum) + (constant - rhs + infinity));
                    
                }
            }
        }
    }
}

void IPStateBasedLinear::numericEffects(){
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
                model.add(IloSum(toSum) + infinity * x.x[getIndex(iA,t-1)] <= constant + infinity);
                model.add(IloSum(toSum) - infinity * x.x[getIndex(iA,t-1)] >= constant - infinity);
            }
        }
    }
}

void IPStateBasedLinear::numericInitialStateConstraint(){
    if (debug) cout << "adding numericInitialStateConstraint" << endl;
    // get variables from rpg
    for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
        stringstream name;
        PNE *p = RPGBuilder::getPNE(iV);
        model.add(x.v[getIndex(indexPNE[p],0)] == theState.second[iV]);
    }
}

void IPStateBasedLinear::numericGoalStateConstraint(){
    
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

void IPStateBasedLinear::numericNoopConstraint(){
    //bool debug = true;
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
    for (int t = 0; t < maxT; ++t){
        for (int i = 0; i < RPGBuilder::getPNECount(); ++i){
            //for (auto index : indexFluentActions){
            int indexFluent = i;
            list<int> actions = indexFluentActions[i];
            IloExprArray toSum(env);
            toSum.add(x.x[getIndex(indexPNENoop[RPGBuilder::getPNE(indexFluent)],t)]);
            for (auto iA : actions){
                toSum.add(x.x[getIndex(iA,t)]);
            }
            model.add(IloSum(toSum) == 1);
        }
    }
}

void IPStateBasedLinear::numericNoopEffects(){
    if (debug) cout << "adding numericNoopEffects" << endl;
    for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
        for (int t = 1; t < maxT; ++t){
            stringstream name;
            PNE *p = RPGBuilder::getPNE(iV);
            model.add(x.v[getIndex(iV, t)] - x.v[getIndex(iV, t-1)] + infinity * x.x[getIndex(indexPNENoop[p],t-1)] <= infinity);
            model.add(x.v[getIndex(iV, t)] - x.v[getIndex(iV, t-1)] - infinity * x.x[getIndex(indexPNENoop[p],t-1)] >= - infinity);
        }
    }
}

void IPStateBasedLinear::numericMutex(){
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

void IPStateBasedLinear::extractSolution(){
    
}
