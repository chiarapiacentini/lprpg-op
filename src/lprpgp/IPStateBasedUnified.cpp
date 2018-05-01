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
#include "IPStateBasedUnified.hpp"
#include "MILPRPG.h"
#include "VariableElimination.hpp"

void IPStateBasedUnified::buildConstraints(){
    FukunagaAnalysis::getTimeStampedMaxValuesGeneral(minMaxValues, minMaxSteps,minEffect, maxT,theState);
    
    initialStateConstraint();
    goalStateConstraint();
    objectiveFunction(0,maxT);
    actionPreconditions(0,maxT);
    actionsEffects(0,maxT);
    mutexRelations(0,maxT);
    noopPreconditions(0,maxT);
    noopMutex(0,maxT);
    
    if (RPGBuilder::useValidityInequality){
        forcePropositionsNotSelected(0,maxT);
        lastLayer(maxT);
    }
    //noopForcedIfNoActionsAdd();
    //noopMutexAdd();
    
    numericPreconditions(0,maxT);
    numericEffects(0,maxT);
    numericInitialStateConstraint();
    numericGoalStateConstraint();
    //numericNoopConstraint(0,maxT);
    //numericNoopEffects(0,maxT);
    numericMutex(0,maxT);
    numericMutexEffects(0,maxT);
    
    if (RPGBuilder::useExtraConstraints){
        addRelevanceConstraints(0,maxT);
        //variablesBounds();
    }
    if (MILPRPG::addLandmarks){
        addLandmarksConstraints(0,maxT);
    }
    if (checkOptimal && RPGBuilder::useValidityInequality){
        sequentialActions(0,maxT);
        firstActions(0,maxT);
    }
}

void IPStateBasedUnified::initialiseVariables(){
    if (debug) cout << "initialising variables 1" << endl;
    iBool = 0;
    iBoolAux = 0;
    iInt = 0;
    iCount = 0;
    addVariables(0, maxT);
}

void IPStateBasedUnified::addVariables(int tMin, int tMax){
    for (int t = tMin; t < tMax; ++t){
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            // actions
            
            stringstream name;
            instantiatedOp *a = RPGBuilder::getInstantiatedOp(iA);
            name << *a << "_" << t;
            //cout << t << " "  << iA << " " << iTA << " " << getRow(iTA) << " " << getCol(iTA) << " " << getIndex(iA,t) << endl;
            indexAction.insert({{a,t},iBool++});
            if (t < maxT-1) solver->addCol(IPBoolean, 0, 1, name.str());
            else solver->addCol(IPBoolean, 0, 1, name.str());
            
        }
        // literals
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            stringstream name;
            Literal *l = RPGBuilder::getLiteral(iL);
            name << *l << "_" << t;
            indexLiteral.insert({{l,t},iBoolAux++});
            solver->addCol(IPBooleanAux, 0, 1, name.str());
            
        }
        // no-op literals
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            stringstream name;
            Literal *l = RPGBuilder::getLiteral(iL);
            name << *l << "_noopt_" << t;
            indexActionNoop.insert({{l,t}, iBool++});
            if (t < maxT-1) solver->addCol(IPBoolean, 0, 1, name.str());
            else solver->addCol(IPBoolean, 0, 1, name.str());
            
        }
        
        
        // numeric variables
        for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
            stringstream name;
            PNE *p = RPGBuilder::getPNE(iV);
            name << *p << "_" << t;
            indexPNE.insert({{p,t},iCount++});
            solver->addCol(IPContinous, -1000000000000, 100000000000, name.str());
            
        }
//        // numeric variables noop
//        for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
//            stringstream name;
//            PNE *p = RPGBuilder::getPNE(iV);
//            name << *p << "_noopt_" << t;
//            indexPNENoop.insert({{p,t}, iBool++});
//            if (t < maxT - 1) solver->addCol(IPBoolean, 0, 1, name.str());
//            else solver->addCol(IPBoolean, 0, 1, name.str());
//
//        }
    }
}


void IPStateBasedUnified::numericPreconditions(int tMin, int tMax){
    if (debug) cout << "adding numericPreconditions" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[iA];
        for (auto precID : preconditions){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[precID];
            if (debug) cout << "action " << *RPGBuilder::getInstantiatedOp(iA) << " has precondition " << prec << endl;
            int lhs = prec.LHSVariable;
            double rhs = prec.RHSConstant;
            for (int t = tMin; t < tMax; ++t){
                if (lhs < RPGBuilder::getPNECount()){
                    // x > 0
                    if (debug && t==0) cout << "first case " << lhs << " " <<prec.op<<" "<< rhs << endl;
                    double epsilon = 0;
                    if (prec.op == VAL::E_GREATER){
                        epsilon+=minEffect[precID];
                    }
                    list<IPSolver::IPConstraint> toAdd;
                    double lowerbound = minMaxValues[t][lhs].first - rhs - epsilon;
                    toAdd.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], lowerbound });
                    toAdd.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(lhs), t}], 1});
                    solver->addRow(toAdd, IPGreater, lowerbound + epsilon + rhs );
                    
                }else if (lhs < 2*RPGBuilder::getPNECount()){
                    // x < 0
                    double epsilon = 0;
                    if (prec.op == VAL::E_GREATER){
                        epsilon+=minEffect[precID];
                    }
                    if (debug && t==0)cout << "second case " << lhs << endl;
                    
                    list<IPSolver::IPConstraint> toAdd;
                    double lowerbound = minMaxValues[t][lhs - RPGBuilder::getPNECount()].second - rhs - epsilon;
                    toAdd.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],  lowerbound});
                    toAdd.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(lhs), t}], 1});
                    solver->addRow(toAdd, IPLess, lowerbound + rhs + epsilon);
                }else{
                    // get artificial variable
                    double lowerbound = -rhs;
                    RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(lhs);
                    vector<double> weights = av.weights;
                    vector<int> fluents = av.fluents;
                    double constant = av.constant;
                    list<IPSolver::IPConstraint> toSum;
                    if (debug && t==0)cout << "third case " << lhs << " " << rhs << " " << av.constant << endl;
                    for (int i = 0; i<weights.size(); ++i){
                        int index = fluents[i];
                        int w = weights[i];
                        if (fluents[i] >= RPGBuilder::getPNECount()){
                            w = -1*w;
                            index-=RPGBuilder::getPNECount();
                            lowerbound += w*minMaxValues[t][index].second;
                        }else{
                            lowerbound += w*minMaxValues[t][index].first;
                        }
                        const double k = w;
                        toSum.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(index), t}], k});
                    }
                    // TODO check this
                    double epsilon = 0;
                    if (prec.op == VAL::E_GREATER){
                        epsilon+=minEffect[precID];
                    }
                    lowerbound += (constant - epsilon);
                    toSum.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], lowerbound});
                    solver->addRow(toSum,IPGreater, lowerbound - constant + epsilon + rhs);
                }
            }
        }
    }
}

void IPStateBasedUnified::numericEffects(int tMin, int tMax){
    if (debug) cout << "adding numericEffects" << endl;
    for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
        list<pair<int,double>> simpleConditions = simpleEffectActionsConstant[iV];
        list<pair<int,int>> linearConditions = linearEffectActionEffectID[iV];
        for (int t = tMin; t < tMax - 1; ++t){
            list<IPSolver::IPConstraint> simpleConditionUB;
            list<IPSolver::IPConstraint> simpleConditionLB;
            simpleConditionUB.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(iV), t + 1}], 1});
            simpleConditionLB.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(iV), t + 1}], 1});
            simpleConditionUB.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(iV), t}], -1});
            simpleConditionLB.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(iV), t}], -1});
            for (auto actionConstant : simpleConditions){
                
                int iA = actionConstant.first;
                double k = actionConstant.second;
                //cout << "action " << *RPGBuilder::getInstantiatedOp(iA) << " has simple effect on " << *RPGBuilder::getPNE(iV) << endl;
                simpleConditionUB.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], -k});
                simpleConditionLB.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], -k});
            }
            for (auto actionLinear : linearConditions){
                int iA = actionLinear.first;
                simpleConditionUB.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],  - minMaxValues[t+1][iV].second +  minMaxValues[t][iV].first});
                simpleConditionLB.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], - minMaxValues[t+1][iV].first + minMaxValues[t][iV].second  });
                
                // add linear effects
                int idEff = actionLinear.second;
                RPGBuilder::RPGNumericEffect & eff = RPGBuilder::getNumericEff()[idEff];
                int indexFluent = eff.fluentIndex;
                bool isAssignment = eff.isAssignment;
                vector<double> weights = eff.weights;
                vector<int> variables = eff.variables;
                double constant = eff.constant;
                list<IPSolver::IPConstraint> toSum;
                list<IPSolver::IPConstraint> toSum2;
                toSum.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(iV), t + 1}], 1});
                toSum2.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(iV), t + 1}], 1});
                double upperBound = minMaxValues[t+1][iV].second - constant;
                double lowerBound = minMaxValues[t+1][iV].first - constant;
                //cout << "action " << *RPGBuilder::getInstantiatedOp(iA) << " has linear effect on " << *RPGBuilder::getPNE(indexFluent) << endl;
                for (int i = 0; i<weights.size(); ++i){
                    if (abs(weights[i]) < 0.0001) continue;
                    int x = variables[i];
                    double w = weights[i];
                    if (x < RPGBuilder::getPNECount()){
                        
                    }else if (x < 2*RPGBuilder::getPNECount()){
                        x = x - RPGBuilder::getPNECount();
                        w = (-1)*w;
                    }
                    //if (w > 0)
                    toSum.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(x), t}], -1* w});
                    toSum2.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(x), t}], -1* w});
                    if(w>0){
                        upperBound -= w * minMaxValues[t][x].first;
                        lowerBound -= w * minMaxValues[t][x].second;
                    }else{
                        upperBound -= w * minMaxValues[t][x].first;
                        lowerBound -= w * minMaxValues[t][x].second;
                    }
                }
                if (!isAssignment){
                    toSum.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(indexFluent), t}], -1});
                    toSum2.push_back({IPContinous, indexPNE[{RPGBuilder::getPNE(indexFluent), t}], -1});
                    upperBound -= minMaxValues[t][iV].first;
                    lowerBound -= minMaxValues[t][iV].second;
                }
                toSum.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], upperBound});
                toSum2.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],lowerBound});
                solver->addRow(toSum,IPLess,  upperBound + constant);
                solver->addRow(toSum2,IPGreater, lowerBound + constant);
            
            }
            solver->addRow(simpleConditionUB,IPLess,0 );
            solver->addRow(simpleConditionLB,IPGreater,0);
        }
    }
}

void IPStateBasedUnified::numericInitialStateConstraint(){
    if (debug) cout << "adding numericInitialStateConstraint" << endl;
    // get variables from rpg
    for (int iV = 0; iV < RPGBuilder::getPNECount(); ++iV){
        stringstream name;
        PNE *p = RPGBuilder::getPNE(iV);
        solver->addRow(indexPNE[{p, 0}],1,IPContinous,IPEqual,theState.second[iV]);
    }
}

void IPStateBasedUnified::numericGoalStateConstraint(){
    if (debug) cout << "adding numericGoalStateConstraint" << endl;
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto goal : numericGoals){
        if (goal.first!=-1){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[goal.first];
            int lhs = prec.LHSVariable;
            double rhs = prec.RHSConstant;
            if (lhs < RPGBuilder::getPNECount()){
                // x > 0
                if (debug) cout << "first case " << lhs << endl;
                double epsilon = 0;
                if (prec.op == VAL::E_GREATER){
                    epsilon+=minEffect[goal.first];
                }
                solver->addRow(indexPNE[{RPGBuilder::getPNE(lhs), maxT-1}],1,IPContinous,IPGreater,rhs + epsilon);
                toErase.push_back(solver->getNConstraint());
            }else if (lhs < 2*RPGBuilder::getPNECount()){
                // x < 0
                if (debug) cout << "second case " << lhs << endl;
                double epsilon = 0;
                if (prec.op == VAL::E_GREATER){
                    epsilon+minEffect[goal.first];
                }
                solver->addRow(indexPNE[{RPGBuilder::getPNE(lhs - RPGBuilder::getPNECount()), maxT-1}],1,IPContinous,IPLess,rhs - epsilon);
                toErase.push_back(solver->getNConstraint());
            }else{
                // get artificial variable
                if (debug)cout << "third case " << lhs << endl;
                RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(lhs);
                if (debug) cout << av << " " << rhs << endl;
                vector<double> weights = av.weights;
                vector<int> fluents = av.fluents;
                double constant = av.constant;
                list<IPSolver::IPConstraint> toSum;
                for (int i = 0; i<weights.size(); ++i){
                    //cout << i << " " << weights[i] << " " << fluents[i] << endl;
                    int index = fluents[i];
                    double w = weights[i];
                    if (fluents[i] >= RPGBuilder::getPNECount()){
                        w = -1*w;
                        index-=RPGBuilder::getPNECount();
                    }
                    const double k = w;
                    toSum.push_back({IPContinous,indexPNE[{RPGBuilder::getPNE(index), maxT-1}],k});
                }
                double epsilon = 0;
                if (prec.op == VAL::E_GREATER){
                    epsilon+=minEffect[goal.first];
                }
                solver->addRow(toSum, IPGreater, rhs - constant + epsilon);
                toErase.push_back(solver->getNConstraint());
            }
        }
    }
}

void IPStateBasedUnified::numericNoopConstraint(int tMin, int tMax){
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
            for (int t = tMin; t<tMax; ++t){
                list<IPSolver::IPConstraint> toAdd;
                toAdd.push_back({IPBoolean, indexPNENoop[{RPGBuilder::getPNE(fluentIndex),t}], 1});
                toAdd.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], 1});
                solver->addRow(toAdd, IPLess,1);
            }
        }
    }
    // fluent actions
    for (int t = tMin; t < tMax; ++t){
        for (int i = 0; i < RPGBuilder::getPNECount(); ++i){
            //for (auto index : indexFluentActions){
            int indexFluent = i;
            list<int> actions = indexFluentActions[i];
            list<IPSolver::IPConstraint> toSum;
            toSum.push_back({IPBoolean,indexPNENoop[{RPGBuilder::getPNE(indexFluent),t}],1});
            for (auto iA : actions){
                toSum.push_back({IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],1});
            }
            solver->addRow(toSum, IPEqual,1);
            if (actions.size()==0){
                list<IPSolver::IPConstraint> toSum;
                toSum.push_back({IPContinous,indexPNE[{RPGBuilder::getPNE(indexFluent),t}],1});
                toSum.push_back({IPContinous,indexPNE[{RPGBuilder::getPNE(indexFluent),0}],-1});
                solver->addRow(toSum, IPEqual,0);
            }
        }
    }
}

void IPStateBasedUnified::numericMutex(int tMin, int tMax){
    if (debug) cout << "adding numericMutex on preconditions" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEffI = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        list<int> mutex;
        for (int jA = 0; jA < RPGBuilder::getNOp(); ++jA){
            // indicate that iA and jA are mutex
            bool isMutex = false;
            if (iA == jA)
                continue;
            // TODO: check
            list<int> numEffJ = RPGBuilder::getActionsToRPGNumericStartPreconditions()[jA];
            for (auto nEff : numEffI){
                int indexFluentI = RPGBuilder::getNumericEff()[nEff].fluentIndex;
                for (auto nEffJ : numEffJ){
                    RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[nEffJ];
                    pair<list<pair<int,double> >,double> precondition = FukunagaAnalysis::getExpression(prec);
                    for(auto indexFluent : precondition.first){
                        if (indexFluent.second==0) continue;
                        int indexFluentJ = indexFluent.first;
                        if (indexFluentI == indexFluentJ){
                            isMutex = true;
                            //cout << "mutex because action " << *RPGBuilder::getInstantiatedOp(iA) << " has an effect on variable " << *(RPGBuilder::getPNE(indexFluentI)) << " and action " << *RPGBuilder::getInstantiatedOp(jA) << " has precondition " << prec << " " << indexFluent.first << " " << indexFluent.second << endl;
                            break;
                        }
                        if (isMutex) break;
                    }
                } // end numeric precondition
                if (isMutex) break; // no need to check other effects
            } // end numeric effect
            if (isMutex){
                if (debug)
                    cout << "actions " << *RPGBuilder::getInstantiatedOp(iA) << " and " << *RPGBuilder::getInstantiatedOp(jA) << " are mutex"<<  endl;
                mutex.push_back(jA);
                for (int t = tMin; t < tMax; ++t){
                    list<IPSolver::IPConstraint> toAdd;
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],1});
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(jA),t}],1});
                    solver->addRow(toAdd, IPLess, 1);
                }
            }
        }
//        for (int t = tMin; t < tMax; ++t){
//            list<IPSolver::IPConstraint> toAdd;
//            toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],1});
//            for (auto jA : mutex){
//                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(jA),t}],1});
//            }
//            solver->addRow(toAdd, IPLess, 1);
//        }
    }
}

void IPStateBasedUnified::numericMutexEffects(int tMin, int tMax){
    if (debug) cout << "adding numericMutex effects" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        list<int> mutex;
        for (int jA = 0; jA < RPGBuilder::getNOp(); ++jA){
            // indicate that iA and jA are mutex
            bool isMutex = false;
            if (iA == jA)
                continue;
            // TODO: check
            list<int> numEffJ = RPGBuilder::getActionsToRPGNumericStartEffects()[jA];
            for (auto nEff : numEff){
                int indexFluent = RPGBuilder::getNumericEff()[nEff].fluentIndex;
                if (!variableActionLinear[indexFluent][iA] && !variableActionLinear[indexFluent][jA]) continue; // if simple effect, ignore mutex effect
                for (auto nEffJ : numEffJ){
                    vector<int> affectedJ =  RPGBuilder::getNumericEff()[nEffJ].variables;
                    for (auto lhs : affectedJ){
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
                    }
                    if (isMutex) break;
                }
                if (isMutex) break; // no need to check other effects
            } // end numeric effect
            if (isMutex){
                mutex.push_back(jA);
                if (debug) cout << "actions " << *RPGBuilder::getInstantiatedOp(iA) << " and " << *RPGBuilder::getInstantiatedOp(jA) << " are mutex (for effects) "<<  endl;
                for (int t = tMin; t < tMax; ++t){
                    list<IPSolver::IPConstraint> toAdd;
                    toAdd.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], 1});
                    toAdd.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(jA),t}], 1});
                    solver->addRow(toAdd, IPLess,1);
                }
            }
        }
//        for (int t = tMin; t < tMax; ++t){
//            list<IPSolver::IPConstraint> toAdd;
//            toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],1});
//            for (auto jA : mutex){
//                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(jA),t}],1});
//            }
//            solver->addRow(toAdd, IPLess, 1);
//        }
    }
}

void IPStateBasedUnified::symmetryEnd(int tMin, int tMax){
    if (debug) cout << "adding sequential action constraint" << endl;
    for (int t = tMin; t < tMax; ++t){
        list<IPSolver::IPConstraint> toSum;
        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
            instantiatedOp *action = RPGBuilder::getInstantiatedOp(a);
            toSum.push_back({IPBoolean,indexAction[{action,t}],1});
        }
        solver->addRow(toSum, IPGreater, 1);
        toErase2.push_back(solver->getNConstraint());
    }
}



void IPStateBasedUnified::incrementTimeHorizon(MinimalState & state, int increment ){
    int start = maxT;
    int end = maxT + increment;
    maxT += increment;
    FukunagaAnalysis::getTimeStampedMaxValuesGeneral(minMaxValues, minMaxSteps,minEffect, maxT,theState);
//    for (int i = 0; i < RPGBuilder::getPNECount();++i)
//        cout << *RPGBuilder::getPNE(i) << " " << maxT-1 << " " << minMaxValues[maxT-1][i].first << " " << minMaxValues[maxT-1][i].second << endl;
    // add new variables
    addVariables(start,end);
    solver->update();
    static bool firstIt = true;
    
    {
    
        actionPreconditions(start,end);
        actionsEffects(start-1,end);
        mutexRelations(start,end);
        noopPreconditions(start,end);
        noopMutex(start,end);
        noopMutexAdd(start,end);
        
        //noopForcedIfNoActionsAdd();
        //noopMutexAdd();
        
        numericPreconditions(start,end);
        numericEffects(start-1,end);
        //numericNoopConstraint(start,end);
        //numericNoopEffects(start-1,end);
        numericMutex(start,end);
        numericMutexEffects(start,end);
        
        
        if (RPGBuilder::useExtraConstraints){
            addRelevanceConstraints(start,end);
            //variablesBounds();
        }
        if (checkOptimal  && RPGBuilder::useValidityInequality){
            sequentialActions(0,end);
            firstActions(0,end);
        }

//        if (checkOptimal){
//            for (int e : toErase2){
//                solver->eliminateConstraint(e);
//            }
//        }else{
//            if (firstIt){
//                firstIt = false;
//                symmetryEnd(0,start);
//            }else{
//                symmetryEnd(start,end);
//            }
//        }
        for (int e : toErase){
            solver->eliminateConstraint(e);
        }
        
        toErase.clear();
        goalStateConstraint();
        numericGoalStateConstraint();
        if (MILPRPG::addLandmarks){
            addLandmarksConstraints(0,end);
        }
        if (RPGBuilder::useValidityInequality){
            lastLayer(maxT);
            forcePropositionsNotSelected(start,end);
        }
        
    }
    
    // delete objective function
    solver->eliminateObjective();
    objectiveFunction(0, maxT);
}

//void IPStateBasedUnified::extractSolution(){
//    cout << "printing solution " << endl;
//    for (int t = 0; t < maxT; ++t){
//        for (int a = 0; a < Planner::RPGBuilder::getNOp(); ++a){
//            double isSelected = solver->getValue(IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(a),t}]);
//            //cout << a << " " << t  << " " << isSelected << endl;
//            if (isSelected>0){
//                stringstream name;
//                name << *RPGBuilder::getInstantiatedOp(a);
//                solution << (t)/1000. << ": " << name.str().c_str() << " [0.001]" << endl;
//            }
//        }
//    }
//
//    for (int v = 0; v < Planner::RPGBuilder::getPNECount(); ++v){
//        for (int t = 0; t < maxT; ++t){
//            double isSelected = solver->getValue(IPContinous, indexPNE[{RPGBuilder::getPNE(v),t}]);
//            cout << *RPGBuilder::getPNE(v) << " " << t  << " " << isSelected << endl;
//        }
//    }
//}

