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

#include "IPStateBasedModel.hpp"
#include "NumericAnalysis.h"
#include "MILPRPG.h"
#include "VariableElimination.hpp"

//TODO:
// mutex noopt in precondition
// mutex clique
// consecutive actions

IPStateBasedModel::IPStateBasedModel(MinimalState & state, int horizon, vector<bool> & aL, vector<bool> & fL, vector<bool> & aE, vector<bool> & fE, bool d) : IPModel(horizon,d), maxPossible(Planner::RPGBuilder::getPNECount()),minPossible(Planner::RPGBuilder::getPNECount()), mT(horizon), theState(state), toErase() {
    B = 1000000;
    maxUse = 0;
    actionLandmarks = aL;
    factLandmarks = fL;
    eliminatedActions = aE;
    eliminatedFacts = fE;
    for (int iV = 0; iV < Planner::RPGBuilder::getPNECount(); ++iV){
        maxPossible[iV] = B;
        minPossible[iV] = -B;
    }
}

void IPStateBasedModel::initialiseVariables(){
    if (debug) cout << "initialising variables 1" << endl;
    iBool = 0;
    iBoolAux = 0;
    iInt = 0;
    iCount = 0;
    // actions
    addVariables(0,maxT);
}

void IPStateBasedModel::addVariables(int tMin, int tMax){
    for (int t = tMin; t < tMax; ++t){
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            
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
        
        // count actions
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            
            stringstream name;
            instantiatedOp *a = RPGBuilder::getInstantiatedOp(iA);
            name << "m_" << *a << "_" << t;
            indexAction.insert({{a,t}, iInt++});
            if (t < maxT-1) solver->addCol(IPInteger, 0, B, name.str());
            else solver->addCol(IPInteger, 0, 1, name.str());
            
        }
        // numeric condition
        for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
            // uc
            
            stringstream name;
            name << "uc_" << i << "_" << t;
            solver->addCol(IPBooleanAux,0,1,name.str());
            uc.insert({{i,t}, iBoolAux++});
        }
    }
}
void IPStateBasedModel::buildConstraints(){
    FukunagaAnalysis::getTimeStampedMaxValues(minMaxValues, minMaxSteps, maxT+1,theState);
    initialStateConstraint();
    goalStateConstraint();
    objectiveFunction(0,maxT);
    actionPreconditions(0,maxT);
    actionsEffects(0,maxT);
    mutexRelations(0,maxT);
    noopPreconditions(0,maxT);
    noopMutex(0,maxT);
    if (RPGBuilder::useValidityInequality){
        lastLayer(maxT);
        forcePropositionsNotSelected(0, maxT);
        noopMutexAdd(0,maxT);
    }
    
    numericActionPreconditions(0,maxT);
    numericPreconditions(0,maxT);
    numericGoalConditions();
    //numericMutex();
    numericMutexPrecondition(0,maxT);

    if (RPGBuilder::useExtraConstraints){
        addRelevanceConstraints(0,maxT);
        addNumericRelevanceConstraints(0,maxT);
        //variablesBounds();
    }
    if (MILPRPG::addLandmarks){
        addLandmarksConstraints(0,maxT);
        addNumericLandmarksConstraints(0,maxT);
    }
    if (checkOptimal){
        sequentialActions(0,maxT);
        firstActions(0,maxT);
    }
}

void IPStateBasedModel::extractSolution(){
    cout << "printing solution " << endl;
    for (int t = 0; t < maxT; ++t){
        for (int a = 0; a < Planner::RPGBuilder::getNOp(); ++a){
            double isSelected = solver->getValue(IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(a),t}]);
            //cout << a << " " << t  << " " << isSelected << endl;
            if (isSelected>0.00001){
                stringstream name;
                name << *RPGBuilder::getInstantiatedOp(a);
                solution << (t)/1000. << ": " << name.str().c_str() << " [0.001]" << endl;
            }
        }
    }
}

int IPStateBasedModel::extractSolutionLength(){
    int pL = 0;
    for (int t = 0; t < maxT; ++t){
        for (int a = 0; a < Planner::RPGBuilder::getNOp(); ++a){
            double isSelected = solver->getValue(IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(a),t}]);
            //cout << a << " " << t  << " " << isSelected << endl;
            if (isSelected>0.00001){
                pL++;
            }
        }
    }
    return pL;
}

void IPStateBasedModel::updateConstraints(MinimalState &state){
    
}

void IPStateBasedModel::initialStateConstraint(){
    if (debug) cout << "adding initial state" << endl;
    // get variables from rpg
    for (int li = 0; li < RPGBuilder::getNLiterals(); ++li){
        Literal *l = RPGBuilder::getLiteral(li);
        set<int> & propositions = theState.first;
        if (find(propositions.begin(),propositions.end(), li) != propositions.end())
            solver->addRow(indexLiteral[{l,0}],1,IPBooleanAux, IPEqual, 1);
        else
            solver->addRow(indexLiteral[{l,0}],1,IPBooleanAux, IPEqual, 0);
    }
}

void IPStateBasedModel::goalStateConstraint(){
    if (debug) cout << "adding goalStateConstraint" << endl;
    for (auto l : RPGBuilder::getLiteralGoals()){
        solver->addRow(indexLiteral[{l, maxT - 1}],1,IPBooleanAux, IPEqual, 1);
        toErase.push_back(solver->getNConstraint());
    }
}

void IPStateBasedModel::objectiveFunction(int tMin, int tMax){
    if (debug) cout << "objectiveFunction" << endl;

    list<IPSolver::IPConstraint> obj;

    for (int t = tMin; t < tMax; ++t){
        // actions
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            double cost = RPGBuilder::usePlanLength ? 1 : RPGBuilder::getActionCost()[iA];
            if (cost == 0.0001) cost = 0;
            const double w = cost;
            obj.push_back({IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],w});
        }
    }
    solver->addObjective(obj);
}

// actionPreconditions
void IPStateBasedModel::actionPreconditions(int tMin, int tMax){
    if (debug) cout << "adding actionPreconditions" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        for (int t = tMin; t < tMax; ++t){
            for (auto prec : RPGBuilder::getStartPropositionalPreconditions()[iA]){
                list<IPSolver::IPConstraint> toAdd;
                toAdd.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(iA),t}], 1});
                toAdd.push_back({IPBooleanAux, indexLiteral[{prec,t}], -1});
                solver->addRow(toAdd, IPLess,0);
            }
        }
    }
}

// actionsEffects
void IPStateBasedModel::actionsEffects(int tMin, int tMax){
    if (debug) cout << "adding actionsEffects" << endl;
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        Literal *l = RPGBuilder::getLiteral(iL);
        list<int> idAddActions;
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            list<Literal *> addList = RPGBuilder::getStartAddEffects()[iA];
            if (find(addList.begin(),addList.end(),l) != addList.end()) {
                idAddActions.push_back(iA);
            }
        }
        for (int t = tMin; t < tMax - 1; ++t){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPBooleanAux,indexLiteral[{l,t+1}],1});
            for (auto a : idAddActions ){
                toAdd.push_back({IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(a),t}],-1});
            }
            toAdd.push_back({IPBoolean,indexActionNoop[{l,t}],-1});
            solver->addRow(toAdd, IPLess, 0);
        }
    }
}

// actionsEffects
void IPStateBasedModel::forcePropositionsNotSelected(int tMin, int tMax){
    if (debug) cout << "adding forcePropositionsNotSelected" << endl;
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        Literal *l = RPGBuilder::getLiteral(iL);
        list<int> idAddActions;
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            list<Literal *> addList = RPGBuilder::getStartAddEffects()[iA];
            if (find(addList.begin(),addList.end(),l) != addList.end()) {
                idAddActions.push_back(iA);
            }
        }
        for (int t = tMin; t < tMax - 1; ++t){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({ IPBooleanAux,indexLiteral[{l,t+1}],1});
            toAdd.push_back({ IPBooleanAux,indexLiteral[{l,t}],-1});
            for (auto a : idAddActions ){
                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(a),t}],-1});
            }
            toAdd.push_back({ IPBoolean,indexActionNoop[{l,t}],-1});
            solver->addRow(toAdd,IPLess,0);
        }
    }
}

// mutexRelations
void IPStateBasedModel::mutexRelations(int tMin, int tMax){
    if (debug) cout << "adding mutexRelations" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<Literal *> delList = RPGBuilder::getStartDeleteEffects()[iA];
        for (int jA = 0; jA < RPGBuilder::getNOp(); ++jA){
            if (iA == jA)
                continue;
            list<Literal *> addList = RPGBuilder::getStartAddEffects()[jA];
            list<Literal *> preList = RPGBuilder::getStartPropositionalPreconditions()[jA];
            bool isMutex = false;
            Literal *mutex;
            for (auto l : delList){
                if(
                   find(addList.begin(),addList.end(),l) != addList.end() ||
                   find(preList.begin(),preList.end(),l) != preList.end()
                   ){
                    mutex = l;
                    isMutex = true;
                    break;
                }
            }
            if (isMutex){
                if (debug) cout << "actions " << *RPGBuilder::getInstantiatedOp(iA) << " and " << *RPGBuilder::getInstantiatedOp(jA) << " are mutex because of " << *mutex <<  endl;
                for (int t = tMin; t < tMax; ++t){
                    list<IPSolver::IPConstraint> toAdd;
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA), t}],1});
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(jA), t}],1});
                    solver->addRow(toAdd, IPLess, 1);
                }
            }
        }
    }
    
}

void IPStateBasedModel::noopPreconditions(int tMin, int tMax){
    if (debug) cout << "adding actionPreconditions" << endl;
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        Literal *l = RPGBuilder::getLiteral(iL);
        for (int t = tMin; t < tMax; ++t){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({ IPBoolean,indexActionNoop[{l,t}],1});
            toAdd.push_back({ IPBooleanAux,indexLiteral[{l,t}],-1});
            solver->addRow(toAdd, IPLess, 0);
        }
    }
}

//void IPStateBasedModel::noopMutex(int tMin, int tMax){
//    if (debug) cout << "adding noopMutex" << endl;
//    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
//        list<Literal *> delList = RPGBuilder::getStartDeleteEffects()[iA];
//        //cout << *RPGBuilder::getInstantiatedOp(iA) << endl;
//        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
//            Literal *l = RPGBuilder::getLiteral(iL);
//            //cout << "\t" << *l << endl;
//            if (find(delList.begin(),delList.end(),l) != delList.end()){
//                //cout << "\t\tismutex" << endl;
//                for (int t = tMin; t < tMax - 1; ++t)
//                    model.add(x.x[getIndex(iA, t)] +  x.x[indexActionNoop[{l, t}]] <= 1);
//            }
//        }
//    }
//}


void IPStateBasedModel::noopMutex(int tMin, int tMax){
    if (debug) cout << "adding noopMutex" << endl;
    for (int t = tMin; t < tMax; ++t){
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            Literal *l = RPGBuilder::getLiteral(iL);
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({ IPBoolean,indexActionNoop[{l, t}],1});
            list<pair<int, VAL::time_spec> > delList = RPGBuilder::getNegativeEffectsToActions()[iL];
            for (auto& action : delList){
                int iA = action.first;
                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],1});
            }
            solver->addRow(toAdd, IPLess, 1);
        }
    }
}


void IPStateBasedModel::noopMutexAdd(int tMin, int tMax){
    if (debug) cout << "adding noopMutex" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<Literal *> delList = RPGBuilder::getStartAddEffects()[iA];
        //cout << *RPGBuilder::getInstantiatedOp(iA) << endl;
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            Literal *l = RPGBuilder::getLiteral(iL);
            //cout << "\t" << *l << endl;
            if (find(delList.begin(),delList.end(),l) != delList.end()){
                //cout << "\t\tismutex" << endl;
                for (int t = tMin; t < tMax; ++t){
                    list<IPSolver::IPConstraint> toAdd;
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA), t}],1});
                    toAdd.push_back({ IPBoolean,indexActionNoop[{l, t}],1});
                    solver->addRow(toAdd, IPLess, 1);
                }
            }
        }
    }
}

void IPStateBasedModel::noopForcedIfNoActionsAdd(int tMin, int tMax){
    if (debug) cout << "adding noopMutex" << endl;
    for (int t = tMin; t < tMax - 2; ++t){
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            Literal *l = RPGBuilder::getLiteral(iL);
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({ IPBoolean,indexActionNoop[{l, t}],1});
            for (int iA = 0; iA< RPGBuilder::getNOp(); ++iA){
                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA), t+1}],1});
            }
            solver->addRow(toAdd, IPGreater, 1);
        }
    }
}



void IPStateBasedModel::numericActionPreconditions(int tMin, int tMax){
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        list<int> preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[i];
        for (auto precID : preconditions){
            for (int t = tMin; t < tMax; ++t){
                list<IPSolver::IPConstraint> toAdd;
                toAdd.push_back({ IPBooleanAux,uc[{precID, t}],1});
                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(i),t}],-1});
                solver->addRow(toAdd, IPGreater, 0);
            }
        }
    }
}

void IPStateBasedModel::numericPreconditions(int tMin, int tMax){
    if (debug) cout << "numericPreconditions" << endl;
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        for (int t = tMin; t < tMax; ++t){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
            if (debug) cout << i << " " << prec << " " << prec.op << endl;
            pair<list<pair<int,double> >,double> precondition = FukunagaAnalysis::getExpression(prec);
            list<pair<int,double> > coefficient = precondition.first;
            double rhs = -precondition.second;
            if (debug) cout << "rhs" << rhs << endl;
            list<IPSolver::IPConstraint> actionSum;
            double constantCoefficient = rhs;
            const double maxMinEff = 999999;
            vector<double> minEffect(RPGBuilder::getNOp(), 0);
            double lowerbound = 0;
            bool toAdd = false;
            for (auto prec : coefficient){
                int v = prec.first;
                double w = prec.second;
                if (debug) cout << "\tconsidering fluent " << v << " of " << RPGBuilder::getPNECount() << endl;//" " << *RPGBuilder::getPNE(v) << endl;
                if ( abs(w)< 0.000001) continue;
                toAdd = true;
                for (int ji = 0; ji < RPGBuilder::getNOp(); ++ji){
                    int j = ji;
                    list<int> numEff = RPGBuilder::RPGBuilder::getActionsToRPGNumericStartEffects()[j];
                    if (debug) cout << "\taction " << *RPGBuilder::getInstantiatedOp(j) << " has " << numEff.size() << " effects on numeric variables " << endl;
                    double constIncrease = 0;
                    for (int idEff : numEff){
                        RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[idEff];
                        int fluentIndex = eff.fluentIndex;
                        if (fluentIndex == v){
                            if (debug) cout << "\t\t affected by " << *RPGBuilder::getInstantiatedOp(j) << endl;
                            vector<double> weights = eff.weights;
                            vector<int> variables = eff.variables;
                            for (int z = 0; z < variables.size(); ++z){
                                if ( abs(weights[z]) >= 0.000001 && variables[z] != v) cout << "error: there is a non-simple numeric condition" << endl;
                                //if (variables[z] == v) constIncrease = weights[z];
                            }
                            if (!eff.isAssignment) constIncrease += eff.constant;
                        }
                    }
                    minEffect[j] += w * constIncrease;
                    
                    for (int iT = 0; iT < t; ++iT){
                        actionSum.push_back({IPBoolean, indexAction[{RPGBuilder::getInstantiatedOp(j),iT}],  w * constIncrease });
                    }
                }
                //if (debug) cout << v << " : w " << w << " value " << tinitialFluents[v] << endl;
                constantCoefficient += w * theState.second[v];
                if (w >= 0)
                    lowerbound += w * minMaxValues[t][v].first;
                else
                    lowerbound += w * minMaxValues[t][v].second;
            }
            
            if (toAdd){
                lowerbound += rhs;
                if (prec.op == VAL::E_GREATER){
                    //                    cout << "strictly disequality" << endl;
                    //                    if (constantCoefficient <= 0) constantCoefficient+=0.000001;
                    //                    else
                    double minConstantCoefficient = maxMinEff;
                    //cout << minConstantCoefficient << " ";
                    //if (i==12) cout << "test " << endl;
                    for (int ji = 0; ji < RPGBuilder::getNOp(); ++ji){
                        //if (i==12) cout << *RPGBuilder::getInstantiatedOp(ji) << " " << minEffect[ji] << endl;
                        if (abs(minEffect[ji])>0.0001 && abs(minEffect[ji]) < minConstantCoefficient){
                            minConstantCoefficient = abs(minEffect[ji]);
                            //if (i==12)  cout << "new min " << abs(minEffect[ji]) << endl;
                        }
                    }
                    //if (i==12) cout << "total min " << minConstantCoefficient << endl;
                    if (minConstantCoefficient!=maxMinEff){
                        constantCoefficient-=minConstantCoefficient;
                        lowerbound-=minConstantCoefficient;
                    }else{
                        constantCoefficient-=0.00001;
                        lowerbound-=0.00001;
                    }
                    //if (i==6) cout << prec << " " << lowerbound << " " << constantCoefficient <<" "<< rhs << " " << minConstantCoefficient << endl;
                }
                //if (abs(lowerbound)>0.0001)
                {
                    actionSum.push_back({IPBooleanAux, uc[{i,t}], lowerbound});
                    solver->addRow(actionSum, IPGreater, -constantCoefficient + lowerbound);
                }
            }
        }
    }
}

void IPStateBasedModel::numericGoalConditions(){
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto goal : numericGoals){
        if (goal.first!=-1){
            solver->addRow(uc[{goal.first,maxT-1}],1,IPBooleanAux,IPEqual,1);
            toErase.push_back(solver->getNConstraint());
        }
    }
}

void IPStateBasedModel::numericMutex(int tMin, int tMax){
    if (debug) cout << "adding numericMutex" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEffI = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        for (int jA = 0; jA < RPGBuilder::getNOp(); ++jA){
            // indicate that iA and jA are mutex
            bool isMutex = false;
            if (iA == jA)
                continue;
            // TODO: check
            list<int> numEffJ = RPGBuilder::getActionsToRPGNumericStartEffects()[jA];
            for (auto nEff : numEffI){
                int indexFluentI = RPGBuilder::getNumericEff()[nEff].fluentIndex;
                for (auto nEffJ : numEffJ){
                    int indexFluentJ = RPGBuilder::getNumericEff()[nEffJ].fluentIndex;
                    if (indexFluentI == indexFluentJ ){
                        isMutex = true;
                        break;
                    }
                } // end numeric precondition
                if (isMutex) break; // no need to check other effects
            } // end numeric effect
            if (isMutex){
                if (debug) cout << "actions " << *RPGBuilder::getInstantiatedOp(iA) << " and " << *RPGBuilder::getInstantiatedOp(jA) << " are mutex"<<  endl;
                for (int t = tMin; t < tMax; ++t){
                    list<IPSolver::IPConstraint> toAdd;
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA), t}],1});
                    toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(jA), t}],1});
                    solver->addRow(toAdd, IPLess, 1);
                }
            }
        }
    }
}

void IPStateBasedModel::numericMutexPrecondition(int tMin, int tMax){
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
        for (int t = tMin; t < tMax; ++t){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(iA),t}],1});
            for (auto jA : mutex){
                toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(jA),t}],1});
            }
            solver->addRow(toAdd, IPLess, 1);
        }
    }
}


void IPStateBasedModel::lastLayer(int tMax){
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        list<IPSolver::IPConstraint> toAdd;
        toAdd.push_back({ IPBoolean,indexAction[{RPGBuilder::getInstantiatedOp(i),tMax-1}],1});
        solver->addRow(toAdd, IPEqual, 0);
        toErase.push_back(solver->getNConstraint());
    }
}


void IPStateBasedModel::addLandmarksConstraints(int tMin, int tMax){
    //bool debug = true;
    if (debug) cout << "adding addLandmarksConstraints" << endl;
    for (int i = 0; i < actionLandmarks.size(); ++i){
        if (actionLandmarks[i]){
            if (debug) cout << *RPGBuilder::getInstantiatedOp(i) << " is an action landmark" << endl;
            list<IPSolver::IPConstraint> toAdd;
            instantiatedOp* action = RPGBuilder::getInstantiatedOp(i);
            for (int t = tMin; t<tMax; ++t){
                toAdd.push_back({IPBoolean,indexAction[{action,t}],1});
            }
            solver->addRow(toAdd, IPGreater,1);
            toErase.push_back(solver->getNConstraint());
        }
    }
    
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        Literal *l = RPGBuilder::getLiteral(iL);
        int p = iL;
        if (factLandmarks[p]){
            if (debug ) cout << *l << " is a fact landmark" << endl;
            list<IPSolver::IPConstraint> toAdd;
            for (int t = tMin; t<tMax; ++t){
                toAdd.push_back({IPBooleanAux,indexLiteral[{l,t}],1});
            }
            solver->addRow(toAdd, IPGreater,1);
            toErase.push_back(solver->getNConstraint());
        }
    }
    
}

void IPStateBasedModel::addNumericLandmarksConstraints(int tMin, int tMax){
    //bool debug = true;
    if (debug) cout << "adding addNumericLandmarksConstraints" << endl;
    
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        int precID = i;
        int idL = precID + RPGBuilder::getNLiterals();
        if (factLandmarks[idL]){
            if (debug ) cout << idL << " is a numeric condition landmark" << endl;
            list<IPSolver::IPConstraint> toAdd;
            for (int t = tMin; t < tMax; ++t){
                toAdd.push_back({IPBooleanAux,uc[{precID, t}],1});
            }
            solver->addRow(toAdd, IPGreater,1);
            toErase.push_back(solver->getNConstraint());
        }
    }
}


void IPStateBasedModel::addRelevanceConstraints(int tMin, int tMax){
    //bool debug = true;
    if (debug) cout << "adding eliminatedActions" << endl;
    for (int i = 0; i < eliminatedActions.size(); ++i){
        if (!eliminatedActions[i]){
            if (debug) cout << *RPGBuilder::getInstantiatedOp(i) << " can be eliminated" << endl;
            list<IPSolver::IPConstraint> toAdd;
            instantiatedOp* action = RPGBuilder::getInstantiatedOp(i);
            for (int t = tMin; t<tMax; ++t){
                toAdd.push_back({IPBoolean, indexAction[{action,t}],1});
            }
            solver->addRow(toAdd, IPEqual,0);
        }
    }
    
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        Literal *l = RPGBuilder::getLiteral(iL);
        int p = iL;
        if (!eliminatedFacts[p]){
            if (debug ) cout << *l << " is a fact that can be eliminated" << endl;
            list<IPSolver::IPConstraint> toAdd;
            for (int t = tMin; t<tMax; ++t){
                toAdd.push_back({IPBooleanAux,indexLiteral[{l,t}],1});
            }
            solver->addRow(toAdd, IPEqual,0);
        }
    }
}

void IPStateBasedModel::addNumericRelevanceConstraints(int tMin, int tMax){
    //bool debug = true;
    if (debug) cout << "adding addNumericRelevanceConstraints" << endl;

    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        int precID = i;
        int idL = precID + RPGBuilder::getNLiterals();
        if (!eliminatedFacts[idL]){
            if (debug ) cout << idL << " is a numeric condition  that can be eliminated" << endl;
            list<IPSolver::IPConstraint> toAdd;
            for (int t = tMin; t < tMax; ++t){
                toAdd.push_back({IPBooleanAux,uc[{precID, t}],1});
            }
            solver->addRow(toAdd, IPEqual,0);
        }
    }
}
void IPStateBasedModel::variablesBounds(int tMin, int tMax){
//    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
//    IloEnv env  = model.getEnv();
//
//    // add variables bounds
//    for (int i = 0; i < RPGBuilder::getPNECount(); ++i){
//        IloExprArray toSum(env);
//
//        //if (NumericAnalysis::getBounds().first==-DBL_MAX) boundMin = false;
//        //if (NumericAnalysis::getBounds().second==DBL_MAX) boundMin = false;
//        for (int j = 0; j < RPGBuilder::getNOp(); ++j){
//            instantiatedOp *action = RPGBuilder::getInstantiatedOp(j);
//            list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
//            if (debug) cout << "\taction " << *RPGBuilder::getInstantiatedOp(j) << " has " << numEff.size() << " effects on numeric variables " << endl;
//            double constIncrease = 0;
//            for (int idEff : numEff){
//                RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[idEff];
//                int fluentIndex = eff.fluentIndex;
//                if (fluentIndex == i){
//                    if (debug) cout << "\t\t affected by " << *RPGBuilder::getInstantiatedOp(j) << endl;
//                    vector<double> weights = eff.weights;
//                    vector<int> variables = eff.variables;
//                    for (int z = 0; z < variables.size(); ++z){
//                        if ( abs(weights[z]) >= 0.000001 && variables[z] != i) cout << "error: there is a non-simple numeric condition" << endl;
//                        //if (variables[z] == v) constIncrease = weights[z];
//                    }
//                    if (!eff.isAssignment) constIncrease += eff.constant;
//                }
//            }
//            for (int t = 0; t<maxT; ++t){
//                toSum.add(constIncrease*x.x[getIndex(indexAction[action],t)]);
//            }
//        }
//        if (NumericAnalysis::getBounds()[i].first!=-DBL_MAX)
//            model.add(IloSum(toSum) + theState.second[i] >= NumericAnalysis::getBounds()[i].first);
//        if (NumericAnalysis::getBounds()[i].second!=DBL_MAX)
//            model.add(IloSum(toSum) + theState.second[i] <= NumericAnalysis::getBounds()[i].second);
//
//    }
}

void IPStateBasedModel::sequentialActions(int tMin, int tMax){
    if (debug) cout << "adding sequential action constraint" << endl;
    for (int t = tMin; t < tMax; ++t){
        list<IPSolver::IPConstraint> toSum;
        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
            instantiatedOp *action = RPGBuilder::getInstantiatedOp(a);
            toSum.push_back({IPBoolean,indexAction[{action,t}],1});
        }
        solver->addRow(toSum, IPLess, 1);

    }
}

void IPStateBasedModel::firstActions(int tMin, int tMax){
    if (debug) cout << "adding sequential action constraint" << endl;
    for (int t = tMin; t < tMax - 1; ++t){
        list<IPSolver::IPConstraint> toSum;
        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
            instantiatedOp *action = RPGBuilder::getInstantiatedOp(a);
            toSum.push_back({IPBoolean,indexAction[{action,t}],1});
            toSum.push_back({IPBoolean,indexAction[{action,t+1}],-1});
        }
        solver->addRow(toSum, IPGreater, 0);
    }
}

void IPStateBasedModel::addMIPStart(list<int> solution){
//    addWarmStart = true;
//    // index of action
//    int t = 0;
//    IloEnv env  = model.getEnv();
//    startVar = IloNumVarArray(env);
//    startVal = IloNumArray(env);
//    for (auto aS : solution){
//        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
//            instantiatedOp *action = RPGBuilder::getInstantiatedOp(a);
//            startVar.add(x.x[getIndex(indexAction[action],t)]);
//            if (a==aS){
//                startVal.add(1);
//            }else{
//                startVal.add(0);
//            }
//        }
//        t++;
//    }
}


void IPStateBasedModel::incrementTimeHorizon(MinimalState & state, int increment){
    int start = maxT;
    int end = maxT + increment;
    maxT += increment;
    FukunagaAnalysis::getTimeStampedMaxValues(minMaxValues, minMaxSteps, maxT+1,theState);

    // add new variables
    addVariables(start,end);
    solver->update();
    // add constraints on new variables
    {
        actionPreconditions(start,end);
        actionsEffects(start-1,end);
        mutexRelations(start,end);
        noopPreconditions(start,end);
        noopMutex(start,end);
        
        numericActionPreconditions(start,end);
        numericPreconditions(start,end);
        numericMutexPrecondition(start,end);
        
        if (RPGBuilder::useExtraConstraints){
            addRelevanceConstraints(start,end);
            addNumericRelevanceConstraints(start,end);
            //variablesBounds();
        }
        if (checkOptimal && RPGBuilder::useValidityInequality){
            sequentialActions(0,end);
            firstActions(0,end);
        }
        
        for (int e : toErase){
            solver->eliminateConstraint(e);
        }
        
        toErase.clear();
        goalStateConstraint();
        numericGoalConditions();
        if (MILPRPG::addLandmarks){
            addLandmarksConstraints(0,end);
            addNumericLandmarksConstraints(0,end);
        }
        if (RPGBuilder::useValidityInequality){
            lastLayer(end);
            forcePropositionsNotSelected(start,end);
            noopMutexAdd(start,end);
        }
    }
    // delete objective function
    solver->eliminateObjective();
    objectiveFunction(0, end);

}
