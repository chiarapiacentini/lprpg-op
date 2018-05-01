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

#include "GurobiIPStateBased.hpp"
#include "IPStateBased.hpp"
#include "CPLEXSolverInterface.hpp"
#include "NumericAnalysis.h"
#include "MILPRPG.h"
#include "VariableElimination.hpp"


GurobiIPStateBased::GurobiIPStateBased(MinimalState & state, int horizon, vector<bool> & aL, vector<bool> & fL, vector<bool> & aE, vector<bool> & fE, bool d) : GurobiSolverInterface(horizon,d), maxPossible(Planner::RPGBuilder::getPNECount()),minPossible(Planner::RPGBuilder::getPNECount()), mT(horizon), theState(state) {
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

void GurobiIPStateBased::initialiseVariables(){
    nSize = 1000000;
    x.x = new GRBVar[nSize];
    x.y = new GRBVar[nSize];
    x.v = new GRBVar[nSize];
    int countBool = 0;
    int countY = 0;
    int countV = 0;
    if (debug) cout << "initialising variables 1" << endl;
    int iTA = 0;
    // actions
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        double cost = RPGBuilder::usePlanLength ? 1 : RPGBuilder::getActionCost()[iA];
        if (cost == 0.0001) cost = 0;

        for (int t = 0; t < maxT; ++t){
            
            stringstream name;
            instantiatedOp *a = RPGBuilder::getInstantiatedOp(iA);
            name << *a << "_" << t;
            //cout << t << " "  << iA << " " << iTA << " " << getRow(iTA) << " " << getCol(iTA) << " " << getIndex(iA,t) << endl;
            if (t==0) indexAction.insert(make_pair(a, iA));
            if (t < maxT-1) x.x[countBool++] = model->addVar( 0, 1, cost, GRB_BINARY, name.str().c_str());
            else x.x[countBool++] = model->addVar(0, 0, cost, GRB_BINARY, name.str().c_str());
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
            x.y[countY++] = model->addVar( 0, 1,0, GRB_BINARY, name.str().c_str());
        }
    }
    // no-op literals
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        for (int t = 0; t < maxT; ++t){
            stringstream name;
            Literal *l = RPGBuilder::getLiteral(iL);
            name << *l << "_noopt_" << t;
            if (t==0) indexActionNoop.insert(make_pair(l, RPGBuilder::getNOp()+iL));
            if (t < maxT-1) x.x[countBool++] = model->addVar( 0, 1,0, GRB_BINARY, name.str().c_str());
            
            else x.x[countBool++] = model->addVar( 0, 0,0, GRB_BINARY, name.str().c_str());
            
        }
    }
    
    // count actions
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        for (int t = 0; t < maxT; ++t){
            
            stringstream name;
            instantiatedOp *a = RPGBuilder::getInstantiatedOp(iA);
            name << "m_" << *a << "_" << t;
            if (t==0) indexAction.insert(make_pair(a, iA));
            if (t < maxT-1) x.v[countV++] = model->addVar( 0, B,0, GRB_INTEGER, name.str().c_str());
            
            else x.v[countV++] = model->addVar( 0, 0,0, GRB_INTEGER, name.str().c_str());
            iTA++;
        }
    }
    // numeric condition
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        // uc
        for (int t = 0; t < maxT; ++t){
            
            stringstream name;
            name << "uc_" << i << "_" << t;
            x.y[countY++] = model->addVar( 0, 1,0, GRB_BINARY, name.str().c_str());                if (t==0) uc.insert(make_pair(i,  RPGBuilder::getNLiterals()+i));
        }
    }

}

void GurobiIPStateBased::buildConstraints(){
    initialStateConstraint();
    goalStateConstraint();
    objectiveFunction();
    actionPreconditions();
    actionsEffects();
    mutexRelations();
    noopPreconditions();
    noopMutex();
    numericActionPreconditions();
    numericPreconditions();
    numericGoalConditions();
    numericMutex();
    numericMutexPrecondition();
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

void GurobiIPStateBased::printSolution(){
    
    solution.str("");
    for (int t = 0; t < maxT; ++t){
        for (int a = 0; a < Planner::RPGBuilder::getNOp(); ++a){
            double isSelected = x.x[getIndex(a,t)].get(GRB_DoubleAttr_X);
            //cout << a << " " << t  << " " << isSelected << endl;
            if (isSelected>0){
                string name =x.x[getIndex(a,t)].get(GRB_StringAttr_VarName);
                solution << (t)/1000. << ": " << name.substr(0,name.find(")_")+1) << " [0.001]" << endl;
            }
        }
    }
}

void GurobiIPStateBased::updateConstraints(MinimalState &state, int limitH){
    for (int t = 0; t < maxT; ++t){
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            //
            if (t < limitH){
                x.x[getIndex(indexAction[RPGBuilder::getInstantiatedOp(iA)],t)].set(GRB_DoubleAttr_UB,1);
            }else{
                x.x[getIndex(indexAction[RPGBuilder::getInstantiatedOp(iA)],t)].set(GRB_DoubleAttr_UB,0);
            }
        }
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            Literal *l = RPGBuilder::getLiteral(iL);
            if (t < limitH){
                x.x[getIndex(indexActionNoop[l],t)].set(GRB_DoubleAttr_LB,0);
            }else{
                x.x[getIndex(indexActionNoop[l],t)].set(GRB_DoubleAttr_LB,0);
            }
        }
    }
}

void GurobiIPStateBased::initialStateConstraint(){
    if (debug) cout << "adding initial state" << endl;
    // get variables from rpg
    for (auto li : indexLiteral){
        Literal *l = li.first;
        int p = li.second;
        set<int> & propositions = theState.first;
        if (find(propositions.begin(),propositions.end(), p) != propositions.end())
            model->addConstr(x.y[getIndex(indexLiteral[l],0)] == 1);
        else
            model->addConstr(x.y[getIndex(indexLiteral[l],0)] == 0);
    }
}

void GurobiIPStateBased::goalStateConstraint(){
    if (debug) cout << "adding goalStateConstraint" << endl;
    for (auto l : RPGBuilder::getLiteralGoals()){
        model->addConstr(x.y[getIndex(indexLiteral[l], maxT - 1)] == 1);
    }
}

void GurobiIPStateBased::objectiveFunction(){
    if (debug) cout << "objectiveFunction" << endl;
    /*
    for (int t = 0; t < maxT; ++t){
        // actions
        for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
            double cost = RPGBuilder::usePlanLength ? 1 : RPGBuilder::getActionCost()[iA];
            if (cost == 0.0001) cost = 0;
            obj += x.x[getIndex(iA,t)] * cost;
        }
    }
    model.add(IloMinimize(env, obj));*/
}

// actionPreconditions
void GurobiIPStateBased::actionPreconditions(){
    if (debug) cout << "adding actionPreconditions" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        for (int t = 0; t < maxT; ++t){
            for (auto prec : RPGBuilder::getStartPropositionalPreconditions()[iA]){
                model->addConstr(x.x[getIndex(indexAction[RPGBuilder::getInstantiatedOp(iA)],t)] <= x.y[getIndex(indexLiteral[prec],t)]);
                
            }
        }
    }
}

// actionsEffects
void GurobiIPStateBased::actionsEffects(){
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
        for (int t = 0; t < maxT-1; ++t){
            GRBLinExpr toSum = 0;
            toSum += x.y[getIndex(indexLiteral[l],t+1)];
            for (auto a : idAddActions ){
                toSum += -1*x.x[getIndex(indexAction[RPGBuilder::getInstantiatedOp(a)],t)];
            }
            toSum += -1*x.x[getIndex(indexActionNoop[l],t)];
            model->addConstr(toSum <= 0);
        }
    }
}

// mutexRelations
void GurobiIPStateBased::mutexRelations(){
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
                for (int t = 0; t < maxT; ++t){
                    model->addConstr(x.x[getIndex(iA, t)] +  x.x[getIndex(jA, t)] <= 1);
                }
            }
        }
    }
    
}

void GurobiIPStateBased::noopPreconditions(){
    if (debug) cout << "adding actionPreconditions" << endl;
    for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
        Literal *l = RPGBuilder::getLiteral(iL);
        for (int t = 0; t < maxT - 1; ++t){
            model->addConstr(x.x[getIndex(indexActionNoop[l],t)] <= x.y[getIndex(indexLiteral[l],t)]);
        }
    }
}

void GurobiIPStateBased::noopMutex(){
    if (debug) cout << "adding noopMutex" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<Literal *> delList = RPGBuilder::getStartDeleteEffects()[iA];
        //cout << *RPGBuilder::getInstantiatedOp(iA) << endl;
        for (int iL = 0; iL < RPGBuilder::getNLiterals(); ++iL){
            Literal *l = RPGBuilder::getLiteral(iL);
            //cout << "\t" << *l << endl;
            if (find(delList.begin(),delList.end(),l) != delList.end()){
                //cout << "\t\tismutex" << endl;
                for (int t = 0; t < maxT - 1; ++t)
                    model->addConstr(x.x[getIndex(iA, t)] +  x.x[getIndex(indexActionNoop[l], t)] <= 1);
            }
        }
    }
}

void GurobiIPStateBased::numericActionPreconditions(){
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        list<int> preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[i];
        for (auto precID : preconditions){
            for (int t = 0; t < maxT; ++t){
                model->addConstr(x.y[getIndex(uc[precID], t)] >= x.x[getIndex(i,t)]);
            }
        }
    }
}

void GurobiIPStateBased::numericPreconditions(){
    if (debug) cout << "numericPreconditions" << endl;
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        for (int t = 0; t < maxT; ++t){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
            if (debug) cout << i << " " << prec << " " << prec.op << endl;
            pair<list<pair<int,double> >,double> precondition = FukunagaAnalysis::getExpression(prec);
            list<pair<int,double> > coefficient = precondition.first;
            double rhs = -precondition.second;
            if (debug) cout << "rhs" << rhs << endl;
            GRBLinExpr actionSum = 0;
            double constantCoefficient = rhs;
            bool toAdd = false;
            for (auto prec : coefficient){
                int v = prec.first;
                double w = prec.second;
                if (debug) cout << "\tconsidering fluent " << v << " of " << RPGBuilder::getPNECount() << endl;//" " << *RPGBuilder::getPNE(v) << endl;
                if ( fabs(w)< 0.000001) continue;
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
                                if ( fabs(weights[z]) >= 0.000001 && variables[z] != v) cout << "error: there is a non-simple numeric condition" << endl;
                                //if (variables[z] == v) constIncrease = weights[z];
                            }
                            if (!eff.isAssignment) constIncrease += eff.constant;
                        }
                    }
                    for (int iT = 0; iT < t; ++iT){
                        actionSum += w * constIncrease * x.x[getIndex(j,iT)];
                    }
                }
                //if (debug) cout << v << " : w " << w << " value " << tinitialFluents[v] << endl;
                constantCoefficient += w * theState.second[v];
            }
            if (toAdd){
                constantCoefficient += B;
                if (prec.op == VAL::E_GREATER){
                    //                    cout << "strictly disequality" << endl;
                    //                    if (constantCoefficient <= 0) constantCoefficient+=0.000001;
                    //                    else
                    constantCoefficient-=0.000001;
                }
                model->addConstr( actionSum + constantCoefficient - B * x.y[getIndex(uc[i],t)] >= 0);
            }
        }
    }
}

void GurobiIPStateBased::numericGoalConditions(){
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto goal : numericGoals){
        if (goal.first!=-1){
            model->addConstr(x.y[getIndex(uc[goal.first],maxT-1)] == 1);
        }
    }
}

void GurobiIPStateBased::numericMutex(){
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
                for (int t = 0; t < maxT; ++t){
                    model->addConstr(x.x[getIndex(iA, t)] +  x.x[getIndex(jA, t)] <= 1);
                }
            }
        }
    }
}

void GurobiIPStateBased::numericMutexPrecondition(){
    if (debug) cout << "adding numericMutex on preconditions" << endl;
    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> numEffI = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
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
                        int indexFluentJ = indexFluent.first;
                        if (indexFluentI == indexFluentJ ){
                            isMutex = true;
                            break;
                        }
                        if (isMutex) break;
                    }
                } // end numeric precondition
                if (isMutex) break; // no need to check other effects
            } // end numeric effect
            if (isMutex){
                if (debug) cout << "actions " << *RPGBuilder::getInstantiatedOp(iA) << " and " << *RPGBuilder::getInstantiatedOp(jA) << " are mutex"<<  endl;
                for (int t = 0; t < maxT; ++t){
                    model->addConstr(x.x[getIndex(iA, t)] +  x.x[getIndex(jA, t)] <= 1);
                }
            }
        }
    }
}

void GurobiIPStateBased::addLandmarksConstraints(){
    //bool debug = true;
    if (debug) cout << "adding addLandmarksConstraints" << endl;
    for (int i = 0; i < actionLandmarks.size(); ++i){
        if (actionLandmarks[i]){
            if (debug) cout << *RPGBuilder::getInstantiatedOp(i) << " is an action landmark" << endl;
            GRBLinExpr toSum = 0;
            instantiatedOp* action = RPGBuilder::getInstantiatedOp(i);
            for (int t = 0; t<maxT; ++t){
                toSum += x.x[getIndex(indexAction[action],t)];
            }
            model->addConstr( toSum >= 1);
        }
    }
    
    for (auto li : indexLiteral){
        Literal *l = li.first;
        int p = l->getStateID();
        if (factLandmarks[p]){
            if (debug ) cout << *l << " is a fact landmark" << endl;
            GRBLinExpr toSum = 0;
            for (int t = 0; t<maxT; ++t){
                toSum += x.y[getIndex(indexLiteral[l],t)];
            }
            model->addConstr(toSum >= 1);
        }
    }
    
    for (auto li : uc){
        int precID = li.first;
        int idL = precID + RPGBuilder::getNLiterals();
        if (factLandmarks[idL]){
            if (debug ) cout << idL << " is a numeric condition landmark" << endl;
            GRBLinExpr toSum = 0;
            for (int t = 0; t < maxT; ++t){
                toSum += x.y[getIndex(uc[precID], t)];
            }
            model->addConstr(toSum >= 1);
        }
    }
}

void GurobiIPStateBased::addRelevanceConstraints(){
    //bool debug = true;
    if (debug) cout << "adding eliminatedActions" << endl;
    for (int i = 0; i < eliminatedActions.size(); ++i){
        if (!eliminatedActions[i]){
            if (debug) cout << *RPGBuilder::getInstantiatedOp(i) << " can be eliminated" << endl;
            instantiatedOp* action = RPGBuilder::getInstantiatedOp(i);
            for (int t = 0; t<maxT; ++t){
                model->addConstr(x.x[getIndex(indexAction[action],t)] == 0);
            }
        }
    }
    
    for (auto li : indexLiteral){
        Literal *l = li.first;
        int p = l->getStateID();
        if (!eliminatedFacts[p]){
            if (debug ) cout << *l << " is a fact that can be eliminated" << endl;
            GRBLinExpr toSum;
            for (int t = 0; t<maxT; ++t){
                toSum += x.y[getIndex(indexLiteral[l],t)];
            }
            model->addConstr(toSum== 1);
        }
    }
    
    for (auto li : uc){
        int precID = li.first;
        int idL = precID + RPGBuilder::getNLiterals();
        if (!eliminatedFacts[idL]){
            if (debug ) cout << idL << " is a numeric condition  that can be eliminated" << endl;
            GRBLinExpr toSum;
            for (int t = 0; t < maxT; ++t){
                toSum += x.y[getIndex(uc[precID], t)];
            }
            model->addConstr(toSum== 1);
        }
    }
}

void GurobiIPStateBased::variablesBounds(){
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    
    // add variables bounds
    for (int i = 0; i < RPGBuilder::getPNECount(); ++i){
        GRBLinExpr toSum;
        
        //if (NumericAnalysis::getBounds().first==-DBL_MAX) boundMin = false;
        //if (NumericAnalysis::getBounds().second==DBL_MAX) boundMin = false;
        for (int j = 0; j < RPGBuilder::getNOp(); ++j){
            instantiatedOp *action = RPGBuilder::getInstantiatedOp(j);
            list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
            if (debug) cout << "\taction " << *RPGBuilder::getInstantiatedOp(j) << " has " << numEff.size() << " effects on numeric variables " << endl;
            double constIncrease = 0;
            for (int idEff : numEff){
                RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[idEff];
                int fluentIndex = eff.fluentIndex;
                if (fluentIndex == i){
                    if (debug) cout << "\t\t affected by " << *RPGBuilder::getInstantiatedOp(j) << endl;
                    vector<double> weights = eff.weights;
                    vector<int> variables = eff.variables;
                    for (int z = 0; z < variables.size(); ++z){
                        if ( fabs(weights[z]) >= 0.000001 && variables[z] != i) cout << "error: there is a non-simple numeric condition" << endl;
                        //if (variables[z] == v) constIncrease = weights[z];
                    }
                    if (!eff.isAssignment) constIncrease += eff.constant;
                }
            }
            for (int t = 0; t<maxT; ++t){
                toSum += constIncrease*x.x[getIndex(indexAction[action],t)];
            }
        }
        if (NumericAnalysis::getBounds()[i].first!=-DBL_MAX)
            model->addConstr(toSum + theState.second[i] >= NumericAnalysis::getBounds()[i].first);
        if (NumericAnalysis::getBounds()[i].second!=DBL_MAX)
            model->addConstr(toSum + theState.second[i] <= NumericAnalysis::getBounds()[i].second);
        
    }
}

void GurobiIPStateBased::sequentialActions(){
    if (debug) cout << "adding sequential action constraint" << endl;
    for (int t = 0; t < maxT; ++t){
        GRBLinExpr toSum = 0;
        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
            instantiatedOp *action = RPGBuilder::getInstantiatedOp(a);
            toSum += x.x[getIndex(indexAction[action],t)];
        }
        model->addConstr(toSum <= 1);
    }
}

void GurobiIPStateBased::firstActions(){
    if (debug) cout << "adding sequential action constraint" << endl;
    for (int t = 0; t < maxT - 1; ++t){
        GRBLinExpr toSum = 0;
        GRBLinExpr toSum2 = 0;
        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
            instantiatedOp *action = RPGBuilder::getInstantiatedOp(a);
            toSum += x.x[getIndex(indexAction[action],t)];
            toSum2+= x.x[getIndex(indexAction[action],t + 1)];
        }
        model->addConstr(toSum2 <= toSum);
    }
}

void GurobiIPStateBased::extractSolution(){
    for (int t = 0; t < maxT; ++t){
        for (int a = 0; a < Planner::RPGBuilder::getNOp(); ++a){
            double isSelected = x.x[getIndex(a,t)].get(GRB_DoubleAttr_X);
            if (isSelected>0){
                solutionIP.push_back(a);
            }
        }
    }
}



