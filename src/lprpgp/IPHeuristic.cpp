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
#include "IPHeuristic.hpp"
#include "IPModel.hpp"
#include "IPSolver.hpp"
#include "NumericAnalysis.h"
#include "MILPRPG.h"
#include "VariableElimination.hpp"
#include <queue>
#include <algorithm>
#include <iomanip>

using namespace Planner;


// constructor
IPHeuristic::IPHeuristic(MinimalState & state, bool d) : IPModel(d), theState(state), up(RPGBuilder::getNLiterals()), npmax(RPGBuilder::getNLiterals()), npmin(RPGBuilder::getNLiterals()), ua(RPGBuilder::getNOp()), eap(RPGBuilder::getNLiterals()*(RPGBuilder::getNOp()+1)), tp(RPGBuilder::getNLiterals()), ta(RPGBuilder::getNOp()), ip(RPGBuilder::getNLiterals()), iv(RPGBuilder::getPNECount()), mac((RPGBuilder::getNOp()+1)*RPGBuilder::getNumericPreTable().size()), ma(RPGBuilder::getNOp()), uc(RPGBuilder::getNumericPreTable().size()), eac((RPGBuilder::getNOp()+1)*RPGBuilder::getNumericPreTable().size()), tc(RPGBuilder::getNumericPreTable().size()), maxAction(RPGBuilder::getNOp()), firstAchievers(RPGBuilder::getNLiterals()), relevantActions(RPGBuilder::getNOp()), candidateLandmarks(RPGBuilder::getNLiterals() + RPGBuilder::getNumericPreTable().size()), candidateActionLandmarks(RPGBuilder::getNOp()), relevantPropositions(RPGBuilder::getNLiterals()), isInGoal(RPGBuilder::getNLiterals(),false), actionLandmarks(RPGBuilder::getNOp(),false),factLandmarks(RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size(),false),actionEliminated(RPGBuilder::getNOp(),true),factEliminated(RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size(),true), fAdd(RPGBuilder::getNOp(), vector<bool>(RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size(),false)) {
    B = 100000;
    maxUse = 0;
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        maxAction[i] = B;
        //relevantActions[i] = false;
        actionsToFilter.push_back(i); //TODO delete this
        actionsToUse.push_back(i); //TODO delete this
    }
    for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
        relevantPropositions[i] = true;
    }
    list<Literal*> & goalPropositions = RPGBuilder::getLiteralGoals();
    for(auto& it : goalPropositions ){
        int pID = (it)->getStateID();
        isInGoal[pID] = true;
    }
    
    
    {
        pAdd.assign(RPGBuilder::getNOp(), vector<bool>(RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size(),false));
        for (int i = 0; i < RPGBuilder::getNOp(); ++i){
            const list<Literal *> & addEff = RPGBuilder::getStartAddEffects()[i];
            for (auto l : addEff){
                pAdd[i][l->getStateID()] = true;
            }
        }
    }
    
    //calculateMaxActions(state);
    //    actionsToUse = vector<int>(maxUse);
    //    int j = 0;
    //    for (int i = 0; i < actionsToFilter.size(); ++i){
    //        if (maxAction[actionsToFilter[i]] > 0) actionsToUse[j++] = actionsToFilter[i];
    //    }
}

//initialize variables
void IPHeuristic::initialiseVariables(){
    int countBool = 0;
    int countInt = 0;
    int countFloat = 0;
    // literals
    for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
        // up
        {
            stringstream name;
            name << "up_" << i << "_" << *RPGBuilder::getLiteral(i);
            solver->addCol(IPBoolean, 0, 1, name.str());
            up[i] = countBool;
            countBool++;
        }
        // npmin
        {
            stringstream name;
            name << "npmin_" << i << "_" << *RPGBuilder::getLiteral(i);
            solver->addCol(IPInteger, -1, 1, name.str());
            npmin[i] = countInt;
            countInt++;
        }
        // npmax
        {
            stringstream name;
            name << "npmax_" << i << "_" << *RPGBuilder::getLiteral(i);
            solver->addCol(IPInteger, -1, 1, name.str());
            npmax[i] = countInt;
            countInt++;
        }
        // tp
        {
            stringstream name;
            name << "tp_" << i << "_" << *RPGBuilder::getLiteral(i);
            solver->addCol(IPInteger, 0, actionsToUse.size(), name.str());
            tp[i] = countInt;
            countInt++;
        }
        
        // ip
        {
            stringstream name;
            name << "ip_" << i << "_" << *RPGBuilder::getLiteral(i);
            solver->addCol(IPBoolean, 0, 1, name.str());
            ip[i] = countBool;
            countBool++;
        }
        
        // eap
        for (int ji = 0; ji < actionsToUse.size(); ++ji){
            int j = actionsToUse[ji];
            stringstream name;
            name << "eap_" << i << "_" << j << "_" << *RPGBuilder::getLiteral(i) << "_" << *RPGBuilder::getInstantiatedOp(j);
            list<Literal *> addList = RPGBuilder::getStartAddEffects()[j];
            Literal *l = RPGBuilder::getLiteral(i);
            if (find(addList.begin(),addList.end(), l) != addList.end()){
                solver->addCol(IPBoolean, 0, 1, name.str());
                eap[getIndex(i,j)] = countBool;
                countBool++;
            }
        }
    }
    
    // actions
    for (int j = 0; j < actionsToUse.size(); ++j){
        // ua
        int i = actionsToUse[j];
        {
            stringstream name;
            name << "ua_" << i << "_" << *RPGBuilder::getInstantiatedOp(i);
            solver->addCol(IPBoolean, 0, 1, name.str());
            ua[i] = countBool;
            countBool++;
        }
        // ta
        {
            stringstream name;
            name << "ta_" << i << "_" << *RPGBuilder::getInstantiatedOp(i);
            solver->addCol(IPInteger, 0, actionsToUse.size(), name.str());
            ta[i] = countInt;
            countInt++;
        }
        // ma
        {
            stringstream name;
            name << "ma_" << i << "_" << *RPGBuilder::getInstantiatedOp(i);
            solver->addCol(IPInteger, 0, B, name.str());
            ma[i] = countInt;
            countInt++;
        }
    }
    // numeric conditions
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        // uc
        {
            stringstream name;
            name << "uc_" << i;
            solver->addCol(IPBoolean, 0, 1, name.str());
            uc[i] = countBool;
            countBool++;
        }
        // tc
        {
            stringstream name;
            name << "tc_" << i;
            solver->addCol(IPInteger, 0, actionsToUse.size(), name.str());
            tc[i] = countInt;
            countInt++;
        }
        // eac
        for (int ji = 0; ji < actionsToUse.size(); ++ji){
            int j = actionsToUse[ji];
            stringstream name;
            name << "eac_" << i << "_" << j << *RPGBuilder::getInstantiatedOp(j);
            solver->addCol(IPBoolean, 0, 1, name.str());
            eac[getIndex(i,j)] = countBool;
            countBool++;
        }
        // mac
        for (int ji = 0; ji < actionsToUse.size(); ++ji){
            int j = actionsToUse[ji];
            stringstream name;
            name << "mac_" << i << "_" << j << *RPGBuilder::getInstantiatedOp(j);
            solver->addCol(IPInteger, 0, B, name.str());
            mac[getIndex(i,j)] = countInt;
            countInt++;
        }
    }
    
    // numeric variables
    for (int i = 0; i < RPGBuilder::getPNECount(); ++i){
        stringstream name;
        name << "iv_" << i;
        solver->addCol(IPContinous, -B, B, name.str());
        iv[i] = countFloat;
        countFloat++;
    }
}

void IPHeuristic::buildConstraints(){
    if (debug) cout << "build constraints fukunaga" << endl;
    objectiveValue();
    goalState();
    firstAchiever();
    actionEffects();
    
    if (!RPGBuilder::useExtraConstraints && !RPGBuilder::useSEQConstraints){
        //inverseAction();
        actionPreconditions();
    }else{
        inverseAction();
        if (RPGBuilder::useBoxing) variablesBounds();
        if (RPGBuilder::useSEQConstraints){
            propositionalSEQConstraints();
            if(!RPGBuilder::planAnyway) numericSEQGoalConstraints();
        }
    }
    if (!RPGBuilder::useTemporalRelaxation){
        timeConstraintPreconditions();
        timeConstraintEffects();
        if(!RPGBuilder::planAnyway) numericTimeConstraintActionPreconditions();
        if(!RPGBuilder::planAnyway) numericTimeConstraintEffects();
    }
    numericActionCounter();
    if(!RPGBuilder::planAnyway){
        numericPreconditions();
        numericGoalConditions();
        numericActionPreconditions();
        numericFirstAchiever();
    }
}

void IPHeuristic::objectiveValue(){
    
    if (debug) cout << "objectiveValue" << endl;
    list<IPSolver::IPConstraint> obj;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        double cost = RPGBuilder::usePlanLength ? 1 : RPGBuilder::getActionCost()[i];
        const double w = cost;
        obj.push_back({IPInteger,ma[i],w});
    }
    solver->addObjective(obj);
}

void IPHeuristic::goalState(){
    
    if (debug) cout << "goalState" << endl;
    list<Literal*> & goals = RPGBuilder::getLiteralGoals();
    for (int i = 0; i<RPGBuilder::getNLiterals(); ++i){
        Literal *l = RPGBuilder::getLiteral(i);
        if (find(goals.begin(),goals.end(), l) != goals.end()){
            solver->addRow(up[i],1,IPBoolean, IPEqual, 1);
        }
    }
}

void IPHeuristic::actionPreconditions(){
    
    if (debug) cout << "actionPreconditions" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        list<Literal*> & prec = RPGBuilder::getStartPropositionalPreconditions()[i];
        for (auto l : prec){
            int j = l->getStateID();
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPBoolean, up[j], 1});
            toAdd.push_back({IPBoolean, ua[i], -1});
            solver->addRow(toAdd, IPGreater,0);
        }
    }
}

void IPHeuristic::firstAchiever(){
    
    if (debug) cout << "firstAchiever" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        list<Literal *> addList = RPGBuilder::getStartAddEffects()[i];
        for (int j = 0; j < RPGBuilder::getNLiterals(); ++j){
            Literal *l = RPGBuilder::getLiteral(j);
            if (find(addList.begin(),addList.end(), l) != addList.end()){
                list<IPSolver::IPConstraint> toAdd;
                toAdd.push_back({IPBoolean, ua[i], 1});
                toAdd.push_back({IPBoolean, eap[getIndex(j, i)], -1});
                solver->addRow(toAdd, IPGreater,0);
            }
        }
    }
}

void IPHeuristic::actionEffects(){
    
    if (debug) cout << "actionEffects" << endl;
    for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
        list<IPSolver::IPConstraint> toAdd;
        for (int ji = 0; ji < actionsToUse.size(); ++ji){
            int j = actionsToUse[ji];
            list<Literal *> addList = RPGBuilder::getStartAddEffects()[j];
            Literal *l = RPGBuilder::getLiteral(i);
            if (find(addList.begin(),addList.end(), l) != addList.end() && relevantPropositions[i]){
                toAdd.push_back({IPBoolean, eap[getIndex(i,j)], 1});
            }
            
        }
        toAdd.push_back({IPBoolean, ip[i], 1});
        toAdd.push_back({IPBoolean, up[i], -1});
        solver->addRow(toAdd, IPEqual,0);
    }
}

void IPHeuristic::propositionalSEQConstraints(){
    
    if (debug) cout << "propositionalSEQConstraints" << endl;
    for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
        list<IPSolver::IPConstraint> add;
        list<IPSolver::IPConstraint> del;
        list<IPSolver::IPConstraint> predel;
        Literal *l = RPGBuilder::getLiteral(i);
        const list<pair<int, VAL::time_spec> > & adding =  RPGBuilder::getEffectsToActions()[i];
        const list<pair<int, VAL::time_spec> > & deleting =  RPGBuilder::getNegativeEffectsToActions()[i];
        for(auto& act: adding) {
            int id = act.first;
            predel.push_back({IPInteger,ma[id],-1});
        }
        for(auto& act: deleting) {
            int id = act.first;
            del.push_back({IPInteger,ma[id],1});
            // check if it's a predel action
            list<Literal*> & prec = RPGBuilder::getStartPropositionalPreconditions()[id];
            if (find(prec.begin(),prec.end(),l)!= prec.end()){
                predel.push_back({IPInteger,ma[id],1});
            }
            
        }
        // is this correct? because it seems not to be true:
        // e.g: action that is A -> add a, b; B -> add c, del a
        // C -> add d, del a
        // goal d
        // plan A B C -> min a = 0, max a = 1
        //model.add( IloSum(add) - IloSum(del) >= x.m[npmin[i]]);
        //model.add( IloSum(add) - IloSum(del) <= x.m[npmax[i]]);
        
        // check if it's a goal condition
        int isGoal = 0;
        const list<Literal*> & goals = RPGBuilder::getLiteralGoals();
        if (find(goals.begin(),goals.end(),l)!=goals.end()) isGoal = -1;
        predel.push_back({IPBoolean,ip[i],-1});
        const int w = isGoal;
        solver->addRow(predel, IPLess, w);
    }
}

void IPHeuristic::timeConstraintPreconditions(){
    
    if (debug) cout << "timeConstraintPreconditions" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        list<Literal*> prec = RPGBuilder::getStartPropositionalPreconditions()[i];
        for (int j = 0; j < RPGBuilder::getNLiterals(); ++j){
            Literal *l = RPGBuilder::getLiteral(j);
            if (find(prec.begin(),prec.end(), l) != prec.end()){
                list<IPSolver::IPConstraint> toAdd;
                toAdd.push_back({IPInteger, tp[j], 1});
                toAdd.push_back({IPInteger, ta[i], -1});
                solver->addRow(toAdd, IPLess,0);
            }
        }
    }
}

void IPHeuristic::timeConstraintEffects(){
    
    if (debug) cout << "timeConstraintEffects" << endl;
    for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
        for (int ji = 0; ji < actionsToUse.size(); ++ji){
            int j = actionsToUse[ji];
            list<Literal *> addList = RPGBuilder::getStartAddEffects()[j];
            Literal *l = RPGBuilder::getLiteral(i);
            if (find(addList.begin(),addList.end(), l) != addList.end() && relevantPropositions[i]){
                int maxTime = actionsToUse.size() + 1;
                list<IPSolver::IPConstraint> toAdd;
                toAdd.push_back({IPInteger, ta[j], 1});
                toAdd.push_back({IPInteger, tp[i], -1});
                const double w = maxTime;
                toAdd.push_back({IPBoolean, eap[getIndex(i,j)], w});
                solver->addRow(toAdd, IPLess,maxTime-1);
            }
        }
    }
}

void IPHeuristic::numericPreconditions(){
    
    
    if (debug) cout << "numericPreconditions" << endl;
    //bool debug = true;
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
        if (debug) cout << "numericCondition " << i << " " << prec << endl;
        pair<list<pair<int,double>>,double> precondition = getExpression(prec);
        list<pair<int,double>> coefficient = precondition.first;
        double rhs = -precondition.second;
        if (debug) cout << "rhs " << rhs << endl;
        list<IPSolver::IPConstraint> actionSum;
        double constantCoefficient = rhs;
        bool toAdd = false;
        double upperU = rhs;
        for (auto prec : coefficient){
            int v = prec.first;
            double w = prec.second;
            if (debug) cout << "\tconsidering fluent " << v << " of " << RPGBuilder::getPNECount() << endl;//" " << *RPGBuilder::getPNE(v) << endl;
            if ( fabs(w)< 0.0001) continue;
            toAdd = true;
            for (int ji = 0; ji < actionsToUse.size(); ++ji){
                int j = actionsToUse[ji];
                list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
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
                            if ( fabs(weights[z]) >= 0.0001 && variables[z] != v) cout << "error: there is a non-simple numeric condition" << endl;
                            //if (variables[z] == v) constIncrease = weights[z];
                        }
                        if (!eff.isAssignment) constIncrease += eff.constant;
                    }
                }
                //cout << "action " << *RPGBuilder::getInstantiatedOp(j)  <<  " affects variable " << constIncrease << " " << w << " " << x.x[uc[i]].getName() << endl;
                if (!RPGBuilder::useDeleteRelaxation || w*constIncrease>=0)
                    actionSum.push_back({IPInteger, mac[getIndex(i,j)], w * constIncrease});
            }
            actionSum.push_back({IPContinous, v, w});
            if (w<0){
                double upper = NumericAnalysis::getBounds()[v].second;
                if (debug) cout << "upper (positive) bound on " << v << " " << upper << " " << w << " (" << NumericAnalysis::getBounds()[v].first << "," << NumericAnalysis::getBounds()[v].second << ")" << endl;
                //if (upper!=DBL_MAX && upperU<=B)
                upperU += w*upper;
                //else upperU = B;
            }else{
                double upper = NumericAnalysis::getBounds()[v].first;
                if (debug) cout << "upper (negative) bound on " << v << " " << upper << " " << w << " (" << NumericAnalysis::getBounds()[v].first << "," << NumericAnalysis::getBounds()[v].second << ")" << endl;
                //if (upper!=-DBL_MAX && upperU>=-B)
                upperU += w*upper;
                //else upperU = -B;
            }
            if (upperU>B)
                upperU = B;
            if (upperU<-B) upperU = -B;
        }
        //cout << "total bound " << upperU << endl;
        if (toAdd){
            constantCoefficient -= upperU;
            if (prec.op == VAL::E_GREATER){
                //cout << "strictly disequality" << endl;
                constantCoefficient-=EPSILON;
                //if (constantCoefficient <= 0) constantCoefficient+=EPSILON;
                //else constantCoefficient-=EPSILON;
            }
            actionSum.push_back({IPBoolean, uc[i], upperU});
            solver->addRow(actionSum, IPGreater,-constantCoefficient);
        }
    }
}

void IPHeuristic::inverseAction(){
    
    map<int,list<int>> inverseActions;
    FukunagaAnalysis::getInverseActions(inverseActions);
    if (debug) cout << "Found " << inverseActions.size() << " pairs of inverse actions over " << RPGBuilder::getNOp() << " from which " << actionsToUse.size() << " are used " << endl;
    
    
    
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        list<Literal*> prec = RPGBuilder::getStartPropositionalPreconditions()[i];
        for (auto l : prec){
            int j = l->getStateID();
            list<IPSolver::IPConstraint> toSum;
            for (auto z : inverseActions[i]){
                list<Literal *> addList = RPGBuilder::getStartAddEffects()[z];
                Literal *l = RPGBuilder::getLiteral(j);
                if (find(addList.begin(),addList.end(), l) != addList.end()){
                    toSum.push_back({IPBoolean,eap[getIndex(j,z)],-1});
                }
            }
            toSum.push_back({IPBoolean,up[j],1});
            toSum.push_back({IPBoolean,ua[i],-1});
            solver->addRow(toSum, IPGreater,0);
        }
    }
}

void IPHeuristic::numericGoalConditions(){
    if(debug) cout << "numeric goal condition" << endl;
    list<pair<int, int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto goal : numericGoals){
        if (goal.first!=-1){
            if (debug) cout << "goal " << goal.first << "/" << uc.size() << endl;
            solver->addRow(uc[goal.first],1, IPBoolean, IPEqual,1);
        }
    }
}

void IPHeuristic::numericActionPreconditions(){
    if (debug) cout << "action to use " << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        const list<int> & preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[i];
        for (auto precID : preconditions){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPBoolean, ua[i], 1});
            toAdd.push_back({IPBoolean, uc[precID], -1});
            solver->addRow(toAdd, IPLess,0);
        }
    }
}
void IPHeuristic::numericFirstAchiever(){
    if (debug) cout << "numericFirstAchiever" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        for (int j = 0; j < RPGBuilder::getNumericPreTable().size(); ++j){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPInteger, mac[getIndex(j,i)], 1});
            const double w = -B;
            toAdd.push_back({IPBoolean, eac[getIndex(j,i)], w});
            solver->addRow(toAdd, IPLess,0);
        }
    }
}
void IPHeuristic::numericActionCounter(){
    if (debug) cout << "numericActionCounter" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        
        list<IPSolver::IPConstraint> toAdd3;
        toAdd3.push_back({IPInteger, ma[i], 1});
        toAdd3.push_back({IPBoolean, ua[i] , -1});
        solver->addRow(toAdd3, IPGreater,0);
        
        for (int j = 0; j < RPGBuilder::getNumericPreTable().size(); ++j){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPInteger, mac[getIndex(j,i)], 1});
            toAdd.push_back({IPInteger, ma[i] , -1});
            solver->addRow(toAdd, IPLess,0);
            
            list<IPSolver::IPConstraint> toAdd2;
            toAdd2.push_back({IPBoolean, eac[getIndex(j,i)], 1});
            toAdd2.push_back({IPBoolean, ua[i] , -1});
            solver->addRow(toAdd2, IPLess,0);
        }
    }
}
void IPHeuristic::numericTimeConstraintActionPreconditions(){
    if (debug) cout << "numericTimeConstraintActionPreconditions" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        list<int> preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[i];
        for (auto precID : preconditions){
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPInteger, tc[precID], 1});
            toAdd.push_back({IPInteger, ta[i] , -1});
            solver->addRow(toAdd, IPLess,0);
        }
    }
}

void IPHeuristic::numericTimeConstraintEffects(){
    if (debug) cout << "numericTimeConstraintEffects" << endl;
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        for (int j = 0; j < RPGBuilder::getNumericPreTable().size(); ++j){
            int maxTime = actionsToUse.size() + 1;
            list<IPSolver::IPConstraint> toAdd;
            toAdd.push_back({IPInteger, ta[i], 1});
            toAdd.push_back({IPInteger, tc[j], -1});
            const double w = maxTime;
            toAdd.push_back({IPBoolean, eac[getIndex(j,i)], w});
            solver->addRow(toAdd, IPLess,maxTime-1);
        }
    }
}

void IPHeuristic::numericSEQGoalConstraints(){
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto goal : numericGoals){
        if (goal.first!=-1){
            int i = goal.first;
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
            if (debug) cout << i << " " << prec << endl;
            pair<list<pair<int,double>>,double> precondition = getExpression(prec);
            list<pair<int,double>> coefficient = precondition.first;
            double rhs = -precondition.second;
            if (debug) cout << "rhs" << rhs << endl;
            list<IPSolver::IPConstraint> actionSum;
            double constantCoefficient = rhs;
            bool toAdd = false;
            for (auto prec : coefficient){
                int v = prec.first;
                double w = prec.second;
                if (debug) cout << "\tconsidering fluent " << v << " of " << RPGBuilder::getPNECount() << endl;//" " << *RPGBuilder::getPNE(v) << endl;
                if ( fabs(w)< 0.0001) continue;
                toAdd = true;
                for (int ji = 0; ji < actionsToUse.size(); ++ji){
                    int j = actionsToUse[ji];
                    list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
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
                                if ( fabs(weights[z]) >= 0.0001 && variables[z] != v) cout << "error: there is a non-simple numeric condition" << endl;
                                //if (variables[z] == v) constIncrease = weights[z];
                            }
                            if (!eff.isAssignment) constIncrease += eff.constant;
                        }
                    }
                    //cout << "action " << *RPGBuilder::getInstantiatedOp(j)  <<  " affects variable " << constIncrease << " " << w << " " << x.x[uc[i]].getName() << endl;
                    actionSum.push_back({IPInteger,ma[j],w*constIncrease});
                }
                if (debug) cout << v << " : w " << w << " value " << theState.second[v] << endl;
                actionSum.push_back({IPContinous,v,w});
            }
            if (toAdd){
                if (prec.op == VAL::E_GREATER){
                    cout << "strictly disequality" << endl;
                    //if (constantCoefficient <= 0) constantCoefficient+=EPSILON;
                    //else
                    constantCoefficient-=EPSILON;
                }
                solver->addRow(actionSum,IPGreater,-constantCoefficient);
            }
            
        }
    }
}

void IPHeuristic::variablesBounds(){
    list<pair<int, int> > numericGoals = RPGBuilder::getNumericRPGGoals();
    
    // add variables bounds
    for (int i = 0; i < RPGBuilder::getPNECount(); ++i){
        list<IPSolver::IPConstraint> toSum;

        //if (NumericAnalysis::getBounds().first==-DBL_MAX) boundMin = false;
        //if (NumericAnalysis::getBounds().second==DBL_MAX) boundMin = false;
        for (int ji = 0; ji < actionsToUse.size(); ++ji){
            int j = actionsToUse[ji];
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
                        if ( fabs(weights[z]) >= 0.0001 && variables[z] != i) cout << "error: there is a non-simple numeric condition" << endl;
                        //if (variables[z] == v) constIncrease = weights[z];
                    }
                    if (!eff.isAssignment) constIncrease += eff.constant;
                }
            }
            toSum.push_back({IPInteger,ma[j],constIncrease});
        }
        toSum.push_back({IPContinous,i,1});
        if (NumericAnalysis::getBounds()[i].first!=-DBL_MAX) solver->addRow(toSum, IPGreater, NumericAnalysis::getBounds()[i].first);
        if (NumericAnalysis::getBounds()[i].second!=DBL_MAX) solver->addRow(toSum,IPLess, NumericAnalysis::getBounds()[i].second);
        
    }
}

pair<list<pair<int,double>>,double> IPHeuristic::getExpression(Planner::RPGBuilder::RPGNumericPrecondition precondition){
    int nVar = RPGBuilder::getPNECount();
    list<pair<int, double>> toReturn;
    int lhs = precondition.LHSVariable;
    double rhs = precondition.RHSConstant;
    if (lhs < nVar){
        // x > 0
        toReturn.push_back({lhs,1});
    }else if (lhs < 2*nVar){
        // x < 0
        toReturn.push_back({lhs-nVar,-1});
    }else{
        // get artificial variable
        RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(lhs);
        vector<double> weights = av.weights;
        vector<int> fluents = av.fluents;
        double constant = av.constant;
        rhs -= constant;
        for (int i = 0; i<weights.size(); ++i){
            if (fluents[i] < nVar )
                toReturn.push_back({fluents[i],weights[i]});
            else
                toReturn.push_back({fluents[i] - nVar, -weights[i]});
        }
    }
    //    for ( auto i : toReturn){
    //        cout << i.first << " " << i.second << endl;
    //    }
    return {toReturn,rhs};
}

void IPHeuristic::extractSolution(){
//    if(debug){
//        cout << "\tRelaxed Plan Fast " << solver->getObjectiveValue() << endl;
//        
//        for (int ji = 0; ji < actionsToUse.size(); ++ji){
//            int i = actionsToUse[ji];
//            double isSelected = solver->getValue(IPInteger,ma[i]);
//            if (isSelected > 0.001){
//                cout << setprecision(6) << "\t" << solver->getValue(IPInteger,ta[i]) << ": " << *RPGBuilder::getInstantiatedOp(i) << " [ *" << isSelected << "]" << endl;
//            }
//        }
//    }
}

int IPHeuristic::extractSolutionLength(){
    int solutionLength = 0;
    vector<list<int> > mapLength(actionsToUse.size() + 1, list<int>());
    for (int ji = 0; ji < actionsToUse.size(); ++ji){
        int i = actionsToUse[ji];
        double isSelected = solver->getValue(IPInteger,ma[i]);
        if (isSelected > 0.001){
            (mapLength[int(solver->getValue(IPInteger,ta[i]))]).push_back(solver->getValue(IPInteger,ma[i]));
        }
    }
    for (auto& e : mapLength){
        solutionLength+= *max_element(e.begin(),e.end());
    }
    return solutionLength;
}

void IPHeuristic::calculateMaxActions(MinimalState &theState){
    if (debug) cout << "calculate upper bounds on actions" << endl;
    bool debug = false;
    for (int ji = 0; ji < actionsToFilter.size(); ++ji){
        int j = actionsToFilter[ji];
        int maxNeeded = 0;
        if (RPGBuilder::getStartAddEffects()[j].size()>0) maxNeeded = 1;
        list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
        if (debug) cout << "\taction " << *RPGBuilder::getInstantiatedOp(j) << " has " << numEff.size() << " effects on numeric variables " << endl;
        for (int idEff : numEff){
            RPGBuilder::RPGNumericEffect & eff = RPGBuilder::getNumericEff()[idEff];
            int fluentIndex = eff.fluentIndex;
            vector<double> & weights = eff.weights;
            vector<int> & variables = eff.variables;
            double constIncrease = 0;
            for (int z = 0; z < variables.size(); ++z){
                if ( fabs(weights[z]) >= 0.0001 && variables[z] != fluentIndex) cout << "error: there is a non-simple numeric condition" << endl;
                if (variables[z] == fluentIndex) constIncrease = weights[z];
            }
            if (!eff.isAssignment) constIncrease += eff.constant;
            double maxEff = constIncrease > 0 ? NumericAnalysis::getMinMaxNeeded()[fluentIndex].second : NumericAnalysis::getMinMaxNeeded()[fluentIndex].first;
            if ( (maxEff == DBL_MAX  && constIncrease > 0) || (maxEff == -DBL_MAX && constIncrease < 0)) {
                maxNeeded = B;
                break;
            }
            int howManyTime = ceil((maxEff - theState.second[fluentIndex])/constIncrease);
            if (debug) cout << "\t\t" << "maxEff: " << maxEff << ", how many times: " << howManyTime << endl;
            if (howManyTime < 0) howManyTime = 0;
            if (howManyTime > maxNeeded){
                maxNeeded = howManyTime;
            }
        }
        maxAction[j] = maxNeeded;
        if (maxNeeded > 0) maxUse++;
        if (debug) cout << *RPGBuilder::getInstantiatedOp(j) << " " << maxAction[j] << endl;
    }
}

void IPHeuristic::updateConstraints(MinimalState &state) {
    if(debug) cout << "updateConstraints" << endl;
    int nConditions = RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size() ;
    set<int> & propositions = state.first;
    
    if (RPGBuilder::useExtraConstraints || MILPRPG::addLandmarks){
        fill(actionLandmarks.begin(),actionLandmarks.end(),false);
        fill(factLandmarks.begin(),factLandmarks.end(),false);
        fill(actionEliminated.begin(),actionEliminated.end(),true);
        fill(factEliminated.begin(), factEliminated.end(),true);
        fill(fAdd.begin(),fAdd.end(), vector<bool>(nConditions,false));
        
        
        FukunagaAnalysis::iterativeVariableElimination(state, actionLandmarks, factLandmarks, actionEliminated, factEliminated, fAdd, debug);
    }
    for (int p = 0; p < RPGBuilder::getNLiterals(); ++p) {
        
        // initial state
        if (find(propositions.begin(),propositions.end(), p) != propositions.end()) {
            solver->updateCol(ip[p],IPBoolean,LB,1);
            solver->updateCol(ip[p],IPBoolean,UB,1);
        }
        else{
            solver->updateCol(ip[p],IPBoolean,LB,0);
            solver->updateCol(ip[p],IPBoolean,UB,0);
        }
        // landmarks
        if (factLandmarks[p]){
            solver->updateCol(up[p],IPBoolean,LB,1);
        }else{
            solver->updateCol(up[p],IPBoolean,LB,0);
        }
        if (!factEliminated[p]) {
            solver->updateCol(up[p],IPBoolean,UB,0);
        }else{
            solver->updateCol(up[p],IPBoolean,UB,1);
        }
        if (RPGBuilder::useExtraConstraints && MILPRPG::addLandmarks){
            for (int a = 0; a < RPGBuilder::getNOp(); ++a){
                if (pAdd[a][p]){
                    if (fAdd[a][p]) solver->updateCol(eap[getIndex(p,a)],IPBoolean,UB,1);
                    else solver->updateCol(eap[getIndex(p,a)],IPBoolean,UB,0);
                }
            }
        }
    }
    
    //    if (RPGBuilder::getPNECount() > 0 && !RPGBuilder::useSEQConstraints){
    //        calculateMaxActions(state);
    //    }
    // initial state numeric
    for (int v = 0; v < RPGBuilder::getPNECount(); ++v){
        solver->updateCol(iv[v],IPContinous,LB,state.second[v]);
        solver->updateCol(iv[v],IPContinous,UB,state.second[v]);
    }
    
    for (int a = 0; a < RPGBuilder::getNOp(); ++a){
        //        if (RPGBuilder::getPNECount() > 0){
        //            x.m[ma[a]].setUB(maxAction[a]);
        //        }
        if (actionLandmarks[a]){
            solver->updateCol(ma[a],IPInteger,LB,1);
        }else{
            solver->updateCol(ma[a],IPInteger,LB,0);
        }
        if (!actionEliminated[a]) {
            solver->updateCol(ua[a],IPBoolean,UB,0);
        }else{
            solver->updateCol(ua[a],IPBoolean,UB,1);
        }
    }
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        int c = i + RPGBuilder::getNLiterals();
        if (factLandmarks[c]){
            solver->updateCol(uc[i],IPBoolean,LB,1);
        }else{
            solver->updateCol(uc[i],IPBoolean,LB,0);
        }
        if (!factEliminated[c]){
            solver->updateCol(uc[i],IPBoolean,UB,0);
        }else{
            solver->updateCol(uc[i],IPBoolean,UB,1);
        }
    }
    
}


