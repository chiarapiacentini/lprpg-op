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


#include "VariableElimination.hpp"
#include "NumericAnalysis.h"
#include "MILPRPG.h"
#include <algorithm>

#define MAXN 1000000

/////// landmarks
void FukunagaAnalysis::performLandmarkAnalysis(set<int> & actions, MinimalState &state, bool includeNumeric, vector<vector<bool>> & factLandmarks, bool debug){
    
    if (debug) cout << " --- landmark analysis ---" << endl;
    
    int nConditions = includeNumeric ? RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size() : RPGBuilder::getNLiterals();
    factLandmarks.assign(nConditions,vector<bool>(nConditions,false));
    vector<bool> addedConditions(nConditions);
    LandmarkState s = {state.first, state.second, state.second};
    for (int i = 0; i< nConditions; ++i){
        if(checkSatisfied(i,state.first, state.second, state.second)){
            factLandmarks[i][i] = true;
            addedConditions[i] = true;
        }else{
            factLandmarks[i].assign(nConditions,true);
            addedConditions[i] = false;
        }
    }
    
    if (debug) {
        cout << "initial candidates " << endl;
        for (int i = 0; i < factLandmarks.size(); ++i){
            cout << "proposition ";
            if (i < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(i) << " : ";
            else cout << RPGBuilder::getNumericPreTable()[i - RPGBuilder::getNLiterals()] << " : ";
            for (int l = 0; l < factLandmarks[i].size(); ++l){
                if (l == -1) cout << " everything ";
                else if (!factLandmarks[i][l]) continue;
                else if (l < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(l) << "  ";
                else cout << l << " " ;
            }
            cout << endl;
        }
    }
    
    queue<int> actionsQueue;
    vector<bool> visited(RPGBuilder::getNOp(),false);
    for (auto i : actions){
        
        if (checkActionSatisfied(i,state.first, state.second, state.second)){
            actionsQueue.push(i);
            visited[i]=true;
            if (debug) cout << " action satisfied in initial state " << *RPGBuilder::getInstantiatedOp(i) << endl;
        }
    }
    
    while(!actionsQueue.empty()){
        int a = actionsQueue.front();
        actionsQueue.pop();
        visited[a] = false;
        if (debug) cout << "\tconsidering action " << *RPGBuilder::getInstantiatedOp(a)  << " (not " << actionsQueue.size() << " in queue)" << endl;
        
        const list<Literal *> & addEff = RPGBuilder::getStartAddEffects()[a];
        
        //LandmarkState tmpState = {state.first, state.second, state.second};
        list<int> addConditions;
        for (auto p : addEff){
            int pID = p->getStateID();
            //updateState(pID,tmpState,false);
            updateState(pID,s,false);
            addConditions.push_back(pID);
        }
        
        list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[a];
        
        if (includeNumeric){
            for (int idEff : numEff){
                //updateState(idEff,tmpState,true);
                updateState(idEff,s,true);
            }
            
            for (int j = RPGBuilder::getNLiterals(); j < RPGBuilder::getNLiterals() + RPGBuilder::getNumericPreTable().size(); ++j){
                if (/*!addedConditions[j] &&*/ checkSatisfied(j,s)) {
                    addConditions.push_back(j); //TODO delete?
                    addedConditions[j] = true;
                    if (debug) cout << "\tcondition " << j << " added" << endl;
                }
            }
        }
        
//        for (auto & p : addEff){
//            int pID = p->getStateID();
//            addConditionLandmark(pID, a, s, factLandmarks,actionsQueue,visited,false,addConditions, actions);
//        }
//        
//        if (includeNumeric){
//            for (int idEff : numEff){
//                addConditionLandmark(idEff, a, s, factLandmarks,actionsQueue,visited,true,addConditions, actions);
//            }
//        }
        
        for( auto ad : addConditions){
            if (ad <  RPGBuilder::getNLiterals())
                addConditionLandmark(ad, a, s, factLandmarks,actionsQueue,visited,false,addConditions, actions);
            else
                addConditionLandmark(ad, a, s, factLandmarks,actionsQueue,visited,true,addConditions, actions);

        }
        
        if (debug) {
            cout << " ++++ " << endl;
            for (int i = 0; i < factLandmarks.size(); ++i){
                cout << "proposition ";
                if (i < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(i) << " : ";
                else cout << RPGBuilder::getNumericPreTable()[i - RPGBuilder::getNLiterals()] << " : ";
                for (int l = 0; l < factLandmarks[i].size(); ++l){
                    if ( l == -1) cout << " everything";
                    else if (!factLandmarks[i][l]) continue;
                    else if (l < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(l) << "  ";
                    else cout << l << " " ;//RPGBuilder::getNumericPreTable()[l - RPGBuilder::getNLiterals()] << " ";
                }
                cout << endl;
            }
            cout << " ++++ " << endl;
        }
        
    }
    if (debug) {
        cout << " --- fact landmarks --- " << endl;
        for (int i = 0; i < factLandmarks.size(); ++i){
            cout << "proposition ";
            if (i < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(i) << " : ";
            else cout << RPGBuilder::getNumericPreTable()[i - RPGBuilder::getNLiterals()] << " : ";
            for (int l = 0; l < factLandmarks[i].size(); ++l){
                if ( l == -1) cout << " everything";
                else if (!factLandmarks[i][l]) continue;
                else if (l < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(l) << "  ";
                else cout << l << " " ;//RPGBuilder::getNumericPreTable()[l - RPGBuilder::getNLiterals()] << " ";
            }
            cout << endl;
        }
        cout << " --- " << endl;
    }
}

pair<list<pair<int,double>>,double> FukunagaAnalysis::getExpression(Planner::RPGBuilder::RPGNumericPrecondition precondition){
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
    return {toReturn,rhs};
}

bool FukunagaAnalysis::checkSatisfied(int condition, LandmarkState &state){
    if (condition < RPGBuilder::getNLiterals()){
        return (state.propositions.find(condition) != state.propositions.end());
    }else{
        int i = condition - RPGBuilder::getNLiterals();
        RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
        pair<list<pair<int,double>>,double> precondition = getExpression(prec);
        double expression = 0;
        for (auto add : precondition.first){
            int v = add.first;
            int w = add.second;
            if (w>0) expression += w * state.maxV[v];
            else expression += w * state.minV[v];
        }
        if (prec.op == VAL::E_GREATER){
            return expression > precondition.second;
        }else{
            return expression >= precondition.second;
        }
    }
    return false;
}

bool FukunagaAnalysis::checkSatisfied(int condition, set<int> &prop, vector<double> &minV, vector<double> &maxV){
    if (condition < RPGBuilder::getNLiterals()){
        return (prop.find(condition) != prop.end());
    }else{
        int i = condition - RPGBuilder::getNLiterals();
        RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
        pair<list<pair<int,double>>,double> precondition = getExpression(prec);
        double expression = 0;
        for (auto add : precondition.first){
            int v = add.first;
            int w = add.second;
            if (w>0) expression += w * maxV[v];
            else expression += w * minV[v];
        }
        if (prec.op == VAL::E_GREATER){
            return expression > precondition.second;
        }else{
            return expression >= precondition.second;
        }
    }
    return false;
}

bool FukunagaAnalysis::checkActionSatisfied(int action, LandmarkState &state){
    if (isSubsetOfI(RPGBuilder::getStartPropositionalPreconditions()[action], state.propositions)){
        bool numericStatisfied = true;
        const list<int> & preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[action];
        for (auto c : preconditions){
            if (!checkSatisfied(c + RPGBuilder::getNLiterals() , state)){
                numericStatisfied = false;
                break;
            }
        }
        return numericStatisfied;
    }
    return false;
}

bool FukunagaAnalysis::checkActionSatisfied(int action, set<int> &prop, vector<double> &minV, vector<double> &maxV){
    if (isSubsetOfI(RPGBuilder::getStartPropositionalPreconditions()[action], prop)){
        bool numericStatisfied = true;
        const list<int> & preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[action];
        for (auto c : preconditions){
            if (!checkSatisfied(c + RPGBuilder::getNLiterals() , prop, minV, maxV)){
                numericStatisfied = false;
                break;
            }
        }
        return numericStatisfied;
    }
    return false;
}

bool FukunagaAnalysis::isSubsetOfI(const list<Literal *>& addEff, list<int> &numericAdd, vector<bool>& toCheck, vector<bool>& superSet){
    
    for (auto & a : addEff){
        int l = a->getStateID();
        if (!toCheck[l]) continue;
        if (!superSet[l]) return false;
    }
    for (auto & l : numericAdd){
        if (!toCheck[l]) continue;
        if (!superSet[l]) return false;
    }
    return true;
}
bool FukunagaAnalysis::isSubsetOfI(const list<Literal *>& toCheck, list<Literal *>& superSet){
    
    if (toCheck.size() > superSet.size()) return false;
    for (auto & l : toCheck){
        if (find(superSet.begin(),superSet.end(),l) == superSet.end()) return false;
    }
    return true;
}

bool FukunagaAnalysis::isSubsetOfI(list<Literal *>& toCheck, set<int>& superSet){
    
    if (toCheck.size() > superSet.size()) return false;
    for (auto & l : toCheck){
        if (superSet.find(l->getStateID()) == superSet.end()) return false;
    }
    return true;
}

bool FukunagaAnalysis::isSubsetOfI(list<int>& toCheck, set<int>& superSet){
    
    if (toCheck.size() > superSet.size()) return false;
    for (auto & l : toCheck){
        if (superSet.find(l) == superSet.end()) return false;
    }
    return true;
}

bool FukunagaAnalysis::isSubsetOfI(set<int>& toCheck, set<int>& superSet){
    
    if (toCheck.size() > superSet.size()) return false;
    if (includes(superSet.begin(),superSet.end(),toCheck.begin(),toCheck.end()) ){
        return true;
    }
    
    return false;
}

void FukunagaAnalysis::updateState(int p, LandmarkState &s, bool isNumeric){
    if (!isNumeric){
        s.propositions.insert(p);
    }else{
        if (RPGBuilder::planAnyway) return;
        int idEff = p;
        RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[idEff];
        int fluentIndex = eff.fluentIndex;
        vector<double> weights = eff.weights;
        vector<int> variables = eff.variables;
        double constIncrease = 0;
        for (int z = 0; z < variables.size(); ++z){
            if ( fabs(weights[z]) >= 0.0001 && variables[z] != fluentIndex) cout << "error: there is a non-simple numeric condition" << endl;
            if (variables[z] == fluentIndex) constIncrease = weights[z];
        }
        if (!eff.isAssignment) constIncrease += eff.constant;
        double maxEff = constIncrease > 0 ? MAXN : -MAXN;//NumericAnalysis::getMinMaxNeeded()[fluentIndex].second : NumericAnalysis::getMinMaxNeeded()[fluentIndex].first;
        if (constIncrease > 0) s.maxV[fluentIndex] = maxEff;//s.maxV[fluentIndex] + constIncrease;//maxEff;
        else s.minV[fluentIndex] = maxEff;//s.minV[fluentIndex] + constIncrease ;//maxEff;
        
    }
}

void FukunagaAnalysis::addConditionLandmark(int p, int a, LandmarkState &s, vector<vector<bool>>& currentLandmarks, queue<int> & actions, vector<bool>& visited, bool isNumeric, list<int>& addConditions, set<int>& actionsSet){
    
    // update state
    //updateState(p,s,isNumeric);
    bool updated = updateCandidate(p, a, s, currentLandmarks,  addConditions);
//;
//    if (!isNumeric)
//        updated = updateCandidate(p,  a,  s, currentLandmarks, addConditions);
//    else{
//        updateCandidate(c, a, s, currentLandmarks,  addConditions);
//
//        for (auto c : addConditions){
//            if (c < RPGBuilder::getNLiterals()) continue;
//            else updated = updated || updateCandidate(c, a, s, currentLandmarks,  addConditions);
//        }
//    }
    //
    //    cout << "after applying action " << *RPGBuilder::getInstantiatedOp(a) << endl;
    //    s.printState();
    if (updated){
        for (auto i : actionsSet){
            if (!visited[i]) {
                if (checkActionSatisfied(i,s)){
                    actions.push(i);
                    visited[i]=true;
                }
            }
        }
    }
    //
}

bool  FukunagaAnalysis::updateCandidate(int p, int a,  LandmarkState & s, vector<vector<bool>>& currentLandmarks, list<int> &addedNumericCondition){
    bool debug = false;
    
    vector<bool> X;
    calculateNumericX(p,a,s,currentLandmarks,addedNumericCondition,X);
    if (debug){
        cout << p << "\nL: ";
        for ( auto l : currentLandmarks[p]){
            cout << l << " ";
        }
        cout << "\nX : " ;
        for ( auto x : X){
            cout << x << " ";
        }
        cout << endl;
    }
    if ( currentLandmarks[p] != X){
        currentLandmarks[p] = X;
        // check if action is satisfied //TODO: make this more efficient
        return true;
    }
    return false;
}

void FukunagaAnalysis::calculateNumericX(int p, int a, LandmarkState & s, vector<vector<bool>>& currentLandmarks, list<int> & addedNumericCondition, vector<bool> &toReturn){

	int vector_size = currentLandmarks.size();
    toReturn.assign(vector_size,false);
    // insert in X add effects    
    const list<Literal *> & addEff = RPGBuilder::getStartAddEffects()[a];
    for (auto & add : addEff){
        int idAdd = add->getStateID();
        toReturn[idAdd] = true;
    }
    // condition, check if condition is satisfied;
    //    for (int j = RPGBuilder::getNLiterals(); j < RPGBuilder::getNLiterals() + RPGBuilder::getNumericPreTable().size(); ++j){
    //        if (checkSatisfied( j,s,conditionSatisfied)) toReturn.insert(j);
    //    }
    // TODO: check this
    //for (auto & add : addedNumericCondition){
    //   toReturn[add] = true;
    //}
    toReturn[p] = true; // TODO: possibly delete add effect or add in not repeted relaxation
    /*
     list<int> & prec = getPrecondition()[a];
     for (auto & idPrec: prec){
     vector<bool> & Lpre = currentLandmarks[idPrec];
     for (int p = 0; p < Lpre.size(); ++p){
     bool pre = Lpre[p];
     if (pre) toReturn[p] = pre;
     }
     }
     */
    list<Literal *> & prec = RPGBuilder::getStartPropositionalPreconditions()[a];
    for (auto & pre: prec){
        int idPrec = pre->getStateID();
        vector<bool> & Lpre = currentLandmarks[idPrec];
        vector_size = Lpre.size();
        for (int p = 0; p < vector_size; ++p){
            if (Lpre[p]) toReturn[p] = true;
        }
    }
    
    
    /*
     list<int> & prec = getPrecondition()[a];
     for (int p = 0; p < toReturn.size(); ++p){
     for (auto & pr: prec){
     int idPrec = pr;//->getStateID();
     vector<bool> & Lpre = currentLandmarks[idPrec];
     bool pre = Lpre[p];
     if (pre) toReturn[p] = pre;
     }
     }
     */
    
    //const list<int> & preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[a];
    const list<int> & preconditions = getActionsToRPGExtendedNumericStartPreconditions()[a];
    for (auto pr: preconditions){
        int idPrec = pr + RPGBuilder::getNLiterals();
        vector<bool> & Lpre = currentLandmarks[idPrec];
        vector_size = Lpre.size();
        for (int pp = 0; pp < vector_size; ++pp){
            bool pre = Lpre[pp];
            if (pre) toReturn[pp] = pre;
        }
    }
    
    vector<bool> & L = currentLandmarks[p];
    vector_size = L.size();
    for (int pp = 0; pp < vector_size; ++pp){
        if (L[pp] && toReturn[pp]) toReturn[pp] = true;
        else toReturn[pp] = false;
    }
    //return intersect;
}

void FukunagaAnalysis::calculateNumericXIntersection(int p, int a, LandmarkState & s, vector<vector<bool>>& currentLandmarks, list<int> & addedNumericCondition, vector<bool> &toReturn){
    toReturn.assign(currentLandmarks.size(),false);
    // insert in X add effects
    
    // get achievers
    const vector<list<pair<int, VAL::time_spec> > > & actionsWithAdd = RPGBuilder::getEffectsToActions();
    vector<int> intersection(currentLandmarks.size(),0);
    int max = 0;
    
    if (p < RPGBuilder::getNLiterals()){
        for (auto it : actionsWithAdd[p]){
            int achiever = it.first;
            //if(checkActionSatisfied(achiever,s))
            {
                // for each precondition, add landmarks of all the preconditions
                calculateLandmarksIntersections(intersection,max, currentLandmarks,achiever,p);
            }
        }
    }else{
        list<int> achievers;
        getNumericAchievers(p,achievers,false);
        for (auto achiever : achievers){
            //if(checkActionSatisfied(achiever,s))
            {
                calculateLandmarksIntersections(intersection, max,currentLandmarks,achiever,p);
            }
        }
        
    }
    
    for (auto i : intersection){
        if (i==max) toReturn[i] = true;
    }
    const list<Literal *> & addEff = RPGBuilder::getStartAddEffects()[a];
    for (auto & add : addEff){
        int idAdd = add->getStateID();
        toReturn[idAdd] = true;
    }
    
    toReturn[p] = true; // TODO: possibly delete add effect or add in not repeted relaxation
    /*
     list<Literal *> & prec = RPGBuilder::getStartPropositionalPreconditions()[a];
     for (auto & pre: prec){
     int idPrec = pre->getStateID();
     vector<bool> & Lpre = currentLandmarks[idPrec];
     for (int p = 0; p < Lpre.size(); ++p){
     if (Lpre[p]) toReturn[p] = true;
     }
     }
     
     const list<int> & preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[a];
     for (auto pr: preconditions){
     int idPrec = pr + RPGBuilder::getNLiterals();
     vector<bool> & Lpre = currentLandmarks[idPrec];
     for (int pp = 0; pp < Lpre.size(); ++pp){
     bool pre = Lpre[pp];
     if (pre) toReturn[pp] = pre;
     }
     }
     */
    vector<bool> & L = currentLandmarks[p];
    
    for (int pp = 0; pp < L.size(); ++pp){
        if (L[pp] && toReturn[pp]) toReturn[pp] = true;
        else toReturn[pp] = false;
    }
    //return intersect;
}

void FukunagaAnalysis::calculateLandmarksIntersections(vector<int> & intersection, int & max, vector<vector<bool>> & currentLandmarks, int action, int p){
    
    list<Literal *> & prec = RPGBuilder::getStartPropositionalPreconditions()[action];
    vector<bool> landmarkAction(currentLandmarks.size(),false);
    for (auto & pre: prec){
        int idPrec = pre->getStateID();
        vector<bool> & Lpre = currentLandmarks[idPrec];
        for (int p = 0; p < Lpre.size(); ++p){
            if (Lpre[p]) landmarkAction[p]=true; ;
        }
    }
    
    const list<int> & preconditions = RPGBuilder::getActionsToRPGNumericStartPreconditions()[action];
    for (auto pr: preconditions){
        int idPrec = pr + RPGBuilder::getNLiterals();
        vector<bool> & Lpre = currentLandmarks[idPrec];
        for (int pp = 0; pp < Lpre.size(); ++pp){
            bool pre = Lpre[pp];
            if (pre)  landmarkAction[p]=true; ;
        }
    }
    for (int i = 0; i < currentLandmarks.size(); ++i){
        if (landmarkAction[i]) intersection[i] = intersection[i] + 1;
    }
    max++;
}
void FukunagaAnalysis::performRelevanceAnalysis(vector<vector<bool>>& factLandmarks, vector<vector<bool>> & factLandmarksForActions,vector<bool> &dominatedActions, vector<bool> &dominatedFacts, vector<vector<bool>> &fAdd,  MinimalState &state, vector<bool>& relevantActions, vector<bool>& relevantPropositions, bool debug){
    int nConditions=RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size();
    vector<vector<bool>> firstAchievers(RPGBuilder::getNOp(), vector<bool>(nConditions,false)); // this is actually useless
    if (debug) cout << "calculate first achievers" << endl;
    calculateFirstAchieversAndFactLandmarksForAction(factLandmarks, firstAchievers,fAdd,factLandmarksForActions);
    
    // TODO:  calculate fadd only one time
    if (debug) cout << "calculate relevantActions" << endl;
    //relevantActions.assign(RPGBuilder::getNOp(),false);
    getRelevantActions(relevantActions,dominatedActions, fAdd, debug);
    if (debug) cout << "calculate getRelevantPropositions" << endl;
    getRelevantPropositions(relevantPropositions, relevantActions, state, debug);
    
}



void FukunagaAnalysis::calculateFirstAchieversAndFactLandmarksForAction( vector<vector<bool>>& factLandmarks, vector<vector<bool> > &firstAchievers, vector<vector<bool>> & fAadd, vector<vector<bool>> &factLandmarksForActions){
    
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        
        list<Literal*> & prec = RPGBuilder::getStartPropositionalPreconditions()[i];
        const list<Literal*> & addEffs = RPGBuilder::getStartAddEffects()[i];
        list<int> numericEffs;
        getNumericAchievers(i, numericEffs, true);
        // calculate fact landmarks for actions
        for (int l = 0; l < RPGBuilder::getNLiterals(); ++l){
            // iterarate over all the precondition
            for (auto &p : prec){
                if (factLandmarks[p->getStateID()][l]) {
                    factLandmarksForActions[i][l] = true;
                    break;
                }
            }
        }
        for (auto & l : addEffs){
            int p = l->getStateID();
            if (!factLandmarksForActions[i][p]) {
                firstAchievers[i][p] = true;
                fAadd[i][p] = true;
            }
        }
        
        for (auto & p : numericEffs){
            if (!factLandmarksForActions[i][p]) {
                firstAchievers[i][p] = true;
                fAadd[i][p] = true;
            }
        }
    }
}

// todo: for numeric actions, add calculation of extimated time to achieve the condition
bool FukunagaAnalysis::dominatedActionElimination(MinimalState &theState, vector<vector<bool>> &fAdd, vector<vector<bool>> &factLandmarksForActions, vector<bool> & dominatedFacts, vector<bool> & dominatedActions, vector<bool>& relevantActions, bool debug){
    //bool debug = true;
    bool toReturn = false;
  //  calculateNumericDominance();
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        
        if (!relevantActions[i]) continue;
        //if (actionLandmarks[i]) continue; // TODO check this condition with
        // ./lprpgp-cplex -y planning-domains-numeric/sailing/domainPDDLValid.pddl planning-domains-numeric/sailing/sailing/instance_1_1_1229.pddl
        vector<bool> & faddI = fAdd[i];
        const list<Literal*> & addEffs = RPGBuilder::getStartAddEffects()[i];
        list<int> addConditionsI;
        //getNumericAchievers(i, addConditionsI,true); // this is not to check numeric effects
        bool isDominated = false;
        int dominatingAction = -1;
        for (int j = 0; j < RPGBuilder::getNOp(); ++j){
            if (!relevantActions[j]) continue;
            if (j==i) continue;
            // condition (iii)
            if (RPGBuilder::getActionCost()[i] < RPGBuilder::getActionCost()[j]) continue;
            vector<bool> & faddJ = fAdd[j];
            // condition (i) at page 643
            if (isSubsetOfI(addEffs,addConditionsI, faddI, faddJ) && dominatedNumericEffects(i,j)){             // check numeric effects
                // condition (ii) at page 643
                bool conditionII = true;
                list<Literal *> & preconditionsJ = RPGBuilder::getStartPropositionalPreconditions()[j];
                const list<int> & numPreconditionsJ = RPGBuilder::getActionsToRPGNumericStartPreconditions()[j];
                vector<bool> & landmarks = factLandmarksForActions[i];
                set<int> & initialState = theState.first;
                for (auto & p : preconditionsJ){
                    int pID = p->getStateID();
                    if ( !(initialState.find(pID) != initialState.end() || landmarks[pID]) ) {
                        conditionII = false;
                        break;
                    }
                }
                // TODO check numeric preconditions!
                for (auto & p : numPreconditionsJ){
                    int pID = p + RPGBuilder::getNLiterals();
                    if ( !(checkSatisfied(pID,theState.first,theState.second,theState.second) || landmarks[pID]) ) {
                        conditionII = false;
                        break;
                    }
                }
                if (conditionII){
                    // TODO: condition (iv)
                    if (calculateNumericDominance()[i][j]){
                        isDominated = true;
                        dominatingAction = j;
                        break;
                    }
                }
            }
            
        }
        
        if (isDominated){
            if (debug){
                cout << "action " << *(RPGBuilder::getInstantiatedOp(i)) << " is dominated by " << *(RPGBuilder::getInstantiatedOp(dominatingAction)) << endl;
                //cout << "fadd 1 : ";
                //for (auto f : fAdd[i]) cout << f << " ";
                //cout << "\nfadd 2 : ";
                for (auto f : fAdd[dominatingAction]) cout << f << " ";
                cout << "\n";
            }
            relevantActions[i] = false;
            if (!dominatedActions[i]){
                dominatedActions[i] = true;
                toReturn = true;
            }
        }else{
            if (debug){
                cout << "action " << *(RPGBuilder::getInstantiatedOp(i)) << " is  not dominated by anything" << endl;
            }
            //relevantActions[i] = false;
        }
    }
    if (debug) {
        for (int j = 0; j< RPGBuilder::getNOp(); ++j){
            if (relevantActions[j]) {
                cout << *RPGBuilder::getInstantiatedOp(j) << " is NOT a dominated action"  << endl;
            }else{
                cout << *RPGBuilder::getInstantiatedOp(j) << " is a dominated action"  << endl;
            }
        }
    }
    return toReturn;
}

void FukunagaAnalysis::iterativeVariableElimination(MinimalState &state, vector<bool> &actionLandmarks, vector<bool> &factLandmarks, vector<bool> &eliminatedActions, vector<bool> &eliminatedFacts, vector<vector<bool>> & fAdd,bool debug){
    
    // landmark extraction
    bool includeNumeric = true;
    set<int> actions;
    for (int i = 0; i<RPGBuilder::getNOp(); ++i) actions.insert(i);
    vector<vector<bool>> factLandmarksTable;
    if (MILPRPG::addLandmarks){
        FukunagaAnalysis::performLandmarkAnalysis(actions,state,includeNumeric, factLandmarksTable, debug);
    }else{
        int nConditions = includeNumeric ? RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size() : RPGBuilder::getNLiterals();
        factLandmarksTable.assign(nConditions,vector<bool>(nConditions,false));
        for (int i = 0; i < nConditions; ++i){
            factLandmarksTable[i][i] = true;
        }

    }
    extractLandmarkActionsAndFacts(state,factLandmarksTable, factLandmarks,actionLandmarks,includeNumeric, debug);

    // relevance analysis
    if (RPGBuilder::useExtraConstraints || RPGBuilder::useSEQConstraints) {
        int nConditions = RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size() ;
        vector<vector<bool>> factLandmarksForActions(RPGBuilder::getNOp(), vector<bool>(nConditions,false));
        vector<bool> dominatedFacts(nConditions,false);
        vector<bool> dominatedActions(RPGBuilder::getNOp(),false);
        performRelevanceAnalysis(factLandmarksTable,factLandmarksForActions, dominatedActions, dominatedFacts, fAdd, state, eliminatedActions, eliminatedFacts, debug);
        bool iterate = !RPGBuilder::useSEQConstraints;
        while (iterate){
            if (!RPGBuilder::planAnyway){
                iterate = dominatedActionElimination(state, fAdd, factLandmarksForActions,dominatedFacts,dominatedActions, eliminatedActions, debug);
                if (iterate)
                    getRelevantActions(eliminatedActions, dominatedActions,fAdd,debug);
            }
        }
    }
}

void FukunagaAnalysis::extractLandmarkActionsAndFacts(MinimalState &state, vector<vector<bool>> & factLandmarksTable, vector<bool> & factLandmarks, vector<bool> & actionLandmarks, bool includeNumeric, bool debug){
    // state
    int nConditions = includeNumeric ? RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size() : RPGBuilder::getNLiterals() ;
    vector<bool> conditionSatisfied(nConditions);
    // goal
    list<Literal*> & goals = RPGBuilder::getLiteralGoals();
    for (auto& l : goals){
        int lID = l->getStateID();
        // for (auto i : factLandmarksTable[lID]){
        for (int i = 0; i < factLandmarksTable[lID].size(); ++i){
            if (!factLandmarksTable[lID][i]) continue;
            // propositional condition
            if (checkSatisfied(i,state.first, state.second, state.second)) continue;
            factLandmarks[i]=true;
            const vector<list<pair<int, VAL::time_spec> > > & actionsWithAdd = RPGBuilder::getEffectsToActions();
            if ( i < RPGBuilder::getNLiterals()){
                const list<pair<int,VAL::time_spec>> & achievers = actionsWithAdd[i];
                if (achievers.size()==1){
                    // add action landmark
                    actionLandmarks[achievers.begin()->first] = true;
                }
            }else{
                list<int> achievers;
                getNumericAchievers(i, achievers, false);
                if (achievers.size()==1){
                    // add action landmark
                    actionLandmarks[*achievers.begin()] = true;
                }
            }
            
        }
    }
    if (includeNumeric){
        list<pair<int, int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
        for (auto goal : numericGoals){
            if (goal.first!=-1){
                int lID = goal.first + RPGBuilder::getNLiterals();
                for (int i = 0; i < factLandmarksTable[lID].size(); ++i){
                    if (!factLandmarksTable[lID][i]) continue;
                    // propositional condition
                    if (checkSatisfied(i,state.first, state.second, state.second)) continue;
                    factLandmarks[i] = true;
                    const vector<list<pair<int, VAL::time_spec> > > & actionsWithAdd = RPGBuilder::getEffectsToActions();
                    if ( i < RPGBuilder::getNLiterals()){
                        const list<pair<int,VAL::time_spec>> & achievers = actionsWithAdd[i];
                        if (achievers.size()==1){
                            // add action landmark
                            actionLandmarks[achievers.begin()->first];
                        }
                    }else{
                        list<int> achievers;
                        getNumericAchievers(i, achievers, false);
                        if (achievers.size()==1){
                            // add action landmark
                            actionLandmarks[*achievers.begin()];
                        }
                    }
                }
            }
        }
    }
    if (debug){
        cout << " --- final landmarks --- " << endl;
        for (int a = 0; a < actionLandmarks.size() ; ++a){
            if (actionLandmarks[a]) cout << *RPGBuilder::getInstantiatedOps()[a] << " is an action landmark" << endl;
        }
        for (int a = 0; a < factLandmarks.size() ; ++a){
            if (!factLandmarks[a]) continue;
            else if (a < RPGBuilder::getNLiterals()) cout << *RPGBuilder::getLiteral(a) << " is a fact landmark" << endl;
            else{
                cout << "condition " << RPGBuilder::getNumericPreTable()[a - RPGBuilder::getNLiterals()] << " is a fact landmark" << endl;
            }
        }
        for (int v = 0; v < RPGBuilder::getPNECount(); ++v){
            cout << *RPGBuilder::getPNE(v) << " " << state.second[v] << endl;
        }
    }
    
}

void FukunagaAnalysis::calculateNumericAchievers(vector<list<int>>& numericAchieversTable, vector<list<int>> & numericAdders){
    numericAchieversTable.assign(RPGBuilder::getNumericPreTable().size(),list<int>());
    numericAdders.assign(RPGBuilder::getNOp(), list<int>());
    if (RPGBuilder::planAnyway) return;
    for (int p = RPGBuilder::getNLiterals(); p < RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size(); ++p){
        list<int> toInsert;
        int c = p - RPGBuilder::getNLiterals();
        RPGBuilder::RPGNumericPrecondition precondition = RPGBuilder::getNumericPreTable()[c];
        pair<list<pair<int,double>>,double> exp = getExpression(precondition);
        for (int i = 0; i<RPGBuilder::getNOp(); ++i){
            list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[i];
            for (int ne : numEff){
                bool toBreak = false;
                RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[ne];
                int fluentIndex = eff.fluentIndex;
                vector<double> weights = eff.weights;
                vector<int> variables = eff.variables;
                double constIncrease = 0;
                for (int z = 0; z < variables.size(); ++z){
                    if ( fabs(weights[z]) >= 0.0001 && variables[z] != fluentIndex) cout << "error: there is a non-simple numeric condition" << endl;
                    if (variables[z] == fluentIndex) constIncrease = weights[z];
                }
                if (!eff.isAssignment) constIncrease += eff.constant;
                for (pair<int,double> e : exp.first){
                    if (e.first == fluentIndex && constIncrease*e.second > 0){
                        numericAdders[i].push_back(p);
                        toInsert.push_back(i);
                        toBreak = true;
                        break;
                    }
                }
                if (toBreak) break;
            }
        }
        numericAchieversTable[c] = toInsert;
    }
}

void FukunagaAnalysis::calculateNumericIncreasers(vector<list<double>> &increasers , vector<list<double>> &decreasers){
    increasers.assign(RPGBuilder::getPNECount(),list<double>());
    decreasers.assign(RPGBuilder::getPNECount(),list<double>());
    for (int v = 0; v < RPGBuilder::getPNECount(); ++v){
        for (int i = 0; i<RPGBuilder::getNOp(); ++i){
            list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[i];
            for (int ne : numEff){
                bool toBreak = false;
                RPGBuilder::RPGNumericEffect eff = RPGBuilder::getNumericEff()[ne];
                int fluentIndex = eff.fluentIndex;
                vector<double> weights = eff.weights;
                vector<int> variables = eff.variables;
                double constIncrease = eff.constant;
                if (fluentIndex!=v) continue;
//                for (int z = 0; z < variables.size(); ++z){
//                    if ( fabs(weights[z]) >= 0.0001 && variables[z] != fluentIndex) cout << "error: there is a non-simple numeric condition" << endl;
//                    if (variables[z] == fluentIndex) constIncrease = weights[z];
//                }
                if (constIncrease>0){
                    increasers[v].push_back(constIncrease);
                    //cout << "increaser of " << *RPGBuilder::getPNE(v) << " is action " << *RPGBuilder::getInstantiatedOp(i) << " by " << constIncrease << endl;
                }else{
                    decreasers[v].push_back(constIncrease);
                    //cout << "decreaser of " << *RPGBuilder::getPNE(v) << " is action " << *RPGBuilder::getInstantiatedOp(i) << " by " << constIncrease << endl;
                }
            }
        }
    }
}

void FukunagaAnalysis::getNumericAchievers(int p, list<int> & numericAchievers, bool adder){
    if (RPGBuilder::getNumericPreTable().size()==0) return;
    static vector<list<int>> numericAchieversTable;
    static vector<list<int>> numericAdders;
    if (numericAchieversTable.size() == 0){
        calculateNumericAchievers(numericAchieversTable, numericAdders);
    }
    if(!adder) {
        numericAchievers = numericAchieversTable[p-RPGBuilder::getNLiterals()];
    }else{
        numericAchievers = numericAdders[p];
    }
}

bool FukunagaAnalysis::getRelevantActions(vector<bool>& relevantActions, vector<bool>&dominatedActions, vector<vector<bool>> & fAdd, bool debug){
    //bool debug = true;
    relevantActions.assign(RPGBuilder::getNOp(),false);
    bool toReturn = false;
    int nRelevantActions = -1;
    list<Literal*> & goals = RPGBuilder::getLiteralGoals();
    list<pair<int, int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
    bool first = true;
    while (nRelevantActions != 0){
        nRelevantActions = 0;
        for (int i = 0; i< RPGBuilder::getNOp(); ++i){
            if (relevantActions[i]) continue;
            // check if add goal
            vector<bool> & fadd = fAdd[i];
            if (first) {
                if (!emptyIntersection(goals,fadd) && !relevantActions[i]) {
                    relevantActions[i] = true;
                    toReturn = true;
                    nRelevantActions++;
                    continue;
                }
                if (!emptyIntersection(numericGoals,fadd) && !relevantActions[i]) {
                    relevantActions[i] = true;
                    toReturn = true;
                    nRelevantActions++;
                    continue;
                }
            }else{
                // check if any relevant action has preconditions added by this action
                for (int j = 0; j< RPGBuilder::getNOp(); ++j){
                    if ( i == j ) continue;
                    if (!relevantActions[j]) continue;
                    if (dominatedActions[j]) continue;
                    list<Literal*> & prec= RPGBuilder::getStartPropositionalPreconditions()[j];
                    if (!emptyIntersection(prec,fadd) && !relevantActions[i]) {
                        relevantActions[i] = true;
                        toReturn = true;
                        nRelevantActions++;
                        break;
                        
                    }
                    const list<int> & numPrec = RPGBuilder::getActionsToRPGNumericStartPreconditions()[j];
                    if (!emptyIntersection(numPrec,fadd) && !relevantActions[i]) {
                        relevantActions[i] = true;
                        toReturn = true;
                        nRelevantActions++;
                        break;
                        
                    }
                }
            }
        }
        first = false;
    }
    if (debug) {
        for (int j = 0; j< RPGBuilder::getNOp(); ++j){
            if (relevantActions[j]) {
                cout << *RPGBuilder::getInstantiatedOp(j) << " is a relevant action"  << endl;
            }else{
                cout << *RPGBuilder::getInstantiatedOp(j) << " is NOT a relevant action"  << endl;
            }
        }
    }
    return toReturn;
}

bool FukunagaAnalysis::getRelevantPropositions(vector<bool>& relevantPropositions, vector<bool>& relevantActions, MinimalState &theState, bool debug){
    bool toReturn = false;
    // check relevant proposition
    set<int> & propositions = theState.first;
    list<Literal*> & goals = RPGBuilder::getLiteralGoals();
    for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
        // check if it's goal
        Literal *l = RPGBuilder::getLiteral(i);
        if (propositions.find(i)!=propositions.end()){
            relevantPropositions[i]=true;
            toReturn  = true;
            continue;
        }
        if (find(goals.begin(),goals.end(),l)!=goals.end()){
            relevantPropositions[i]=true;
            toReturn  = true;
            continue;
        }
        list<pair<int, VAL::time_spec> > actions = RPGBuilder::getProcessedPreconditionsToActions()[i];
        for (auto a : actions){
            if (relevantActions[a.first]) {
                relevantPropositions[i] = true;
                toReturn  = true;
                break;
            }
        }
    }
    
    // numeric part
    list<pair<int, int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
    for (auto ng : numericGoals){
        if (ng.first!=-1) relevantPropositions[ng.first+RPGBuilder::getNLiterals()] = true;
    }
    for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
        // check if it's goal
        int c = i + RPGBuilder::getNLiterals();
        if (relevantPropositions[c]) continue;
        // check if it's initial state
        if (checkSatisfied(c,theState.first,theState.second,theState.second)){
            relevantPropositions[c] = true;
            toReturn = true;
            continue;
        }
        
        list<int> actions;
        if (!RPGBuilder::planAnyway) getNumericAchievers(c,actions,false);
        for (auto a : actions){
            if (relevantActions[a]) {
                relevantPropositions[i] = true;
                toReturn  = true;
                break;
            }
        }
    }
    
    if (debug) {
        for (int i = 0; i < RPGBuilder::getNLiterals(); ++i){
            cout << "proposition " << *(RPGBuilder::getLiteral(i));
            if  (relevantPropositions[i] == 0) cout << " is NOT relevant" << endl;
            else cout << " is relevant" << endl;
        }
    }
}

bool FukunagaAnalysis::emptyIntersection(list<Literal *> &goals,  set<int> &fadd){
    set<int>::iterator j = fadd.begin();
    for (;j != fadd.end(); ++j){
        list<Literal *>::iterator i = goals.begin();
        for (;i!= goals.end();++i){
            //cout << "\t" << **i << " " << *RPGBuilder::getLiteral(*j) << endl;
            if (*i == RPGBuilder::getLiteral(*j))
                return false;
        }
    }
    return true;
}

bool FukunagaAnalysis::emptyIntersection(list<Literal *> &goals,  vector<bool> &fadd){
    for (auto & g : goals){
        if (fadd[g->getStateID()]) return false;
    }
    return true;
}

bool FukunagaAnalysis::emptyIntersection(list<pair<int, int> >  &goals,  vector<bool> &fadd){
    for (auto & g : goals){
        if (g.first!=-1 && fadd[g.first]) return false;
    }
    return true;
}


bool FukunagaAnalysis::emptyIntersection(const list<int >  &goals,  vector<bool> &fadd){
    for (auto & g : goals){
        if (fadd[g + RPGBuilder::getNLiterals()]) return false;
    }
    return true;
}

vector<list<int>> & FukunagaAnalysis::getPrecondition(){
    static vector<list<int>> preconditions;
    if (preconditions.size()==0){
        preconditions.assign(RPGBuilder::getNOp(), list<int>());
        for (int i = 0; i < preconditions.size(); ++i){
            list<Literal *> & prec = RPGBuilder::getStartPropositionalPreconditions()[i];
            list<int> toFill;
            for (auto & l : prec){
                toFill.push_back(l->getStateID());
            }
            preconditions[i] = toFill;
        }
    }
    return preconditions;
}

vector<set<int>> & FukunagaAnalysis::getPreconditionSet(){
    static vector<set<int>> preconditions;
    if (preconditions.size()==0){
        preconditions.assign(RPGBuilder::getNOp(), set<int>());
        for (int i = 0; i < preconditions.size(); ++i){
            list<Literal *> & prec = RPGBuilder::getStartPropositionalPreconditions()[i];
            set<int> toFill;
            for (auto & l : prec){
                toFill.insert(l->getStateID());
            }
            preconditions[i] = toFill;
        }
    }
    return preconditions;
}

void FukunagaAnalysis::getInverseActions(map<int,list<int>>& inverseActions){
    for (int i = 0; i < RPGBuilder::getNOp(); ++i){
        for (int j = 0; j < RPGBuilder::getNOp(); ++j){
            if (j==i) continue;
            const list<Literal *> & addEffI = RPGBuilder::getStartAddEffects()[i];
            const list<Literal *> & addEffJ = RPGBuilder::getStartAddEffects()[j];
            list<Literal*> & precI = RPGBuilder::getStartPropositionalPreconditions()[i];
            list<Literal*> & precJ = RPGBuilder::getStartPropositionalPreconditions()[j];
            list<int> & numEffI = RPGBuilder::getActionsToRPGNumericStartEffects()[i];
            
            if (isSubsetOfI(addEffI, precJ) && isSubsetOfI(addEffJ, precI)){
                list<int> & numEffI = RPGBuilder::getActionsToRPGNumericStartEffects()[i];
                list<int> & numEffJ = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
                const list<int> & precI = RPGBuilder::getActionsToRPGNumericStartPreconditions()[i];
                const list<int> & precJ = RPGBuilder::getActionsToRPGNumericStartPreconditions()[j];
                if (numEffI.empty() && numEffJ.empty() && precI.empty() && precJ.empty()) {
                    if (inverseActions.find(i)!=inverseActions.end())
                        inverseActions[i].push_back(j);
                    else {
                        list<int> first;
                        first.push_back(j);
                        inverseActions.insert(make_pair(i,first));
                    }
//                }else{
//                    // add condition on inverse actions
//                    bool foundInvers = true;
//                    for (auto ni : numEffI){
//                        RPGBuilder::RPGNumericEffect & effI = RPGBuilder::getNumericEff()[ni];
//                        bool getInverse = false;
//                        for (auto nj : numEffJ){
//                            // if same numEff,
//                            RPGBuilder::RPGNumericEffect & effJ = RPGBuilder::getNumericEff()[nj];
//                            if (effI.fluentIndex != effJ.fluentIndex) continue;
//                            if (effI.constant * effJ.constant <=0 && fabs(effI.constant) >= fabs(effJ.constant)) getInverse = true;
//                        }
//                        if(!getInverse){
//                            foundInvers = false;
//                            break;
//                        }
//                    }
//                    if (foundInvers){
//                        if (inverseActions.find(i)!=inverseActions.end())
//                            inverseActions[i].push_back(j);
//                        else {
//                            list<int> first;
//                            first.push_back(j);
//                            inverseActions.insert(make_pair(i,first));
//                        }
//                    }
                }
            }
        }
    }
}

void FukunagaAnalysis::extractLandmarkAndOrGraph(MinimalState &state, vector<vector<bool>> & factLandmarksTable, vector<bool> & factLandmarks, vector<bool> & actionLandmarks, bool includeNumeric, bool debug){
    int nConditions = includeNumeric ? RPGBuilder::getNLiterals()+RPGBuilder::getNumericPreTable().size() : RPGBuilder::getNLiterals();
    vector<bool> propositionsVisited(nConditions,false);
    vector<bool> actionsVisited(RPGBuilder::getNOp(),false);
    for (int i = 0; i < nConditions; ++i){
        if (propositionsVisited[i]) continue;
        // get edges achievers
        
        
    }
}

bool FukunagaAnalysis::dominatedNumericEffects(int ix, int jx){
    static vector<vector<bool>> actionsDominationByNumericEffects(RPGBuilder::getNOp(), vector<bool>(RPGBuilder::getNOp(),true));
    static bool first = true;
    if (first){
        for (int i = 0; i < RPGBuilder::getNOp(); ++i){
            for (int j = 0; j < RPGBuilder::getNOp(); ++j){
                list<int> & numEffI = RPGBuilder::getActionsToRPGNumericStartEffects()[i];
                list<int> & numEffJ = RPGBuilder::getActionsToRPGNumericStartEffects()[j];
                for (auto iN : numEffI){
                    RPGBuilder::RPGNumericEffect & effI = RPGBuilder::getNumericEff()[iN];
                    int fI = effI.fluentIndex;
                    vector<double> & wI = effI.weights;
                    vector<int> & vI = effI.variables;
                    
                    bool effectDominated = false;
                    for (auto jN : numEffJ){
                        if (iN==jN) {
                            effectDominated = true;
                            break;
                        }
                        RPGBuilder::RPGNumericEffect & effJ = RPGBuilder::getNumericEff()[jN];
                        int fJ = effJ.fluentIndex;
                        vector<double> & wJ = effJ.weights;
                        vector<int> & vJ = effJ.variables;
                        // check if not dominated
                        if (fI==fJ && wI.size()==0 && wJ.size()==0){
                            if ( effI.constant*effJ.constant > 0){
                                if ( fabs(effI.constant) <= fabs(effJ.constant)){
                                    effectDominated = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (!effectDominated) {
                        //cout << *(RPGBuilder::getInstantiatedOp(i)) << " is NOT dominated by " << *(RPGBuilder::getInstantiatedOp(j)) << endl;
                        actionsDominationByNumericEffects[i][j] = false;
                        break;
                    } else{
                        //cout << *(RPGBuilder::getInstantiatedOp(i)) << " is dominated by " << *(RPGBuilder::getInstantiatedOp(j)) << endl;
                        
                    }
                }
            }
        }
        first = false;
    }
    return actionsDominationByNumericEffects[ix][jx];
}

vector<list<int>> &  FukunagaAnalysis::getActionsToRPGExtendedNumericStartPreconditions(){
    static bool first = true;
    static vector<list<int> > prec(RPGBuilder::getNOp(), list<int>());
    if (first){
        first = false;
        for (int a = 0; a < RPGBuilder::getNOp(); ++a){
            list<int> toAdd;
            const list<int> & numPrec = RPGBuilder::getActionsToRPGNumericStartPreconditions()[a];
            for (auto n : numPrec){
                if (find(toAdd.begin(), toAdd.end(), n)==toAdd.end()) toAdd.push_back(n);
                const RPGBuilder::RPGNumericPrecondition & precondition = RPGBuilder::getNumericPreTable()[n];
                pair<list<pair<int,double>>,double> preconditionList = getExpression(precondition);
                for (int j = 0; j < RPGBuilder::getNumericPreTable().size(); ++j){
                    if (n==j) continue;
                    const RPGBuilder::RPGNumericPrecondition & preJ = RPGBuilder::getNumericPreTable()[j];
                    pair<list<pair<int,double>>,double> preconditionListJ = getExpression(preJ);
                    if (preconditionList.first.size() != preconditionListJ.first.size()) continue;
                    if (preconditionList.second <= preconditionListJ.second)             continue;
                    bool equivalent = true;
                    for ( auto p : preconditionList.first){
                        if (find(preconditionListJ.first.begin(), preconditionListJ.first.end(), p)== preconditionListJ.first.end()){
                            equivalent = false;
                            break;
                        }
                    }
                    if (equivalent){
                        if (find(toAdd.begin(), toAdd.end(), j)==toAdd.end()){
                            toAdd.push_back(j);
                        }
                        //cout << "************** " << precondition << " is dominated by " << preJ << endl;
                    }
                }
            }
            prec[a] = toAdd;
        }
    }
    return prec;
}

vector<vector<bool>> & FukunagaAnalysis::calculateNumericDominance(){
    static  vector<vector<bool>>  numericDominance(RPGBuilder::getNOp(),vector<bool>(RPGBuilder::getNOp(),false));
    static bool first = true;
    static bool debug = false;
    static int vCount = RPGBuilder::getPNECount() + RPGBuilder::getNumericPreTable().size();
    static vector<vector<double>> totalEffectOfActions(RPGBuilder::getNOp(),vector<double>(vCount,0));
    if (first){
        first  = false;
        numericDominance.assign(RPGBuilder::getNOp(),vector<bool>(RPGBuilder::getNOp(),false));
        if (RPGBuilder::planAnyway) return numericDominance;
        for(int i = 0; i < RPGBuilder::getNOp(); ++i){
            if (debug) cout << *RPGBuilder::getInstantiatedOp(i) << endl;
            list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[i];
            for (int idEff : numEff){
                RPGBuilder::RPGNumericEffect & eff = RPGBuilder::getNumericEff()[idEff];
                int fluentIndex = eff.fluentIndex;
                vector<double> weights = eff.weights;
                vector<int> variables = eff.variables;
                double constIncrease = 0;
                for (int z = 0; z < variables.size(); ++z){
                    if ( fabs(weights[z]) >= 0.0001 && variables[z] != fluentIndex) cout << "error: there is a non-simple numeric condition" << endl;
                    if (variables[z] == fluentIndex) constIncrease = weights[z];
                }
                if (!eff.isAssignment) constIncrease += eff.constant;
                totalEffectOfActions[i][fluentIndex] = constIncrease;
            }
            if (debug){
                for (int v = 0; v < RPGBuilder::getPNECount(); ++v) {
                    cout << "\t" <<  *(RPGBuilder::getPNE(v))  << " " << totalEffectOfActions[i][v] << endl;
                }
            }
            for (int idEff = 0; idEff < vCount - RPGBuilder::getPNECount(); ++ idEff){
                RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[idEff];
                if (debug) cout << "\t" << prec << endl;
                pair<list<pair<int,double>>,double> precondition = getExpression(prec);
                for (auto & p : precondition.first){
                    totalEffectOfActions[i][RPGBuilder::getPNECount()+idEff] += totalEffectOfActions[i][p.first]*p.second;
                }
                 if (debug) cout << "\t\t" <<totalEffectOfActions[i][RPGBuilder::getPNECount()+idEff] << endl;
            }
        }
        for(int i = 0; i < RPGBuilder::getNOp(); ++i){
            for(int j = 0; j < RPGBuilder::getNOp(); ++j){
                bool dominated = true;
                for (int v = 0; v < vCount; ++v){
                    if (totalEffectOfActions[i][v] >=0 && totalEffectOfActions[j][v] >= 0  && totalEffectOfActions[i][v] <= totalEffectOfActions[j][v]){
                        dominated = false;
                        break;
                    }
                }
                numericDominance[i][j] = dominated;
                if (debug) cout << *RPGBuilder::getInstantiatedOp(i) << " " << *RPGBuilder::getInstantiatedOp(j) << dominated << endl;
            }
        }
        
    }
    return numericDominance;
}

void FukunagaAnalysis::getTimeStampedMaxValuesGeneral(vector<vector<pair<double,double>>> & minMaxValues, vector<vector<pair<double,double>>>  & minMaxSteps, vector<double> & minEffect, int tMax, MinimalState &theState){
    static vector<list<pair<int,double>>> simpleEffectActionsConstant(0);
    static vector<list<pair<int,int>>> linearEffectActionEffectID(0);
    minEffect.assign(RPGBuilder::getNumericPreTable().size(),0.00001);
    static bool first = true;
    if (first){
        getActionsEffects(simpleEffectActionsConstant,linearEffectActionEffectID);
        first = false;
        // calculate minEff
        for (int i = 0; i < RPGBuilder::getNumericPreTable().size(); ++i){
            RPGBuilder::RPGNumericPrecondition prec = RPGBuilder::getNumericPreTable()[i];
            vector<double> minEffAction(RPGBuilder::getNOp(),0);
            pair<list<pair<int,double> >,double> precondition = FukunagaAnalysis::getExpression(prec);
            if (prec.op == VAL::E_GREATER){
                list<pair<int,double> > coefficient = precondition.first;
                for (auto prec : coefficient){
                    int v = prec.first;
                    double w = prec.second;
                    if (linearEffectActionEffectID[v].size()!=0){
                        minEffect[i] = 0.00001;
                        break;
                    }
                    list<pair<int,double>> scactions = simpleEffectActionsConstant[v];
                    for (auto ak : scactions){
                        minEffAction[ak.first] += w * ak.second;
                    }
                }
                double minConstantCoefficient = 99999999;
                for (int ji = 0; ji < RPGBuilder::getNOp(); ++ji){
                    if (abs(minEffAction[ji])>0.0001 && abs(minEffAction[ji]) < minConstantCoefficient){
                        minConstantCoefficient = abs(minEffAction[ji]);
                    }
                }
                minEffect[i] = minConstantCoefficient;
                //cout << " constraint " << i << " " << minConstantCoefficient << endl;
            }
        }
    }
    
    for (int t = minMaxValues.size(); t < tMax; ++t){
        // add layer
        minMaxValues.push_back(vector<pair<double,double>>(RPGBuilder::getPNECount(),make_pair(DBL_MIN, DBL_MAX)));
        minMaxSteps.push_back(vector<pair<double,double>>(RPGBuilder::getPNECount(),make_pair(DBL_MIN, DBL_MAX)));
        for (int v = 0; v < RPGBuilder::getPNECount(); ++v){
            // if first time-step, initialise with initial state values
            if (t==0){
                minMaxValues[t][v].first = theState.second[v];
                minMaxValues[t][v].second = theState.second[v];
            } else {
                // get effect of previous time-step and add maximum effect
                // (at the moment apply all the actions, not checking if
                // precondition are satisfied or not)
                minMaxValues[t][v].first = minMaxValues[t - 1][v].first;
                minMaxValues[t][v].second = minMaxValues[t - 1][v].second;
                for (auto ac : simpleEffectActionsConstant[v]){
                    double c = ac.second;
                    if(c > 0){
                        minMaxValues[t][v].second += c;
                    } else{
                        minMaxValues[t][v].first += c;
                    }
                }
                if (!RPGBuilder::planAnyway){
                minMaxValues[t][v].first = minMaxValues[t][v].first > Planner::NumericAnalysis::getBounds()[v].first ? minMaxValues[t][v].first : Planner::NumericAnalysis::getBounds()[v].first;
                minMaxValues[t][v].second = minMaxValues[t][v].second < Planner::NumericAnalysis::getBounds()[v].second ? minMaxValues[t][v].second : Planner::NumericAnalysis::getBounds()[v].second;
                }
                for (auto actionLinear : linearEffectActionEffectID[v]){
                    
                    int idEff = actionLinear.second;
                    RPGBuilder::RPGNumericEffect & eff = RPGBuilder::getNumericEff()[idEff];
                    int indexFluent = eff.fluentIndex;
                    //cout << *RPGBuilder::getPNE(indexFluent)  << " " << minMaxValues[t][v].first << " " << minMaxValues[t][v].second << " modified by " << *RPGBuilder::getInstantiatedOp(actionLinear.first) << endl;
                    bool isAssignment = eff.isAssignment;
                    vector<double> weights = eff.weights;
                    vector<int> variables = eff.variables;
                    double constant = eff.constant;
                    double maxEff = 0;
                    double minEff = 0;
                    for (int i = 0; i<weights.size(); ++i){
                        double w = weights[i];
                        int x = variables[i];
                        if (x < RPGBuilder::getPNECount()){
                            //cout << "\t" << x << " " << *RPGBuilder::getPNE(x) << "*" << w << endl;
                            if ( w > 0){
                                maxEff += w*minMaxValues[t-1][x].second;
                                minEff += w*minMaxValues[t-1][x].second;
                            }else{
                                minEff -= w*minMaxValues[t-1][x].first;
                                maxEff -= w*minMaxValues[t-1][x].first;
                            }
                        }else if (x < 2*RPGBuilder::getPNECount()) {
                            double w = -weights[i];
                            int x = variables[i] - RPGBuilder::getPNECount();
                            //cout << "\t-" << x << " " << *RPGBuilder::getPNE(x) << "*" << w << endl;
                            if ( w > 0){
                                maxEff += w*minMaxValues[t-1][x].second;
                                minEff += w*minMaxValues[t-1][x].second;
                            }else{
                                minEff -= w*minMaxValues[t-1][x].first;
                                maxEff -= w*minMaxValues[t-1][x].first;
                            }
                        }else{
                            cout << "case not handled in big M constraints" << endl;
                        }
                    }
                    if (constant > 0){
                        maxEff+=constant;
                    }else{
                        minEff+=constant;
                    }
                    if (!isAssignment){
                        maxEff += minMaxValues[t-1][v].second;
                        //minEff += w*minMaxValues[t-1][x].second;
                    }
//                    cout << *RPGBuilder::getPNE(indexFluent) << " at time " << t << " SC " << minMaxValues[t][indexFluent].first << " " << minMaxValues[t][indexFluent].second << " - after linear " <<  *RPGBuilder::getInstantiatedOp(actionLinear.first) << " " << minEff << " " << maxEff << endl;
                    if (maxEff >= minMaxValues[t][indexFluent].second) minMaxValues[t][indexFluent].second = maxEff;
                    if (minEff <= minMaxValues[t][indexFluent].first) minMaxValues[t][indexFluent].first = minEff;
                }
                
                //minMaxValues[t][v].second = 20;
                //minMaxValues[t][v].first = -20;

            }
        }
    }
}


void FukunagaAnalysis::getTimeStampedMaxValues(vector<vector<pair<double,double>>> & minMaxValues, vector<vector<pair<double,double>>>  & minMaxSteps, int tMax, MinimalState &theState){
    static vector<list<double>> increasers(0);
    static vector<list<double>> decreasers(0);
    static bool first = true;
    if (first){
        calculateNumericIncreasers(increasers,decreasers);
        first = false;
    }
    for (int t = minMaxValues.size(); t < tMax; ++t){
        // add layer
        minMaxValues.push_back(vector<pair<double,double>>(RPGBuilder::getPNECount(),make_pair(DBL_MIN, DBL_MAX)));
        minMaxSteps.push_back(vector<pair<double,double>>(RPGBuilder::getPNECount(),make_pair(DBL_MIN, DBL_MAX)));
        for (int v = 0; v < RPGBuilder::getPNECount(); ++v){
            // if first time-step, initialise with initial state values
            if (t==0){
                minMaxValues[t][v].first = theState.second[v];
                minMaxValues[t][v].second = theState.second[v];
            } else {
                // get effect of previous time-step and add maximum effect
                // (at the moment apply all the actions, not checking if
                // precondition are satisfied or not)
                minMaxValues[t][v].first = minMaxValues[t - 1][v].first;
                for (auto c : decreasers[v]) minMaxValues[t][v].first += c;
                
                minMaxValues[t][v].first = minMaxValues[t][v].first > Planner::NumericAnalysis::getBounds()[v].first ? minMaxValues[t][v].first : Planner::NumericAnalysis::getBounds()[v].first;
                
                //minMaxValues[t][v].first = minMaxValues[t][v].first > theState.second[v] ? theState.second[v] : minMaxValues[t][v].first;

                minMaxValues[t][v].second = minMaxValues[t - 1][v].second;
                for (auto c : increasers[v]) minMaxValues[t][v].second += c;
                
                minMaxValues[t][v].second = minMaxValues[t][v].second < Planner::NumericAnalysis::getBounds()[v].second ? minMaxValues[t][v].second : Planner::NumericAnalysis::getBounds()[v].second;
                
                //minMaxValues[t][v].second = minMaxValues[t][v].second < theState.second[v] ? theState.second[v] : minMaxValues[t][v].second;

            }
            
            // to calculate step
            double minC = 0;
            for (auto c : decreasers[v]){
                if (c < minC) minC = c;
            }
            minMaxSteps[t][v].first = minC;
            
            double maxC = 0;
            for (auto c : increasers[v]){
                if ( c > maxC) maxC = c;
            }
            minMaxSteps[t][v].second = maxC;
        }
    }
}

void FukunagaAnalysis::getActionsEffects(vector<list<pair<int,double>>> & simpleEffectActionsConstant, vector< list<pair<int,int>>> & linearEffectActionEffectID){
    
    simpleEffectActionsConstant.assign(RPGBuilder::getPNECount(),list<pair<int,double>>());
    linearEffectActionEffectID.assign(RPGBuilder::getPNECount(),list<pair<int,int>>());

    for (int iA = 0; iA < RPGBuilder::getNOp(); ++iA){
        list<int> & numEff = RPGBuilder::getActionsToRPGNumericStartEffects()[iA];
        for (int idEff : numEff){
            RPGBuilder::RPGNumericEffect & eff = RPGBuilder::getNumericEff()[idEff];
            int indexFluent = eff.fluentIndex;
            bool isAssignment = eff.isAssignment;
            vector<double> weights = eff.weights;
            vector<int> variables = eff.variables;
            double constant = eff.constant;
            bool isSimpleEffect = true;
            for (int i = 0; i<weights.size(); ++i){
                if (abs(weights[i]) < 0.0001) continue;
                if (variables[i]!=indexFluent) isSimpleEffect = false;
                else{
                    if (abs(weights[i]-1) > 0.0001 && !isAssignment) isSimpleEffect = false;
                }
            }
            if(isAssignment) isSimpleEffect = false;
            if (isSimpleEffect){
                simpleEffectActionsConstant[indexFluent].push_back({iA,constant});
            }else{
                linearEffectActionEffectID[indexFluent].push_back({iA,idEff});
            }
        }
    }
}

