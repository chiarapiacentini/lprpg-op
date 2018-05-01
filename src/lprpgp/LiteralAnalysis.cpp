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

#include "LiteralAnalysis.h"

#include "RPGBuilder.h"

#include "PreferenceHandler.h"

#include <sstream>
using std::endl;

namespace Planner {
  
vector<pair<bool,bool> > LiteralAnalysis::staticLiterals;
    
const pair<bool,bool> & LiteralAnalysis::isStatic(const Literal * const l) {
    return staticLiterals[l->getGlobalID()];
}

void LiteralAnalysis::getBasicStaticInformationFromTIM()
{
    static const int litCount = instantiatedOp::howManyLiteralsOfAnySort();
    
    staticLiterals.resize(litCount, pair<bool,bool>(false,false));
    
    LiteralStore::iterator lsItr = instantiatedOp::literalsBegin();
    const LiteralStore::iterator lsEnd = instantiatedOp::literalsEnd();
    
    for (int i = 0; lsItr != lsEnd; ++lsItr, ++i) {
        if (EPS((*lsItr)->getHead())->isStatic()) {
            staticLiterals[i].first = true;
            if (EPS((*lsItr)->getHead())->getInitials()->get((*lsItr)->getEnv(),(*lsItr)->getProp())) {
                staticLiterals[i].second = true;
            } else {
                staticLiterals[i].second = false;
            }
        }
    }
}


void LiteralAnalysis::findStaticLiterals() {

    LiteralSet initialState;
   	bool debug = false; 
    
    RPGBuilder::getAllInitialLiterals(initialState);

    const vector<list<pair<int, VAL::time_spec> > > & eta = RPGBuilder::getEffectsToActions();
    const vector<list<pair<int, VAL::time_spec> > > & neta = RPGBuilder::getNegativeEffectsToActions();
    
    const int litCount = eta.size();   

	//Checks if literal is the effects of an action
    {
        LiteralStore::iterator lsItr = instantiatedOp::literalsBegin();
        const LiteralStore::iterator lsEnd = instantiatedOp::literalsEnd();
            
        
        if (debug) cout << "Static literals:" << endl;
        
        int fID;
        for (int gID = 0; gID < litCount; ++gID, ++lsItr) {
            fID = (*lsItr)->getStateID();
            //NEW WAY
            if(fID == -1){
            	staticLiterals[gID] = pair<bool,bool>(true,initialState.find(*lsItr) != initialState.end());
            }
            else{
            	bool in_initial_state = initialState.find(*lsItr) != initialState.end();
            	if(eta[fID].empty() && neta[fID].empty() && !in_initial_state) {
            		staticLiterals[gID] = pair<bool,bool>(true,in_initial_state);
            		RPGBuilder::preconditionsToActions[fID].clear();
                    RPGBuilder::processedPreconditionsToActions[fID].clear();
                     if (debug) {
                    	cout << "\t" << fID << *(RPGBuilder::getLiteral(fID)) << "; Initial State: " <<  staticLiterals[gID].second << endl;
                    }
            	}
            	else{
            		staticLiterals[gID] = pair<bool,bool>(false,false);
            	}
            }
            
            
            /* //OLD WAY
            if (fID == -1 || (eta[fID].empty() && neta[fID].empty())) {
                staticLiterals[gID] = pair<bool,bool>(true,initialState.find(*lsItr) != initialState.end());
               

                if (fID != -1) {
                    RPGBuilder::preconditionsToActions[fID].clear();
                    RPGBuilder::processedPreconditionsToActions[fID].clear();
                    if (debug) {
                    	cout << "\t" << fID << *(RPGBuilder::getLiteral(fID)) << "; Initial State: " <<  staticLiterals[gID].second << endl;
                    }
                }
            } else {
                staticLiterals[gID] = pair<bool,bool>(false,false);
            }
            */
        }
    }

    vector<list<RPGBuilder::ProtoConditionalEffect*> > & actionsToRawConditionalEffects = RPGBuilder::actionsToRawConditionalEffects;
    
    const int actCount = actionsToRawConditionalEffects.size();
    
    for (int act = 0; act < actCount; ++act) {
        {
            list<RPGBuilder::ProtoConditionalEffect*>::const_iterator ceItr = actionsToRawConditionalEffects[act].begin();
            const list<RPGBuilder::ProtoConditionalEffect*>::const_iterator ceEnd = actionsToRawConditionalEffects[act].end();
            for (; ceItr != ceEnd; ++ceItr) {
                for (int pass = 0; pass < 2; ++pass) {
                    list<Literal*> & currList = (pass ? (*ceItr)->startAddEff : (*ceItr)->startDelEff);
                    {
                        list<Literal*>::const_iterator factItr = currList.begin();
                        const list<Literal*>::const_iterator factEnd = currList.end();
                        for (; factItr != factEnd; ++factItr) {
                            staticLiterals[(*factItr)->getGlobalID()] = pair<bool,bool>(false,false);
                        }
                    }
                }
            }
        }
    }
    

}

bool LiteralAnalysis::pruneStaticPreconditions(list<Literal*> & toPrune, int & toDec) {
    
    bool toReturn = true;
    list<Literal*>::iterator tpItr = toPrune.begin();
    const list<Literal*>::iterator tpEnd = toPrune.end();

    while (tpItr != tpEnd) {
        const pair<bool,bool> & info = isStatic(*tpItr);
        if (info.first) {
            toReturn = (toReturn && info.second);
            list<Literal*>::iterator tpPrev = tpItr;
            ++tpItr;
            toPrune.erase(tpPrev);
            --toDec;
        } else {
            ++tpItr;
        }
    }
    
    return toReturn;
}

void LiteralAnalysis::pruneStaticPreconditions() {

    const int opCount = RPGBuilder::instantiatedOps.size();
    for (int i = 0; i < opCount; ++i) {
        if (!RPGBuilder::rogueActions[i]) {
            bool applicable = true;
            {
                int & toDec = RPGBuilder::initialUnsatisfiedStartPreconditions[i];
                applicable = (applicable && pruneStaticPreconditions(RPGBuilder::actionsToStartPreconditions[i], toDec));
            }
            {
                int & toDec = RPGBuilder::initialUnsatisfiedProcessedStartPreconditions[i];
                const int before = toDec;
                applicable = (applicable && pruneStaticPreconditions(RPGBuilder::actionsToProcessedStartPreconditions[i], toDec));
                if (!toDec && before) {
                    RPGBuilder::preconditionlessActions.push_back(pair<int, VAL::time_spec>(i, VAL::E_AT_START));
                }
            }
            if (!applicable) {
                RPGBuilder::pruneIrrelevant(i);
            }
        }
    }

    PreferenceHandler::pruneStaticLiterals(staticLiterals);
    
    vector<list<RPGBuilder::ProtoConditionalEffect*> > & actionsToRawConditionalEffects = RPGBuilder::actionsToRawConditionalEffects;
    
    for (int act = 0; act < opCount; ++act) {
        {
            list<RPGBuilder::ProtoConditionalEffect*>::const_iterator ceItr = actionsToRawConditionalEffects[act].begin();
            const list<RPGBuilder::ProtoConditionalEffect*>::const_iterator ceEnd = actionsToRawConditionalEffects[act].end();
            for (; ceItr != ceEnd; ++ceItr) {
                list<Literal*> & currList = (*ceItr)->startPrec;
            
                bool hasNull = false;
                bool needsNull = false;
                
                list<Literal*>::iterator factItr = currList.begin();
                const list<Literal*>::iterator factEnd = currList.end();
                while (factItr != factEnd) {
                    if (!(*factItr)) {
                        ++factItr;
                        hasNull = true;
                        continue; // equality where (a != b) introduces a 0 to signify 'false'
                    }
                    
                    const pair<bool,bool> & s = isStatic(*factItr);
                    if (s.first) {
                        if (!s.second) {
                            needsNull = true;                            
                        }
                        list<Literal*>::iterator factDel = factItr++;
                        currList.erase(factDel);                                                                            
                    } else {
                        ++factItr;
                    }
                }
                
                if (needsNull && !hasNull) {
                    currList.push_back((Literal*)0);
                }
            }
        }
    }

};

    
};
