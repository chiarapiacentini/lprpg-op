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

#ifndef GLOBALS_H
#define GLOBALS_H

#include <instantiation.h>

using Inst::Literal;

#include <set>
#include <list>
#include <utility>
#include <string>

using std::set;
using std::list;
using std::pair;
using std::string;

enum AutomatonPosition { satisfied = 0, unsatisfied = 1, triggered = 2, unreachable = 3, eternallysatisfied = 4, seenoncealreadyandstillholds = 5, seenoncealready = 6};

extern const char * positionName[7];

#define BIG 100000.0
#define SMALL 0.001


struct AddingConstraints {
    
    set<int> addingWillViolate;    
    list<pair<int,bool> > extraGoalsToAvoidViolations;
    
};

template<typename T>
class LiteralCellDependency {
    
public:
    T dest;
    int index;
    bool init;
    LiteralCellDependency(const T & d, const int & i) : dest(d), index(i), init(true) {};
    LiteralCellDependency() : init(false) {};
        
};


struct PreferenceSetAndCost {
    set<int> needsToViolate;
    double cost;
    
    int achiever;
    double atLayer;
    
    PreferenceSetAndCost(const bool satisfied=false)
        : cost(0.0), achiever(-1), atLayer(satisfied ? 0.0 : -1.0)
    {
    }
                                
    PreferenceSetAndCost(const int & currAct, const double & factLayerTime,
                         const double & ncnum, const set<int> & ncset)
        : needsToViolate(ncset), cost(ncnum), achiever(currAct), atLayer(factLayerTime)
    {
    }
};
    
struct LiteralLT {
    
    bool operator()(const Literal* const & a, const Literal* const & b) const  {
        if (a && b) {
            return (a->getGlobalID() < b->getGlobalID());
        } else {
            if (!a && b) return true;
            return false;
        }
    }

};

typedef set<Literal*, LiteralLT> LiteralSet;


#endif

