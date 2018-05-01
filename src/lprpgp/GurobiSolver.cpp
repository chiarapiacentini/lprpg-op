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

#include "GurobiSolver.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

void GurobiSolver::initialise(){
    try {
        env = new GRBEnv();
        model = new GRBModel(*env);
        model->set(GRB_DoubleParam_Heuristics, 0.0);
        nSize = 10000000;
        x.x = new GRBVar[nSize];
        x.y = new GRBVar[nSize];
        x.v = new GRBVar[nSize];
        x.m = new GRBVar[nSize];
        countX = 0;
        countY = 0;
        countV = 0;
        countM = 0;
        nConstraints = 0;
    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
}

void GurobiSolver::addCol(VarType type, double min, double max, string name){
    switch (type)
    {
        case IPInteger:
        {
            x.m[countM++] = model->addVar(min, max, 0, GRB_INTEGER, name.c_str());
            break;
        }
        case IPBoolean:
        {
            x.x[countX++] = model->addVar(min, max, 0, GRB_BINARY, name.c_str());
            break;
        }
        case IPBooleanAux:
        {
            x.y[countY++] = model->addVar(min, max, 0, GRB_BINARY, name.c_str());
            break;
        }
        case IPContinous:
        {
            x.v[countV++] = model->addVar(min, max, 0, GRB_CONTINUOUS, name.c_str());
            break;
        }
        default:
        {
            // is likely to be an error
        }
    };
    
}

void GurobiSolver::addObjective(list<IPConstraint> coefficients){
    GRBLinExpr toSum = 0;
    for (auto c : coefficients){
        switch (c.varType)
        {
            case IPInteger:
            {
                toSum += c.coefficient*x.m[c.index];
                break;
            }
            case IPBoolean:
            {
                toSum+=(c.coefficient*x.x[c.index]);
                break;
            }
            case IPBooleanAux:
            {
                toSum+=(c.coefficient*x.y[c.index]);
                break;
            }
            case IPContinous:
            {
                toSum+=(c.coefficient*x.v[c.index]);
                break;
            }
            default:
            {
                // is likely to be an error
            }
        };
    }
    model->setObjective(toSum, GRB_MINIMIZE);
}


void GurobiSolver::addRow(list<IPSolver::IPConstraint> coefficients, EqType eqType, double constCoeff){
    GRBLinExpr toSum = 0;
    for (auto c : coefficients){
        switch (c.varType)
        {
            case IPInteger:
            {
                toSum+=(c.coefficient*x.m[c.index]);
                break;
            }
            case IPBoolean:
            {
                toSum+=(c.coefficient*x.x[c.index]);
                break;
            }
            case IPBooleanAux:
            {
                toSum+=(c.coefficient*x.y[c.index]);
                break;
            }
            case IPContinous:
            {
                toSum+=(c.coefficient*x.v[c.index]);
                break;
            }
            default:
            {
                // is likely to be an error
            }
        };
    }
    switch (eqType)
    {
        case IPGreater:
        {
            model->addConstr(toSum >= constCoeff);
            break;
        }
        case IPEqual:
        {
            model->addConstr(toSum == constCoeff);
            break;
        }
        case IPLess:
        {
            model->addConstr(toSum <= constCoeff);
            break;
        }
            
        default:
        {
            // is likely to be an error
        }
    };
    nConstraints++;
}

void GurobiSolver::addRow(int index, double w, VarType varType, EqType eqType, double constCoeff){
    GRBLinExpr toSum = 0;
    switch (varType)
    {
        case IPInteger:
        {
            toSum+=(w*x.m[index]);
      break;
        }
        case IPBoolean:
        {
            toSum+=(w*x.x[index]);
            break;
        }
        case IPBooleanAux:
        {
            toSum+=(w*x.y[index]);
            break;
        }
        case IPContinous:
        {
            toSum+=(w*x.v[index]);
            break;
        }
        default:
        {
            // is likely to be an error
        }
    };

    switch (eqType)
    {
        case IPGreater:
        {
            model->addConstr(toSum >= constCoeff);
            break;
        }
        case IPEqual:
        {
            model->addConstr(toSum == constCoeff);
            break;
        }
        case IPLess:
        {
            model->addConstr(toSum <= constCoeff);
            break;
        }
            
        default:
        {
            // is likely to be an error
        }
    };
 
    nConstraints++;
}

double GurobiSolver::getValue(VarType varType, int index){
    double toReturn;
    switch (varType)
    {
        case IPInteger:
        {
            toReturn = x.m[index].get(GRB_DoubleAttr_X);
            break;
        }
        case IPBoolean:
        {
            toReturn = x.x[index].get(GRB_DoubleAttr_X);
            break;
        }
        case IPBooleanAux:
        {
            toReturn = x.x[index].get(GRB_DoubleAttr_X);
            break;
        }
        case IPContinous:
        {
            toReturn = x.v[index].get(GRB_DoubleAttr_X);
            break;
        }
        default:
        {
            // is likely to be an error
        }
    }
    return toReturn;
}

bool GurobiSolver::solve(){
    static int idSolve = 1;
    stringstream name;
    name << "gurobi_model_" << setfill('0') << setw(5)<< idSolve << ".lp";
    if (debug) cout << "state expanded " << idSolve << endl;
    ++idSolve;
    if (debug)
    {
        cout << "exporting " << name.str().c_str() << endl;
        model->write(name.str().c_str());
    }
    if (!debug)  model->set(GRB_IntParam_OutputFlag, 0);
    model->set(GRB_IntParam_Threads,1);
    model->optimize();
    int status = model->get(GRB_IntAttr_Status);
    
    if (status == GRB_OPTIMAL){
        model->get(GRB_DoubleAttr_ObjVal);
    }
    return status == GRB_OPTIMAL;

}

void GurobiSolver::setLPRelaxation(bool b){
    if (b){
        for (int i = 0; i < nSize; ++i){
            x.x[i].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
            x.y[i].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
            x.m[i].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
        }
    } else {
        for (int i = 0; i < nSize; ++i){
            x.x[i].set(GRB_CharAttr_VType, GRB_BINARY);
            x.y[i].set(GRB_CharAttr_VType, GRB_BINARY);
            x.m[i].set(GRB_CharAttr_VType, GRB_INTEGER);
        }
    }
    
}

void GurobiSolver::updateCol(int index, VarType varType, BoundType bType, double value){
    switch (varType)
    {
        case IPInteger:
        {
            x.m[index].set(bType == LB ? GRB_DoubleAttr_LB : GRB_DoubleAttr_UB, value );
            break;
        }
        case IPBoolean:
        {
            x.x[index].set(bType == LB ? GRB_DoubleAttr_LB : GRB_DoubleAttr_UB, value );
            break;
        }
        case IPBooleanAux:
        {
            x.x[index].set(bType == LB ? GRB_DoubleAttr_LB : GRB_DoubleAttr_UB, value );
            break;
        }
        case IPContinous:
        {
            x.v[index].set(bType == LB ? GRB_DoubleAttr_LB : GRB_DoubleAttr_UB, value );
            break;
        }
        default:
        {
            // is likely to be an error
        }
    }
}

void GurobiSolver::eliminateConstraint(int id){
    GRBConstr *constraint = model->getConstrs();
    model->remove(constraint[id]);
    nConstraints--;
}

void GurobiSolver::eliminateObjective(){

}

int GurobiSolver::getNConstraint(){
    return nConstraints - 1;
}
