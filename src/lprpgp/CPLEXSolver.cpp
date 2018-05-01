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

#include "CPLEXSolver.hpp"

void CPLEXSolver::initialise(){
    IloEnv env;
    try {
        if (debug) cout << "initialising variables" << endl;
        model = IloModel(env);
        // initialise var
        x.y = IloNumVarArray(env);
        x.x = IloIntVarArray(env);
        x.v = IloNumVarArray(env);
        x.m = IloIntVarArray(env);
        constraints = IloConstraintArray(env);
    }catch (IloException& ex) {
        env.out() << "Error: " << ex << std::endl;
    }
}

void CPLEXSolver::addCol(VarType type, double min, double max, string name){
    switch (type)
    {
        case IPInteger:
        {
            x.m.add(IloBoolVar(model.getEnv(), min, max, name.c_str()));
            break;
        }
        case IPBoolean:
        {
            x.x.add(IloBoolVar(model.getEnv(), min, max, name.c_str()));
            break;
        }
        case IPBooleanAux:
        {
            x.y.add(IloNumVar(model.getEnv(), min, max, name.c_str()));
            break;
        }
        case IPContinous:
        {
            x.v.add(IloNumVar(model.getEnv(), min, max, name.c_str()));
            break;
        }
        default:
        {
            // is likely to be an error
        }
    };
}

void CPLEXSolver::addObjective(list<IPConstraint> coefficients){
    IloExpr toSum(model.getEnv());
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
    objective = IloMinimize(model.getEnv(),toSum);
    model.add(objective);
}


void CPLEXSolver::addRow(list<IPSolver::IPConstraint> coefficients, EqType eqType, double constCoeff){
    IloExprArray toSum(model.getEnv());
    for (auto c : coefficients){
        switch (c.varType)
        {
            case IPInteger:
            {
                toSum.add(c.coefficient*x.m[c.index]);
                break;
            }
            case IPBoolean:
            {
                toSum.add(c.coefficient*x.x[c.index]);
                break;
            }
            case IPBooleanAux:
            {
                toSum.add(c.coefficient*x.y[c.index]);
                break;
            }
            case IPContinous:
            {
                toSum.add(c.coefficient*x.v[c.index]);
                break;
            }
            default:
            {
                // is likely to be an error
            }
        };
    }
    IloConstraint constraint;
    switch (eqType)
    {
        case IPGreater:
        {
            constraint = IloSum(toSum) >= constCoeff;
            break;
        }
        case IPEqual:
        {
            constraint =  IloSum(toSum) == constCoeff;
            break;
        }
        case IPLess:
        {
            constraint =  IloSum(toSum) <= constCoeff;
            break;
        }
            
        default:
        {
            // is likely to be an error
        }
    };
    model.add(constraint);
    constraints.add(constraint);
    
}

void CPLEXSolver::addRow(int index, double w, VarType varType, EqType eqType, double constCoeff){
    IloExprArray toSum(model.getEnv());
    switch (varType)
    {
        case IPInteger:
        {
            toSum.add(w*x.m[index]);
            break;
        }
        case IPBoolean:
        {
            toSum.add(w*x.x[index]);
            break;
        }
        case IPBooleanAux:
        {
            toSum.add(w*x.y[index]);
            break;
        }
        case IPContinous:
        {
            toSum.add(w*x.v[index]);
            break;
        }
        default:
        {
            // is likely to be an error
        }
    };
    
    IloConstraint constraint;
    switch (eqType)
    {
        case IPGreater:
        {
            constraint = IloSum(toSum) >= constCoeff;
            break;
        }
        case IPEqual:
        {
            constraint =  IloSum(toSum) == constCoeff;
            break;
        }
        case IPLess:
        {
            constraint =  IloSum(toSum) <= constCoeff;
            break;
        }
            
        default:
        {
            // is likely to be an error
        }
    };
    model.add(constraint);
    constraints.add(constraint);
    
}

double CPLEXSolver::getValue(VarType varType, int index){
    double toReturn;
    switch (varType)
    {
        case IPInteger:
        {
            toReturn = cplex.getValue(x.m[index]);
            break;
        }
        case IPBoolean:
        {
            toReturn = cplex.getValue(x.x[index]);
            break;
        }
        case IPBooleanAux:
        {
            toReturn = cplex.getValue(x.y[index]);
            break;
        }
        case IPContinous:
        {
            toReturn = cplex.getValue(x.v[index]);
            break;
        }
        default:
        {
            // is likely to be an error
        }
    }
    return toReturn;
}

bool CPLEXSolver::solve(){
    static int idSolve = 1;
    stringstream name;
    name << "cplex_model_" << setfill('0') << setw(5)<< idSolve << ".lp";
    if (debug) cout << "state expanded " << idSolve << endl;
    ++idSolve;
    IloEnv env = model.getEnv();
    cplex.end();
    cplex = IloCplex(model);
    if (debug)
    {
        cout << "exporting " << name.str().c_str() << endl;
        cplex.exportModel(name.str().c_str());
    }
    if (!debug) cplex.setOut(env.getNullStream());
    cplex.setParam(IloCplex::Threads, 1);
    cplex.setParam(IloCplex::NumericalEmphasis, true);
    //cplex.setParam(IloCplex::EpInt, 0.00000001);
    cplex.solve();
    if (cplex.getStatus() == IloCplex::IloAlgorithm::Unknown || cplex.getStatus() == IloCplex::IloAlgorithm::Infeasible){
        //cplex.end();
        return false;
    }
    if (cplex.getStatus() == IloCplex::IloAlgorithm::Optimal){
        //cplex.end();
        return true;
    }
	//cplex.end();
    return false;
}

void CPLEXSolver::updateCol(int index, VarType varType, BoundType bType, double value){
    switch (varType)
    {
        case IPInteger:
        {
            bType == LB ? x.m[index].setLB(value) : x.m[index].setUB(value);
            break;
        }
        case IPBoolean:
        {
            bType == LB ? x.x[index].setLB(value) : x.x[index].setUB(value);
            break;
        }
        case IPBooleanAux:
        {
            bType == LB ? x.y[index].setLB(value) : x.y[index].setUB(value);
            break;
        }
        case IPContinous:
        {
            bType == LB ? x.v[index].setLB(value) : x.v[index].setUB(value);
            break;
        }
        default:
        {
            // is likely to be an error
        }
            
    };

}
void CPLEXSolver::setLPRelaxation(bool b){
    IloEnv env = model.getEnv();
    if (b){
        model.add(IloConversion(env, x.x, ILOFLOAT));
        model.add(IloConversion(env, x.y, ILOFLOAT));
        model.add(IloConversion(env, x.m, ILOFLOAT));
    } else {
        model.add(IloConversion(env, x.x, ILOINT));
        model.add(IloConversion(env, x.y, ILOINT));
        model.add(IloConversion(env, x.m, ILOINT));
    }
    
}


void CPLEXSolver::eliminateConstraint(int id){
    model.remove(constraints[id]);
}

void CPLEXSolver::eliminateObjective(){
    model.remove(objective);
}

int CPLEXSolver::getNConstraint(){
    return constraints.getSize() - 1;
}
