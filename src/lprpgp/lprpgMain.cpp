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

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <ptree.h>
#include <assert.h>
#include <FlexLexer.h>
#include "instantiation.h"
#include "SimpleEval.h"
#include "DebugWriteController.h"
#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"
#include "DotSearchSpace.h"

//#include "graphconstruct.h"
#include "RPGBuilder.h"
//#include "PartialPlan.h"
#include "Decomposition.h"
#include "FFSolver.h"
#include "GlobalSchedule.h"
#include "PreferenceHandler.h"

#include <sys/times.h>
#include <chrono>
#include <ctime>

#include "MILPRPG.h"
#include "solver.h"
#include "IPCompilation.hpp"
#include <algorithm>

#ifdef TESTCPLEX

#endif

using std::ifstream;
using std::cerr;

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace Planner;

namespace VAL {
bool ContinueAnyway;
bool ErrorReport;
bool InvariantWarnings;
bool LaTeX;
//bool LaTeXRecord;
bool makespanDefault;
};

void usage()
{
        cout << "LPRPG\n";
        cout << "By releasing this code we imply no warranty as to its reliability\n";
        cout << "and its use is entirely at your own risk.\n\n";
        cout << "Usage: lprpg [OPTIONS] domainfile problemfile\n\n";
        cout << "Options are: \n\n";
	cout << "\t" << "-citation" << "\t" << "Display citation to relevant conference papers;\n";
	cout << "\t" << "-plananyway" << "\t" << "Force planning, even on problems with non-producer-consumer numerics;\n";
    cout << "\t" << "-y[options]" << "\t" << "Use A* search and h^{c}_{IP} heuristic;\n";
    cout << "\t\t" << "options:\n";
    cout << "\t\t" << "l" << "\t" << "Use lp relaxation;\n";
    cout << "\t\t" << "m" << "\t" << "Use Compilation (State-Based model);\n";
    cout << "\t\t" << "n" << "\t" << "Use Compilation (State-Change model);\n";
    cout << "\t" << "-L" << "\t" << "disable landmarks;\n";
    cout << "\t" << "-d[options]" << "\t" << "disable/enable some constraints;\n";
    cout << "\t\t" << "t" << "\t" << "disable sequencing constraints;\n";
    cout << "\t\t" << "e" << "\t" << "use constraints in h^{e};\n";
    cout << "\t\t" << "n" << "\t" << "use constraints in h^{ir};\n";
    cout << "\n *** options from LPGRP *** \n\n";
    cout << "\t" << "-e" << "\t" << "Use standard EHC instead of steepest descent;\n";
	cout << "\t" << "-E" << "\t" << "Skip HC - go straight to best-first search;\n";
    cout << "\t" << "-K" << "\t" << "Apply action elimination to each new best solution found;\n";
    cout << "\t" << "-G" << "\t" << "Don't put the numeric goal conjunct formula into the LP;\n";
    cout << "\t" << "-P" << "\t" << "Don't put the propositional goal formula in the LP;\n";    
    cout << "\t" << "-L" << "\t" << "Don't put propositional landmarks in the LP;\n";    
    cout << "\t" << "-I<n>" << "\t" << "Control use of parameters learnt by ParamILS:\n";
    cout << "\t\t\t\t0 - do not set parameters\n";
    cout << "\t\t\t\t1-4 - use a hardwired parameter set (default: 1)\n";
    cout << "\t" << "-w<n>" << "\t" << "Control the extent to which integers are used in the LP (from 0 to 4);\n";
    cout << "\t" << "-f<n>" << "\t" << "Set the factor used to generate the geometric series for weighting actions in the LP;\n";
    cout << "\t" << "-F[max|sum]" << "\t" << "Use hmax/hadd cost propagation in the RPG;\n";
    cout << "\t" << "-r" << "\t" << "Only force the LP to wholly satisfy infrastructure numeric preconditions, rather than wholly satisfying all preconditions;\n";
    cout << "\t" << "-x" << "\t" << "Never use presolving in the LP;\n";
    cout << "\t" << "-B" << "\t" << "Do not recognise infrastructure propositional preconditions in the LP;\n";
    cout << "\t" << "-p" << "\t" << "Add all propositional preconditions and effects to the LP;\n";
    cout << "\t" << "-U" << "\t" << "Disable LP warm-starting based on sensible initial column values;\n";    
	cout << "\t" << "-h" << "\t" << "Disable helpful-action pruning.\n\n";
    
};

int readAndTestPlan(const char *, const bool &);

void printCitation(){
    cout << "@CONFERENCE{piacentiniaaai18,\n";
    cout << "\tauthor = \"C. Piacentini, M. P. Castro, A. A. Cire, J. C. Beck\",\n";
    cout << "\ttitle = \"Linear and Integer Programming-based Heuristics for Cost-optimal Numeric Planning\",\n";
    cout << "\tbooktitle = \"Proceedings of the Thirty-Second AAAI Conference on Artificial Intelligence (AAAI-18)\",\n";
    cout << "\tyear = \"2018\",\n";
    cout << "\tmonth = \"Febrauary\",\n";
    cout << "}\n";
    cout << "@CONFERENCE{lprpgicaps08,\n";
    cout << "\tauthor = \"A. I. Coles and M. Fox and D. Long and A. J. Smith\",\n";
    cout << "\ttitle = \"A Hybrid Relaxed Planning Graph-LP Heuristic for Numeric Planning Domains\",\n";
    cout << "\tbooktitle = \"Proceedings of the Eighteenth International Conference on Automated Planning and Scheduling (ICAPS 08)\",\n";
    cout << "\tyear = \"2008\",\n";
    cout << "\tmonth = \"September\",\n";
    cout << "}\n";
}

int main(int argc,char * argv[])
{

	int argcount = 1;

	FF::ignorePenalties = true;
	FF::initialAdvancing = false;
	FF::disableWaits = true;
	FF::neverReuseOrWait = true;
	FF::steepestDescent = false;
	GlobalSchedule::checkOpenStartsOnly = true;
        FF::incrementalExpansion = false;
        FF::justWaitIncr = true;
	FF::invariantRPG = false;
	FF::timeWAStar = false;
    MILPRPG::residualEverything = true;
    bool writeSearch = false;
    bool writeFluents = false;
    string nameOuputFluents = "";
    RPGBuilder::ipSolver = CPLEX;
    
    while(argcount < argc && argv[argcount][0] == '-') {

		string remainder(&(argv[argcount][1]));
		if (remainder == "citation") {
            printCitation();
		} else if (remainder == "plananyway") {
			RPGBuilder::planAnyway = true;
		} else {

	
		switch(argv[argcount][1]) {
            case '1':
                FF::allowDualOpenList = false;
                break;
            case '2': {
                RPGBuilder::switchInCheaperAchievers = false;
                break;
            }
            case 'a':
                FF::initialAdvancing = false;
                break;
            case 'b':
                FF::bestFirstSearch = false;
                break;
            case 'e':
                FF::steepestDescent = true;
                break;
            case 'E':
                FF::skipEHC = true;
                break;
            case 'c':
            	if(argv[argcount][2] == 'p'){
            		FF::AStar = true;
            		FF::skipEHC = true;
            		FF::multipleHelpfuls = false;
            		
            		//TODO: variable for cost partition
            		break;
            	}
            
                FF::disablePareto = 2;
                break;
            case 'C':
                FF::disablePareto = 1;
                break;
            case 'f':
            {
                if(argv[argcount][2] == 0){
                    usage();
                    exit(1);
                }
                MILPRPG::layerFactor = atof(&(argv[argcount][2]));                    
                break;
            }                
            case 'F':
            {
                if(argv[argcount][2] == 0){
                    usage();
                    exit(1);
                }
                
                if (!(strcmp(&(argv[argcount][2]), "sum"))) {
                    RPGBuilder::costPropagationMethod = E_SUMCOST;
                } else if (!(strcmp(&(argv[argcount][2]), "max"))) {
                    RPGBuilder::costPropagationMethod = E_MAXCOST;
                } else {
                    cout << "Unknown cost propagation method '" << &(argv[argcount][2]) << "', should be sum or max.\n";
                    usage();
                    exit(1);
                }
                RPGBuilder::useTheCostsInTheLP = true;
                                
                break;
            }
            case 'g': {
                {
                    if(argv[argcount][2] != 0){
                        MILPRPG::secondaryIntegerLevel = atoi(&(argv[argcount][2]));
                    } else {                    
                        MILPRPG::secondaryIntegerLevel = 1;
                    }
                }
                break;
            }
            case 'h':
                FF::helpfulActions = false;
                break;
            case 'i':{
                FF::firstImprover = true;
                break;
            }
            case 'D':
                PreferenceHandler::preferenceDebug = true;
                break;
            case 'P':
                MILPRPG::addProps = false;
                break;
            case 'L':
                MILPRPG::addLandmarks = false;
                break;
            case 'l':
                MILPRPG::useLocalLandmarks = true;
                break;
            case 'G':
                MILPRPG::addNumGoalConjunct = false;
                break;
            case 'A':
                MILPRPG::alternativeObjective = true;
                break;
            case 'B':
                MILPRPG::recogniseBootstrappingPropositions = false;
                break;
            case 'j':
                FF::relaxedGoalJump = false;
                break;
            case 'm': {
                RPGBuilder::useMetricRPG = true;
                MILPRPG::addNumGoalConjunct = false;
                break;
            }
            case 'M':
                FF::multipleHelpfuls = false;
                break;
            case 'n':
                FF::ignorePenalties = false;
                break;
            case 'O':
                FF::startsBeforeEnds = false;
                break;
            case 'y': {
                RPGBuilder::useMIP = true;
                RPGBuilder::useDeleteRelaxation = false;
                RPGBuilder::useTemporalRelaxation = false;
                RPGBuilder::useBoxing = true;
                RPGBuilder::useValidityInequality = false;
                FF::AStar = true;
                FF::skipEHC = true;
                FF::multipleHelpfuls = false;
                RPGBuilder::useStartPlanLength = false;
                RPGBuilder::blindSearch = false;
                RPGBuilder::useSEQConstraints = true;
                RPGBuilder::useExtraConstraints = false;
                bool breakOption = false;
                for (int index=2; index<10; index++){
                    switch (argv[argcount][index]){
                        case 'l' : {
                            RPGBuilder::useLPRelaxation = true;
                            break;
                        }
                        case '0' : {
                            RPGBuilder::blindSearch = true;
                            break;
                        }
                        case 'b' : {
                            Compilation::solveIPCompilation = true;
                            Compilation::numericType = Simple;
                            Compilation::modelType = StateBased;
                            if (argv[argcount][index+1] != 0) {
                                Compilation::horizon = atoi(&(argv[argcount][index+1]));
                            }else{
                                Compilation::horizon = -1;
                            }
                            break;
                        }
                        case 'm' : {
                            Compilation::solveIPCompilation = true;
                            Compilation::numericType = Linear;
                            Compilation::modelType = StateBased;
                            if (argv[argcount][index+1] != 0) {
                                Compilation::horizon = atoi(&(argv[argcount][index+1]));
                            }else{
                                Compilation::horizon = -1;
                            }
                            break;
                        }
                        case 'c' : {
                            Compilation::solveIPCompilation = true;
                            Compilation::modelType = StateChange;
                            Compilation::numericType = Simple;
                            if (argv[argcount][index+1] != 0) {
                                Compilation::horizon = atoi(&(argv[argcount][index+1]));
                            }else{
                                Compilation::horizon = -1;
                            }
                            break;
                        }
                        case 'n' : {
                            Compilation::solveIPCompilation = true;
                            Compilation::modelType = StateChange;
                            Compilation::numericType = Linear;
                            if (argv[argcount][index+1] != 0) {
                                Compilation::horizon = atoi(&(argv[argcount][index+1]));
                            }else{
                                Compilation::horizon = -1;
                            }
                            break;
                        }
                        default  : {
                            breakOption = true;
                            break;
                        }
                    }
                    if(breakOption)
                        break;
                }
                break;
            }
            case '0': {
                RPGBuilder::usePlanLength = true;
                break;
            }
            case 'N': {
                string solver = &(argv[argcount][2]);
                transform(solver.begin(), solver.end(), solver.begin(), ::tolower);
                if (solver == "cplex"){
                    RPGBuilder::ipSolver = Planner::CPLEX;
                }else if (solver == "gurobi") {
                    RPGBuilder::ipSolver = Planner::GUROBI;
                }
                break;
            }
            case 'd': {
                RPGBuilder::useExtraConstraints = false;
                bool breakOption = false;
                for (int index=2; index<10; index++) {
                    switch (argv[argcount][index]) {
                        case 't' : {
                            RPGBuilder::useTemporalRelaxation = true;
                            break;
                        }
                        case 's' : {
                            RPGBuilder::useSEQConstraints = true;
                            RPGBuilder::useExtraConstraints = false;
                            break;
                        }
                        case 'e' : {
                            RPGBuilder::useSEQConstraints = false;
                            RPGBuilder::useExtraConstraints = true;
                            RPGBuilder::useBoxing = false;
                            break;
                        }
                        case 'd' : {
                            RPGBuilder::useExtraConstraints = true;
                            RPGBuilder::useSEQConstraints = false;
                            RPGBuilder::useBoxing = true;
                            break;
                        }
                        case 'n' : {
                            RPGBuilder::useExtraConstraints = false;
                            RPGBuilder::useSEQConstraints = false;
                            MILPRPG::addLandmarks = false;
                            RPGBuilder::useDeleteRelaxation = true;
                            break;
                        }
                        case 'i' : {
                            RPGBuilder::useValidityInequality = true;
                            break;
                        }
                        case 'p' : {
                            RPGBuilder::useStartPlanLength = true;
                            break;
                        }
                        default  : {
                            breakOption = true;
                            break;
                        }
                    }
                    if(breakOption)
                        break;
                }
                break;
            }
            case 'u' : {
                bool breakOption = false;
                RPGBuilder::compareMDDMIP = true;

                for (int index=2; index<10; index++){
                    switch (argv[argcount][index]){
                        default  : {
                            RPGBuilder::compareMDDMIP = true;
                            breakOption = true;
                            if (argv[argcount][index]!=0){
                                RPGBuilder::nameFileCompare = &(argv[argcount][index]);
                            }else{
                                RPGBuilder::nameFileCompare  = "compareMIPMDD.csv";
                            }
                            cout << RPGBuilder::nameFileCompare << endl;
                            break;
                        }
                    }
                    if(breakOption)
                        break;
                }
                break;
            }
            case 'p':
                MILPRPG::ensureAllPropositionalPreconditionsAreMet = true;
                break;
            case 'R':
                MILPRPG::residualEverything = true;
                break;
            case 'S':
                MILPRPG::useSecondaryObjective = true;
                break;
            case 's':
                RPGBuilder::doNotApplyActionsThatLookTooExpensive = false;
                break;
            case 'T':
                FF::tsChecking = false;
                FF::invariantRPG = false;
                break;
            case 'U': {
                useLPWarmStart = false;
                break;
            }
            case 'v':
                {
                    if(argv[argcount][2] != 0){
                        GlobalSchedule::globalVerbosity = atoi(&(argv[argcount][2]));
                    } else {
                        GlobalSchedule::globalVerbosity = 1;
                    }
                }
                break;
            case 'V':
                {
                    if(argv[argcount][2] == 0){
                        usage();
                        exit(1);
                    }
                    FF::capOnPreferenceCost = atof(&(argv[argcount][2]));
                    break;
                }
            case 'w':
            {
                if(argv[argcount][2] == 0){
                    usage();
                    exit(1);
                }
                MILPRPG::integerLevel= atoi(&(argv[argcount][2]));
                break;
            }
            case 'W':
                FF::doubleU = atof(&(argv[argcount][2]));
                break;
            case 'x':
            {
                MILPRPG::neverUsePresolving = true;                    
                break;
            }                
            case 'X': {
                RPGBuilder::postFilter = false;
                break;
            }
            case 'I':
                {
                    if(argv[argcount][2] != 0){
                        MILPRPG::useParamILS = atoi(&(argv[argcount][2]));
                    } else {                    
                        MILPRPG::useParamILS = 1;
                    }
                }
                break;
            case 'Y': {
                FF::allowDualOpenList = false;
                FF::useWeightedSumWithPrefCost = true;                
                if(argv[argcount][2] != 0){
                    FF::prefWeightInWeightedSum = atof(&(argv[argcount][2]));
                }
                break;
            }
            case 'H':
                {
                    readParams(argv, argcount);
                    //paramILS(argv, argcount);
                    ++argcount;
                }
                break;
            #ifdef FFSEARCHDEBUGHOOKS
            case 'z':
                {
                    FF::actuallyPlanGivenPreviousSolution = true;
                    break;
                }
            #endif
            case 'J' : {
                writeSearch = true;
                bool breakOption = false;
                for (int index=2; index<10; index++){
                    switch (argv[argcount][index]){
                        case '1' : {
                            DotSearchSpace::addColor=true;
                            break;
                        }
                        case '2' : {
                            DotSearchSpace::addEdge=true;
                            break;
                        }
                        case '3' : {
                            DotSearchSpace::addHeuristic=true;
                            break;
                        }
                        case '4' : {
                            writeFluents = true;
                            DotSearchSpace::writeFluents = true;
                            break;
                        }
                        default  : {
                            breakOption = true;
                            if (argv[argcount][index]!=0){
                                DotSearchSpace::nameFile = &(argv[argcount][index]);
                                nameOuputFluents=DotSearchSpace::nameFile+".csv";
                            }else{
                                DotSearchSpace::nameFile = "search.dot";
                                nameOuputFluents="search.csv";
                            }
                            break;
                        }
                    }
                    if(breakOption)
                        break;
                }
                break;
            }

		default:
			cout << "Unrecognised command-line switch '" << argv[argcount][1] << "'\n";
			usage();
			exit(0);
	
		};
		}
		++argcount;
	};

	if (argcount + 2 > argc) {
        if(argcount < argc && argv[argcount][0] == '-') {
            
            string remainder(&(argv[argcount][1]));
            if (remainder == "citation") {
                printCitation();
                exit(0);
            }
        }
		usage();
		exit(0);
	}

	performTIMAnalysis(&argv[argcount]);

    argcount += 2;
            
    const char * solutionFile = 0;
    
    if (argcount < argc) {
        if (argv[argcount][0] != '-') {
            solutionFile = argv[argcount];
            ++argcount;
        }
        
        #ifndef NDEBUG
        GlobalSchedule::planFilename = solutionFile;
        #endif
    }
    
    if (argcount < argc) {
        readParams(&argv[argcount], argc - argcount);
    }

	// start counting time here!!
	chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

	cout << std::setprecision(3) << std::fixed;

    RPGBuilder::initialise(Compilation::solveIPCompilation);

	if (RPGBuilder::getTILVec().size() > 0) {
		cout << "Timed initial literals detected, using extended temporal RPG analysis\n";
		FF::invariantRPG = true;
		FF::timeWAStar = true;
	}

    /***
        @chiarap: there is always one subproblem in performDummyDecomposition
     */
    
	Decomposition::performDummyDecomposition();
	GlobalSchedule::initialise();
	const int spCount = Decomposition::howMany();
	bool carryOn = true;
	
    #ifndef FFSEARCHDEBUGHOOKS    
    if (solutionFile) {
        return readAndTestPlan(solutionFile, false);
    }
    #else
    if (solutionFile) {
        if (FF::actuallyPlanGivenPreviousSolution) {
            const int rv = readAndTestPlan(solutionFile, true);
            if (rv) return rv;
        } else {
            return readAndTestPlan(solutionFile, false);
        }
    }
    #endif

    // @chiarap: add here IP compilation
    if (Compilation::solveIPCompilation){
        cout << "IPC " << Compilation::solveIPCompilation << " " << endl;
        cout << Compilation::horizon<<endl;
        Compilation::solveModel(Compilation::horizon,Compilation::modelType,Compilation::numericType);
        return 0;
    }
    
	while (carryOn) {
	
		carryOn = false;
//		bool reachesAllGoals = true;
		
		for (int i = 0; i < spCount; ++i) {
			if (GlobalSchedule::globalVerbosity & 1) cout << "Planning for subproblem " << i << "\n";
			//list<FFEvent> * const oldSoln = GlobalSchedule::getCurrentManipulator()->reset(i);
			//double toBeat = (oldSoln ? GlobalSchedule::currentEvaluator->heuristicPenaltiesOldSchedule(*GlobalSchedule::currentSchedule, i, true) : 0.0);
            
            list<FFEvent> * const oldSoln = 0;
            double toBeat = 0.0;
			bool solved = false;
            list<FFEvent> * spSoln;
			{
				
                bool reachesGoals;
                int relaxedStepCount = 0;
                
                LiteralSet initialState;
                vector<double> initialFluents;
                
                RPGBuilder::getInitialState(initialState, initialFluents);
                
                spSoln = FF::solveSubproblemWRTSchedule(initialState, initialFluents, Decomposition::getSubproblem(i), INT_MAX, reachesGoals, i, oldSoln, toBeat, relaxedStepCount);
                if (spSoln) {
                    solved = true;
                }
				
			}
			if (solved) {			
				cout << ";;;; Solution Found\n";
                cout << "; States evaluated: " << RPGBuilder::statesEvaluated  << "\n";
                cout << "; States expanded: " << FF::statesExpanded  << "\n";
                cout << "; Plan Cost : " <<  FF::costPlan << "\n;\n";
                
                end = std::chrono::system_clock::now();
 				std::chrono::duration<double> elapsed_seconds = end-start;
                double secs = elapsed_seconds.count();
                //double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
                

				int twodp = (int) (secs * 100.0);
				int wholesecs = twodp / 100;
				int centisecs = twodp % 100;
		
				cout << "; Time : " << wholesecs << ".";
				if (centisecs < 10) cout << "0";
				cout << centisecs << "\n";
				
                if (RPGBuilder::useMIP){
                    cout << "; Time building     : " << Planner::SubproblemMIP::timeBuildConstraints/1000000. << "\n";
                    cout << "; Time solving      : " << Planner::SubproblemMIP::timeSolve/1000000. << "\n";

                }
                cout << ";\n";
                list<FFEvent>::iterator planItr = spSoln->begin();
                const list<FFEvent>::iterator planEnd = spSoln->end();
                
                for (int i = 0; planItr != planEnd; ++planItr) {            
                    if (planItr->time_spec == VAL::E_AT_START) {
                    	//getid of the action
						int id = planItr->action->getID();
                        cout << (i * 1.001) << ": " << *(planItr->action) << " ["<< RPGBuilder::getActionCost()[id] << "]\n";
                        ++i;
                    }
                }
                if(writeSearch){
                    DotSearchSpace::printDotFile();
                    DotSearchSpace::printGnuplotFile();
                }
				return 0;
			} else {
				cout << ";; Problem unsolvable!\n";
				//tms refReturn;
                //times(&refReturn);				
				cout << ";;;; Solution Found\n";
				end = std::chrono::system_clock::now();
 				std::chrono::duration<double> elapsed_seconds = end-start;
                double secs = elapsed_seconds.count();
                //double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));

				int twodp = (int) (secs * 100.0);
				int wholesecs = twodp / 100;
				int centisecs = twodp % 100;
		
				cout << "; Time " << wholesecs << ".";
				if (centisecs < 10) cout << "0";
				cout << centisecs << "\n";
                if(writeSearch){
                    DotSearchSpace::printDotFile();
                    DotSearchSpace::printGnuplotFile();
                }
				return 1;
			}
	
		} 
		
	}
	
	//tms refReturn;
	//times(&refReturn);				
	cout << ";;;; Solution Found\n";
	//double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));
	end = std::chrono::system_clock::now();
 	std::chrono::duration<double> elapsed_seconds = end-start;
    double secs = elapsed_seconds.count();

	int twodp = (int) (secs * 100.0);
	int wholesecs = twodp / 100;
	int centisecs = twodp % 100;

	cout << "; Time " << wholesecs << ".";
	if (centisecs < 10) cout << "0";
	cout << centisecs << "\n";

	return 0;
}

extern int yyparse();
extern int yydebug;

namespace VAL {
    extern yyFlexLexer* yfl;
};

int readAndTestPlan(const char * filename, const bool & prime) {
    
    ifstream * const current_in_stream = new ifstream(filename);
    if (!current_in_stream->good())
    {
        cout << "Exiting: could not open plan file " << filename << "\n";
        exit(1);
    }
        
    VAL::yfl = new yyFlexLexer(current_in_stream,&cout);
    yyparse();
    
    VAL::plan * const the_plan = dynamic_cast<VAL::plan*>(top_thing);
    
    delete VAL::yfl;
    delete current_in_stream;
    
    
    
    if(!the_plan)
    {
        cout << "Exiting: failed to load plan " << filename << "\n";
        exit(1);
    };
    
    if (!theTC->typecheckPlan(the_plan)) {
        cout << "Exiting: error when type-checking plan " << filename << "\n";
        exit(1);
    }
    
    list<int> planSteps;
    
    pc_list<plan_step*>::const_iterator planItr = the_plan->begin();
    const pc_list<plan_step*>::const_iterator planEnd = the_plan->end();
    
    for (int idebug = 0, i=0; planItr != planEnd; ++planItr, ++i, ++idebug) {
        plan_step* const currStep = *planItr;
        
        instantiatedOp * const currOp = instantiatedOp::findInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
        
        if (!currOp) {
            //instantiatedOp * const debugOp = instantiatedOp::getInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
            cout << "Exiting: step " << idebug << " in the input plan uses the action ";
            cout << "(" << currStep->op_sym->getName();
            
            const_symbol_list::const_iterator cslItr = currStep->params->begin();
            const const_symbol_list::const_iterator cslEnd = currStep->params->end();
            for (; cslItr != cslEnd; ++cslItr) {
                cout << " " << (*cslItr)->getName();
            }
            cout << "), which has not been instantiated.\n";
            return 1;
        }
        const int ID = currOp->getID();
        if (RPGBuilder::rogueActions[ID]) {            
            cout << "Exiting: step " << idebug << " in the input plan uses the action " << *(currOp) << ", which has been pruned on the basis that it's never applicable\n";
            return 1;
        }
        planSteps.push_back(ID);
    }
    
    cout << "All necessary plan steps have been instantiated\n";
    
    return FF::testExistingSolution(planSteps,prime);
   
}
