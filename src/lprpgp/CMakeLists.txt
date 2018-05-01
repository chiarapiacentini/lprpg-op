PROJECT(lprpg)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules
                          ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

IF(NOT DISABLE_CLP)
find_package(Clp)
find_package(CoinUtils)
find_package(Cbc)
find_package(Cgl)
find_package(OSI)
ENDIF(NOT DISABLE_CLP)

find_package(LPSolve)
find_package(CBLAS)
find_package(CLapack)

set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS} -std=c++11 -DIL_STD")

IF("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

if(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)
    add_definitions(-DFOUNDCPLEX)
endif(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)

if(GUROBI)
    set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${GUROBI}/lib)
    add_definitions(-DFOUNDGUROBI)
endif(GUROBI)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${val_BINARY_DIR} ${CLP_INCLUDES}/coin ${CBC_INCLUDES}/coin ${CPLEX_INCLUDES} ${CONCERT_INCLUDES} ${LPSOLVE_INCLUDES} ${GUROBI}/include)

set(lprpg_build_srcs
    PreferenceHandler.cpp
    Decomposition.cpp 
    GlobalSchedule.cpp 
    RPGBuilder.cpp
    RPGBuilderAnalysis.cpp 
    FFSolver.cpp 
    NumericAnalysis.cpp
    landmarksanalysis.cpp
    MILPRPG.cpp
    MetricRPG.cpp
    NNF.cpp
    LiteralAnalysis.cpp
    solver.cpp
    SubproblemMIP.cpp
    DotSearchSpace.cpp DotSearchSpace.h
    Color.cpp Color.h
    IPCompilation.cpp IPCompilation.hpp
    VariableElimination.cpp VariableElimination.hpp
    IPModel.cpp IPModel.hpp
    IPSolver.cpp IPSolver.hpp
    IPHeuristic.cpp IPHeuristic.hpp
    IPStateBasedModel.cpp IPStateBasedModel.hpp
    IPStateBasedLinearModel.cpp IPStateBasedLinearModel.hpp
    IPStateBasedUnified.cpp IPStateBasedUnified.hpp
    IPStateChangeModel.cpp IPStateChangeModel.hpp
    IPStateChangeLinearModel.cpp IPStateChangeLinearModel.hpp
    IPStateChangeUnified.cpp IPStateChangeUnified.hpp
)

file(GLOB_RECURSE INCS "*.h")

add_library(LPRPGCommon STATIC ${lprpg_build_srcs} ${INCS})

target_link_libraries(LPRPGCommon ParsePDDL Inst)

if(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES AND NOT(GUROBI))
    ADD_EXECUTABLE(lprpg-cplex lprpgMain.cpp solver-cpx.cpp solver-cpx-globals.cpp solver-cpx-paramils.cpp CPLEXSolver.cpp CPLEXSolver.hpp)
    target_link_libraries(lprpg-cplex
	LPRPGCommon
        ${ILOCPLEX} ${CPLEX} ${CONCERT} pthread
    )

endif(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES AND NOT(GUROBI))
    
if(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES AND GUROBI)
    ADD_EXECUTABLE(lprpg-both lprpgMain.cpp solver-cpx.cpp solver-cpx-globals.cpp solver-cpx-paramils.cpp CPLEXSolver.cpp CPLEXSolver.hpp GurobiSolver.cpp GurobiSolver.hpp)

    target_link_libraries(lprpg-both
	LPRPGCommon
        ${ILOCPLEX} ${CPLEX} ${CONCERT}
	gurobi75 ${GUROBI}/lib/libgurobi_c++.a  pthread
)
endif(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES AND GUROBI)


