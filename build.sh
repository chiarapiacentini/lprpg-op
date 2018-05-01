#!/bin/bash

DIRCOMPILE=compile
mkdir -p $DIRCOMPILE
cd $DIRCOMPILE

if [ ! -e "$CPLEX" ]; then
    echo "$CPLEX is not the libcplex.a library file"
    exit 1
fi

if [ ! -e "$ILOCPLEX" ]; then
    echo "$ILOCPLEX is not the libilocplex.a library file"
    exit 1
fi

if [ ! -e "$CONCERT" ]; then
    echo "$CONCERT is not the libconcert.a library file"
    exit 1
fi


if [ ! -e "$CPLEX_INCLUDES/ilcplex/ilocplex.h" ]; then
    echo "$CPLEX_INCLUDES does not point to a path containing ilcplex/ilocplex.h"
    exit 1
fi


if [ ! -e "$CONCERT_INCLUDES/ilconcert/iloenv.h" ]; then
    echo "$CONCERT_INCLUDES does not point to a path containing ilconcert/iloenv.h"
    exit 1
fi


if [ ! -e "$GUROBI/lib/libgurobi_c++.a" ]; then
    echo "$CONCERT_INCLUDES does not point to a path containing ilconcert/iloenv.h"
    cmake -DCMAKE_VERBOSE_MAKEFILE=TRUE -DCMAKE_BUILD_TYPE=Release -DCPLEX="$CPLEX" -DILOCPLEX="$ILOCPLEX" -DCONCERT="$CONCERT" -DCPLEX_INCLUDES="$CPLEX_INCLUDES" -DCONCERT_INCLUDES="$CONCERT_INCLUDES" ../src
    echo "cd $DIRCOMPILE; make lprpg-cplex"
else
    cmake -DCMAKE_VERBOSE_MAKEFILE=TRUE -DCMAKE_BUILD_TYPE=Release -DCPLEX="$CPLEX" -DILOCPLEX="$ILOCPLEX" -DCONCERT="$CONCERT" -DCPLEX_INCLUDES="$CPLEX_INCLUDES" -DCONCERT_INCLUDES="$CONCERT_INCLUDES" -DGUROBI="$GUROBI" ../src
    echo "cd $DIRCOMPILE; make lprpg-both"
fi

cd ../
