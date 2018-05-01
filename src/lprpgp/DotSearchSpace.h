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

#ifndef DOTSEARCHSPACE_H
#define DOTSEARCHSPACE_H
#include <map>
#include <list>
#include <string>
#include <ostream>

using namespace std;



class DotSearchSpace
{
    
public:
    struct Node{
        Node(int i, string n, int h, double c = 0.0, double sh = 0) : id(i), name(n), heuristic(h), cost(c), secondHeuristic(sh) {};
        int id;
        string name;
        list<int> children;
        int heuristic;
        double secondHeuristic;
        double cost;
    };
    typedef map<int,Node> Graph;
    static Graph graph;
    static string nameFile;
    static string nameGnuplot;
    static string namePdf;
    static bool addColor;
    static bool addEdge;
    static bool addHeuristic;
    static void printDotFile(ostream &o);
    static void printDotFile(string nameFile);
    static void printDotFile();
    static void addNode(int i, string n, int h, int idParent, double cost, double sh = 0);
    static void printGnuplotFile(ostream &o);
    static void printGnuplotFile();
    static int changeSearch;
    static bool writeFluents;
private:
    static Graph init_map(){
        Graph graph;
        return graph;        
    }
};

#endif // DOTSEARCHSPACE_H
