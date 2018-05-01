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


#ifndef COLOR_H
#define COLOR_H

#include <iostream>
#include <string>

using namespace std;

class Color{
public:
    Color();
    Color(int red, int green, int blue);
    //Color(string hex);
    int getRed(){
        return red;
    };
    int getGreen(){
        return green;
    }
    int getBlue(){
        return blue;
    }
    string getHex(){
        return hex;
    }
private:
    string hex;
    int red;
    int green;
    int blue;
    string fromRGB2Hex(int red, int green, int blue);
    int* fromHex2RGB(string Hex);
    string byte2Hex(int number);
};

void makeColorGradien(double frequency1, double frequency2, double frequency3, double phase1, double phase2, double phase3, double center, double width, int len = 50, Color color[] = NULL);

void makeColorGradien(double frequency, double phase1, double phase2, double phase3, double center, double width, int len = 50, Color color[] = NULL);

void makeColorGradien(double frequency, double phase, double center, double width, int len = 50, Color color[] = NULL);

void makeColorGradien(double frequency, double center, double width, int len = 50, Color color[] = NULL);

void makeGrayGradien(double frequency, double phase, double center, double width, int len = 50, Color color[] = NULL);

Color createColor(double i, double min, double max, double center = 128., double width = 127.);

Color createColor2(double i, double min, double max, double center = 128., double width = 127.);

#endif // COLOR_H
