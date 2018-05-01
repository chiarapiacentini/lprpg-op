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

#ifndef KMeans_hpp
#define KMeans_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <list>

using namespace std;

class Point {
private:
    int id_point, id_cluster;
    int total_values;
    
    string name;
    
public:
	Point(){
		arc_id.assign(10, 0);
	}

    Point(int id_point, vector<double>& values_new, string name = "") {
        this->id_point = id_point;
        total_values = values_new.size();
        values = values_new;
        cluster_dist = -1.0;
        ubound = -1.0;
        
        this->name = name;
        id_cluster = -1;
        r = true;
        arc_id.assign(10, 0);
    }
    
    double cluster_dist;
    double ubound;
    bool r;
    vector<double> values;
    vector<int> arc_id;
    int total_ids;
    
    void clear(){
    	total_values =	-1;
		id_cluster = -1;
        r = true;
        cluster_dist = -1.0;
        ubound = -1.0;
        total_ids = 1;
    }

    int getID() {
        return id_point;
    }
    
    void setID( int id) {
        id_point = id;
    }
    
    void set_arcID(int id)
    {
        arc_id.push_back(id);
    }
    
    void setCluster(int id_cluster)
    {
        this->id_cluster = id_cluster;
    }
    
    int getCluster()
    {
        return id_cluster;
    }
    
    double getValue(int index)
    {
        return values[index];
    }

    void setValue(int index, double value)
    {
        values[index] = value;
    }
    

    int getTotalValues()
    {   
        return values.size();
    }
    
	void setTotalValues( int total){
    	total_values = total;
    }
    
    void addValue(double value)
    {
        values.push_back(value);
    }
    
    string getName()
    {
        return name;
    }
};

class Cluster
{
private:
    int id_cluster;

public:
	
	list<int> points_id;
	vector<double> central_values;
	bool modify;
	list<int> new_added;
	list<int> new_removed;
	int total_ids;
	
	Cluster(){
		modify = true;
		total_ids = 0;
	}
	
    Cluster(int id_cluster, Point& point) {
        this->id_cluster = id_cluster;
        central_values = point.values;
        int id = point.getID();
        points_id.push_back(id);
        modify = true;
    }
    
    void addPoint(int point_id)
    {
        points_id.push_back(point_id);
        new_added.push_back(point_id);
		modify = true;
    }
    
    
    void removePoint(int id_point);
    
    void setCentralValue(int index, double value)
    {
        central_values[index] = value;
    }
    
    vector<double>& getCenter(){
    	return central_values;
    } 
    
    int getTotalPoints()
    {
        return points_id.size();
    }
    
    int getID() {
        return id_cluster;
    }
    
    void setID(int id){
    	id_cluster = id;
    }

};

class KMeans
{
private:
    int K; // number of clusters
    int total_values, total_points, max_iterations;
    vector<vector<double> > lb;			// point x cluster
    vector<vector<double> > cdist;		// cluster x cluster
    vector<double> s;					// min dist for one cluster to all the other clusters
    vector<vector<double> > old_c;		//center of each cluster, previous iteration
    
    // return ID of nearest center (uses euclidean distance)
    int getIDNearestCenter(Point & point);
    int getIDNearestCenter_new(Point & point);
    int getIDNearestCenter_optimize(Point & point);
    
    // update cluster of a node
    void updatePoint(Point & point, bool& done);
    
    //compute centers for each cluster
    void compute_centers(vector<Point> & points);
    void compute_centers_opt(vector<Point> & points);
    
    // compute distances between centers and min distances
    void compute_dist_centers();
    
    //Print final clusters
    void printClusters(vector<Point> & points);
    
    //Initial clusters function
    void initialRandom(vector<Point> & points);
    void initialPlusPlus(vector<Point> & points);
    

public:
	vector<Cluster> clusters;
	
	KMeans(){
	
	}

    KMeans(int K, int total_points, int total_values, int max_iterations) {
        this->K = K;
        this->total_points = total_points;
        this->total_values = total_values;
        this->max_iterations = max_iterations;
        
        lb.assign(total_points, vector<double>(K, 0.0) );
        old_c.assign(K, vector<double>(total_values, 0.0) );
        cdist.assign(K, vector<double>(K, 0.0));
        s.assign(K, -1.0);
        
        clusters.assign(K, Cluster() );
    }
    
    void update( int K, int total_points, int total_values, int max_iterations);
    
    void run(vector<Point> & points);
    void run_new(vector<Point> & points);
};


#endif /* KMeans_hpp */
