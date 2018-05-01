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
#include "KMeans.hpp"
#include "GlobalSchedule.h"


void Cluster::removePoint(int id_point) {
    modify = true;
    points_id.remove(id_point);
    new_removed.push_back(id_point);
}

int KMeans::getIDNearestCenter(Point & point) {
    double sum = 0.0, min_dist;
    int id_cluster_center = 0;
    int id_point = point.getID();
    
    for(int i = 0; i < total_values; i++) {
        sum += pow(clusters[0].central_values[i] - point.values[i], 2.0);
    }
    
    min_dist = sqrt(sum);
    lb[id_point][0] = min_dist;
    point.ubound = min_dist;
    
    for(int i = 1; i < K; i++) {
    	lb[id_point][i] = 0.0;
    	
        double dist;
        sum = 0.0;
        
        for(int j = 0; j < total_values; j++) {
            sum += pow(clusters[i].central_values[j] - point.values[j], 2.0);
        }
        
        dist = sqrt(sum);
        
        if(dist < min_dist) {
            min_dist = dist;
            id_cluster_center = i;
            point.ubound = min_dist;
        }
    }
    
    return id_cluster_center;
}


void KMeans::run(vector<Point> & points)
{
    bool debug = Planner::GlobalSchedule::globalVerbosity & 64;

	// one cluster per node if the # of points is smaller
    if(K > total_points){
		for (int i =0; i < total_points; i++){
			points[i].setCluster(i);
		}
		
        return;
   	}

    // choose K distinct values for the centers of the clusters
    if(debug) cout << "Generating " <<  K <<  " clusters with " << total_points << " number of points and " << total_values << " values per point."<< endl;
    
    
	//Initialize clusters
	initialRandom(points);
    
    int iter = 1;
    while(true) {
        bool done = true;
        
        // associates each point to the nearest center
        for(int i = 0; i < total_points; i++) {
            int id_old_cluster = points[i].getCluster();
            int id_nearest_center = getIDNearestCenter(points[i]);
            
            if(id_old_cluster != id_nearest_center) {
                if(id_old_cluster != -1)
                    clusters[id_old_cluster].removePoint(points[i].getID());
                
                points[i].setCluster(id_nearest_center);
                clusters[id_nearest_center].addPoint(i);
                done = false;
            }
        }
        
        // recalculating the center of each cluster
        for(int i = 0; i < K; i++) {
        
        	int total_points_cluster = clusters[i].getTotalPoints();
        	
        	if(total_points_cluster > 0) {
        	
            	for(int j = 0; j < total_values; j++) {
                	double sum = 0.0;
                	
                    for( list<int>:: iterator it = clusters[i].points_id.begin(); it != clusters[i].points_id.end(); ++it){
                    	int point_id = (*it); 
						sum += points[point_id].values[j];
					}
                    
                    clusters[i].setCentralValue(j, sum / total_points_cluster);
                }
            }
        }
        
        if(done == true || iter >= max_iterations) {
            if (debug) cout << "Break in iteration " << iter << "\n\n";
            break;
        }
        
        iter++;
    }
    
    // shows elements of clusters
    if (debug) printClusters(points);
}

// ======= New implemenatation of Kmeans using triangular inequalities
void KMeans::run_new(vector<Point> & points){
	
	bool debug = Planner::GlobalSchedule::globalVerbosity & 64;

	if(debug) cout << "Generating " <<  K <<  " clusters with " << total_points << " number of points and " << total_values << " values per point."<< endl;
	
	// one cluster per node if the # of points is smaller
    if(K > total_points){
    	if(debug) cout << "One point per cluster " << endl;
    	
		for (int i = 0; i < total_points; i++){
			points[i].setCluster(i);
			clusters[i].addPoint(i);
			clusters[i].setID(i);
			clusters[i].central_values = points[i].values;
			clusters[i].total_ids = points[i].total_ids;
		}
		
		if (debug) printClusters(points);
        return;
   	}

	//Initialize clusters
	bool plus = true;
	if(plus) {
		initialPlusPlus(points);
	}
	else{
		initialRandom(points);
	
   		// associates each point to the nearest center
    	for(int i = 0; i < total_points; i++) {
        	int id_old_cluster = points[i].getCluster();
        
        	if (id_old_cluster >= 0) continue;
        
        	int id_nearest_center = getIDNearestCenter_optimize(points[i]);
            
        	if(id_old_cluster != id_nearest_center) {
				if(id_old_cluster != -1)
					clusters[id_old_cluster].removePoint(i);
                
				points[i].setCluster(id_nearest_center);
				clusters[id_nearest_center].addPoint(i);
 			}
    	}
    }
    
    //compute centers of each cluster
    compute_centers_opt(points);
    
    int iter = 1;
    double sum = 0.0, dist = 0.0, diff = 0.0;
    
    
    while(true) {
    	//cout << "==Iteration " << iter << endl;
        bool done = true;

        //Step 1:  Compute distance between centers and s(k), for each cluster k 
        compute_dist_centers();
        
		// associates each point to the nearest center
        for(int i = 0; i < total_points; i++) {
            int id_old_cluster = points[i].getCluster();
            
            //Step 2: check if node needs update 
            if ( points[i].ubound > s[id_old_cluster]){
				//Set 3: smart update on the center closer to the point
            	updatePoint(points[i], done);
            }
        }
        
        //Step 4 & 7: Compute new centers for each cluster
        // save the older centers
        for (int i = 0; i < K; i++){
        	if(!clusters[i].modify) continue;
        	
        	old_c[i] = clusters[i].getCenter();
        }
        
        //Step 4 : compute new center
        compute_centers_opt(points);
        
        for (int k = 0; k < K; k++) {
        	//compute distance between new and old center
        	if(!clusters[k].modify) continue;
        	
        	sum = 0.0;
        	for (int j = 0; j < total_values; j++) {
           		sum += pow(clusters[k].central_values[j] - old_c[k][j], 2.0);     
           	}
           	// p[k] = dist;
        	dist = sqrt(sum);
        	
        	for (int i = 0; i < total_points; i++) {
        		//Step 5: compute lower bounds
        		diff = lb[i][k] - dist;
        		
        		if (diff > 0.0) 	lb[i][k] = diff;
        		else 				lb[i][k] = 0.0;
        	}
        	
        	for (auto& id : clusters[k].points_id) {
        		points[id].ubound +=  dist;
        		points[id].r = true;
        	}
        }
        
        
        if(done == true || iter >= max_iterations) {
        	//if(iter > 1) cout << iter << endl;
            if (debug) cout << "Break in iteration " << iter << "\n\n";
            break;
        }
        
        iter++;
    }
    
    //Count ids per cluster
    for (int k = 0; k < K; k++) {
    	clusters[k].total_ids = 0;
     	for(auto& i : clusters[k].points_id){
     		clusters[k].total_ids += points[i].total_ids;
     	}
    }
    
    
    if (debug) printClusters(points);
    
}

int KMeans::getIDNearestCenter_optimize(Point & point) {
	double sum = 0.0, min_dist = 0.0, dist = 0.0;
    int id_cluster_center = 0;
    int id_point = point.getID();
    
    for(int i = 0; i < total_values; i++) {
        sum += pow(clusters[0].central_values[i] -
                   point.values[i], 2.0);
    }
    
    min_dist = sqrt(sum);
    lb[id_point][0] = min_dist;
    point.ubound = min_dist;
    
    for(int k = 1; k < K; k++) {
    	lb[id_point][k] = 0.0;
    	
    	if(cdist[id_cluster_center][k] >= 2*min_dist){
    		lb[id_point][k] = 2*min_dist;  // in the best case is this number
    	 	continue;
    	 }
    	
        dist = pow(min_dist, 2.0);
        sum = 0.0;
        
        for(int j = 0; j < total_values; j++) {
            sum += pow(clusters[k].central_values[j] - point.values[j], 2.0);
            if (sum > dist) break;
        }
        
        dist = sqrt(sum);
        lb[id_point][k] = dist;
        
        if(dist < min_dist) {
            min_dist = dist;
            id_cluster_center = k;
            point.ubound = min_dist;
        }
    }
    
    return id_cluster_center;
}

void KMeans::updatePoint(Point & point, bool& done ) {
    
    int id_original_cluster = point.getCluster();
    int id_cluster = id_original_cluster;
    int id = point.getID();
    
    double min_dist = 0.0;
    double sum = 0.0;
    double dist_k = 0.0;
    double aux = 0.0;
    
    // Step 3.a: approximate distance to the current center
    if (point.r) {
    	sum = 0.0;
    	for(int i = 0; i < total_values; i++) {
        	sum += pow(clusters[id_cluster].central_values[i] - point.values[i], 2.0);
    	}
    	min_dist = sqrt(sum);
    		
    	//Center got closer to the point
    	point.ubound = min_dist;

    	point.r = false;
    } else {
    	min_dist = point.ubound;
    }
    
    for (int k = 0; k < K; k++) {
    	if (k == id_original_cluster) continue;
    	if (!( (point.ubound > lb[id][k]) || (point.ubound*2 > cdist[k][id_cluster])) ) continue;  
    	
    	//Step 3.b: 
    	if ( (min_dist > lb[id][k]) || ( 2*min_dist > cdist[k][id_cluster]) ) {
    		dist_k = 0.0;
    		sum = 0.0;
    		aux = pow(min_dist, 2.0);
    		
    		//if (clusters[k].modify == false && lb[id][k] > min_dist) continue;
    		
    		//compute distance to center k
    		for(int i = 0; i < total_values; i++) {
        		sum += pow(clusters[k].central_values[i] - point.values[i], 2.0);
        		if(sum > aux) break;
    		}

    		dist_k = sqrt(sum);
    		lb[id][k] = dist_k;

    		// Update center of the node
    		if (dist_k + 0.001 <= min_dist) {
                id_cluster = k;
                point.ubound = dist_k;
                min_dist = dist_k;
                done = false;
    		}
    	}
    }
    
    // Update cluster and point
    if (id_cluster != id_original_cluster) {
   	 	//cout << "Update point " << id << " from cluster " << id_original_cluster << " to " << id_cluster << endl;
    	clusters[id_original_cluster].removePoint(id);
        point.setCluster(id_cluster);
        clusters[id_cluster].addPoint(id);
    }
}


void KMeans::compute_centers(vector<Point> & points) {
	double sum = 0.0;
	int id = 0;
	
	for(int i = 0; i < K; i++) {
		if (!clusters[i].modify) continue;
		
		int total_points_cluster = clusters[i].getTotalPoints();
        //cout << "Cluster " << i <<" Total = " << total_points_cluster << endl;	
		if (total_points_cluster > 0) {
		
			list<int>:: iterator it = clusters[i].points_id.begin();
            list<int>:: iterator it_end = clusters[i].points_id.end();
        	
        	
			for(int j = 0; j < total_values; j++) {
				sum = 0.0;
                it = clusters[i].points_id.begin();
                
				for( ; it != it_end; ++it){
					id = (*it);
					sum += points[id].values[j];
				}
				sum = sum /total_points_cluster;
                    
				clusters[i].setCentralValue(j, sum );
				//cout << "\tvalue " << j << " = " << sum << endl;
			}
		}
	}
}

void KMeans::compute_centers_opt(vector<Point> & points){
	double sum = 0.0;
	int total_points_cluster = 0, total_add = 0,  total_removed = 0;
	
	for(int i = 0; i < K; i++) {
		if (!clusters[i].modify) continue;
		
		total_points_cluster = clusters[i].getTotalPoints();
		
		if(total_points_cluster == 0) continue;
		
		total_add = clusters[i].new_added.size();
		total_removed = clusters[i].new_removed.size();
		
		//cout << "Cluster " << i <<" Total = " << total_points_cluster << ", added = " << total_add << ", removed = " << total_removed << endl;
		
		// Old way to compute cluster
		if (total_points_cluster <= (total_add + total_removed) ){
			//cout << "old way" << endl;
			list<int>:: iterator it = clusters[i].points_id.begin();
            list<int>:: iterator it_end = clusters[i].points_id.end();
        	int id = 0;
        	
			for(int j = 0; j < total_values; j++) {
				sum = 0.0;
                it = clusters[i].points_id.begin();
                
				for( ; it != it_end; ++it){
					id = (*it);
					sum += points[id].values[j];
				}
				sum = sum /total_points_cluster;
                clusters[i].central_values[j] = sum;    
				
				//clusters[i].setCentralValue(j, sum );
				//cout << "\tvalue " << j << " = " << sum << endl;
			}
		}
		//New way to compute cluster
		else{ 
			//cout << "new update! " << total_points_cluster - total_add + total_removed << endl;
			//add new points
			for(int j = 0; j < total_values; j++) {
				sum = clusters[i].central_values[j]*(total_points_cluster - total_add + total_removed );
                //cout << "old center = " << clusters[i].central_values[j] << " sum =" << sum << endl;
                //add new points
				for(auto& id : clusters[i].new_added){
					sum += points[id].values[j];
				}
				
				//remove old points
				for(auto& id : clusters[i].new_removed){
					sum -= points[id].values[j];
				}
				
				sum = sum /total_points_cluster;
                clusters[i].central_values[j] = sum;
                    
				//clusters[i].setCentralValue(j, sum );
				//cout << "\tvalue " << j << " = " << sum << endl;
			}
		}
	}
}

void KMeans::compute_dist_centers(){
	double aux;
	double sum = 0.0;
	
	fill(s.begin(), s.begin() + K, -1.0);
	
	for (int i = 0; i < K-1; i++) {
		for (int j = i+1; j < K; j++) {

			if(clusters[i].modify || clusters[j].modify) {
				sum = 0.0;
				//compute distance between clusters
				for(int k = 0; k < total_values; k++) {
           			sum += pow(clusters[i].central_values[k] - clusters[j].central_values[k], 2.0);     
           		}
           		aux = sqrt(sum);
				cdist[i][j] = aux;
				cdist[j][i] = aux;
			}
			
			aux = (cdist[i][j]/2);
			
			if (s[i] == -1.0 || aux < s[i]) s[i] = aux;
			if (s[j] == -1.0 || aux < s[j]) s[j] = aux;
		}
	}
	
	//set modifications to false
	for(int i = 0; i < K; i++){
		clusters[i].modify = false;
		clusters[i].new_added.clear();
		clusters[i].new_removed.clear();
	}
}

void KMeans::initialRandom(vector<Point> & points) {
	//Randomly assign the centers of the clusters
	
	srand (1);
    vector<int> index(total_points, 0);
    for(int i = 0; i < total_points; i++) index[i] = i;
    random_shuffle(index.begin(), index.end());

	// randomly assing clusters
	for(int i = 0; i < K; i++) {
        int j = index[i];
        points[j].setCluster(i);
        points[j].ubound = 0.0;
        clusters[i].setID(i);
        clusters[i].central_values = points[j].values;
        clusters[i].addPoint(j);
    }

}

void KMeans::initialPlusPlus(vector<Point> & points){

	//Fist cluster --> random
	srand (1);
	int choosen = rand()%total_points;
	int id_cluster = 0;
	
	vector<double> min_dist(total_points, INFINITY);
	vector<int> closest_cluster(total_points, 0);
	
	min_dist[choosen] = -1.0;
	closest_cluster[choosen] = -1;
	points[choosen].setCluster(0);
    points[choosen].ubound = 0.0;
    clusters[0].setID(0);
    clusters[0].central_values = points[choosen].values;
    clusters[0].addPoint(choosen);
    
    double sum = 0.0;
    double dist = 0.0;
    
    int id_max= 0; double val_max = 0;
    
	//Choose the next K clusters
	for(int k=1; k < K; k++){
		val_max = 0;

		//Compute min distance between points and newest cluster ( k-1)
		for(int i =0; i < total_points; i++){
			lb[i][id_cluster] = 0.0;
			
			if( closest_cluster[i] == -1 ) continue;
			
			//distance to newest cluster
			sum = 0.0;
			dist = pow(min_dist[i], 2.0);
			for (int j=0; j < total_values; j++) {
				sum += pow(points[i].values[j] - points[choosen].values[j], 2.0); 
				
				if(sum >= dist + 0.0001) break;  
			}
			
			dist = sqrt(sum);
        	lb[i][id_cluster] = dist;
			
			//update min_dist and cluster
			if( dist < min_dist[i]){
				min_dist[i] = dist;
				closest_cluster[i] = id_cluster;
			}
			
			//find next point that is fardest to all the rest
			if( val_max < min_dist[i]){
				val_max = min_dist[i];
				id_max = i;
			}
			
		}
		
		//Pick new cluster base on the furthest point to all clusters
		//choosen = distance(min_dist.begin(), max_element(min_dist.begin(), min_dist.end()));
		choosen = id_max;
		
		//choosen point to be a cluster
		min_dist[choosen] = -1.0;
		closest_cluster[choosen] = -1;
		points[choosen].setCluster(k);
    	points[choosen].ubound = 0.0;
    	
    	//Update info of the cluster
    	id_cluster = k;
		clusters[k].setID(k);
    	clusters[k].central_values = points[choosen].values;
    	clusters[k].addPoint(choosen);
	}
	
	//Compute distances to last cluster
	for(int i = 0; i < total_points; i++){
			lb[i][id_cluster] = 0.0;
			
			if( closest_cluster[i] == -1 ) continue;
			
			//distance to newest cluster
			sum = 0.0;
			dist = pow(min_dist[i], 2.0);
			for (int j=0; j < total_values; j++) {
				sum += pow(points[i].values[j] - points[choosen].values[j], 2.0); 
				
				if(sum >= dist + 0.0001) break;  
			}
			
			dist = sqrt(sum);
        	lb[i][id_cluster] = dist;
			
			//update min_dist and cluster
			if( dist < min_dist[i]){
				min_dist[i] = dist;
				closest_cluster[i] = id_cluster;
			}
			
	}
		
	//Set points to the closest clusters so far
	
	for(int i = 0; i < total_points; i++) {
		if( closest_cluster[i] == -1 ) continue;
		
		id_cluster = closest_cluster[i];
		points[i].setCluster(id_cluster);
		points[i].ubound = min_dist[i];
		clusters[id_cluster].addPoint(i);
	}
	
}




//======================================================================================================

void KMeans::update( int K, int total_points, int total_values, int max_iterations) {
	this->K = K;
	this->total_points = total_points;
	this->total_values = total_values;
	this->max_iterations = max_iterations;
	
	int max_K = s.size();
	int max_points = lb.size();
	int max_values = old_c[0].size();
        
	if (K > max_K ) {
		bool debug = Planner::GlobalSchedule::globalVerbosity & 64;
		if( debug) cout << "Increasing max number of cluster. Old = " << s.size() << ", New = " << K << endl;
		s.resize(K, -1.0);
		clusters.resize(K, Cluster());
		cdist = vector<vector<double> >(K, vector<double>(K, 0.0) );
        	
        //update lb
        if(max_points < total_points){
        	//Case 1: both K and total_points are bigger now 
        	max_points = total_points;
        	lb = vector<vector<double> >( max_points, vector<double>(K, 0.0) );
        }else{
        	// Case 2: only K is bigger
        	for(int i = 0; i < max_points; i++){
        		lb[i].resize(K, 0.0);
        	}
        }

        //update old_c
        if(max_values < total_values){
        	//Case 1 ; both K and total_values have increased
        	old_c = vector<vector<double> >(K, vector<double>(total_values, 0.0) );
        }
        else{
        	//Case 2: only K has increased
        	old_c.resize(K, vector<double>(max_values, 0.0));
        }
        
        	
	}else{
		// Update lb in the case that the number of points is bigger
    	if(total_points > max_points ){
        	lb.resize(total_points, vector<double>(max_K, 0.0) );
        }
        
        //update old_c
        if( total_values > max_values ){
        	for(int k = 0; k < max_K; k++){
        		old_c[k].resize(total_values, 0.0);
        	}
        }
        
        //fill numbers in cdist
        for(int i = 0; i < K; i++){
        	fill(cdist[i].begin(), cdist[i].begin() + K, 0.0);
        }
	}
	
	//reset clusters
	for(int i = 0; i < K; i++){
		clusters[i].points_id.clear();
		clusters[i].new_added.clear();
		clusters[i].new_removed.clear();
		clusters[i].modify = true;
	}
}
    

void KMeans::printClusters(vector<Point> & points) {
	
	int size = K;
	if ( total_points < K ) size = total_points;
	
	for(int i = 0; i < size; i++) {
            
		cout << "Cluster " << i << endl;
		
		for( list<int>:: iterator it = clusters[i].points_id.begin(); it != clusters[i].points_id.end(); ++it){
			int j = (*it);
			cout << "Point " << (*it) << ": ";
			for(int p = 0; p < total_values; p++)
				cout << points[j].values[p] << " ";
                
			string point_name =  points[j].getName();
                
			if(point_name != "")
				cout << "- " << point_name;
				
			cout << endl;
		}
            
		cout << "Cluster values: ";
            
		for(int p = 0; p < total_values; p++)
			cout << clusters[i].central_values[p] << " ";
            
		cout << "\n\n";
	}
}

int testKMeans(int argc, char *argv[])
{
    srand (time(NULL));
    
    int total_points, total_values, K, max_iterations, has_name;
    
    cin >> total_points >> total_values >> K >> max_iterations >> has_name;
    
    vector<Point> points;
    string point_name;
    
    for(int i = 0; i < total_points; i++)
    {
        vector<double> values;
        
        for(int j = 0; j < total_values; j++)
        {
            double value;
            cin >> value;
            values.push_back(value);
        }
        
        if(has_name)
        {
            cin >> point_name;
            Point p(i, values, point_name);
            points.push_back(p);
        }
        else
        {
            Point p(i, values);
            points.push_back(p);
        }
    }
    
    KMeans kmeans(K, total_points, total_values, max_iterations);
    kmeans.run(points);
    
    return 0;
}

