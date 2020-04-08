#include "yuwei_scan_matching/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
      	min_dist = 100000.00;
        for(int j = 0; j<m; ++j){
          float dist = old_points[j].distToPoint2(&trans_points[i]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;

  			if(min_index==0) { second_min_index = min_index+1;} 
  			else {second_min_index = min_index-1;}
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }
}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();
 // cout<<"size n  "<< n << endl;
  //Do for each point
  for(int i = 0; i<n; ++i){

    int start_at = (last_best<0)?(last_best+1):last_best;
	int min_index = start_at;
	int second_min_index;
	float min_dist = 10000.0;

    bool up_stop = false;
    bool down_stop = false;

    // start from last best, search upwards
    int current = start_at;

    while(!up_stop && current<m){
    	float dist = trans_points[i].distToPoint2(&old_points[current]);    	
    	if (dist<min_dist){
    		min_dist = dist;
    		min_index = current;

    		if(min_index==0) { second_min_index = min_index+1;} 
  			else {second_min_index = min_index-1;}
    	}
    	float theta_diff_up = old_points[current].theta - trans_points[i].theta;
    	//cout<<"theta_diff_up:  "<< theta_diff_up << endl;
 		//check for termination
    	if (trans_points[i].r * sin(theta_diff_up) > min_dist){ 
    		up_stop = true;
    		//cout<<"up stopped:"<< endl;
    	}
    	// advance based on jump table
    	if(old_points[current].r < trans_points[i].r){
    		current = jump_table[current][UP_BIG];
    	}
    	else{
    		current = jump_table[current][UP_SMALL];
    	}
    //	cout<<"current up:  "<< current << endl;
    }
    // back to initial starting point and search downwards
    current = start_at;
    //cout<<"point "<<i<<" start at:  "<< start_at << endl;
    //ROS_INFO("START DOWNWARDS");
    while(!down_stop && current>=0){
    	float dist = trans_points[i].distToPoint2(&old_points[current]);    	
    	if (dist<min_dist){
    		min_dist = dist;
    		min_index = current;

    		if(min_index==0) { second_min_index = min_index+1;} 
  			else {second_min_index = min_index-1;}
    	}
    	float theta_diff_down = trans_points[i].theta - old_points[current].theta;
    	//cout<<"theta_diff_down:  "<< theta_diff_down << endl;
 		//check for termination
    	if (trans_points[i].r * sin(theta_diff_down) > min_dist){ 
    		down_stop = true;
    		//cout<<"down stopped:"<< endl;
    	}
    	// advance based on jump table
    	if(old_points[current].r < trans_points[i].r){
    		current = jump_table[current][DOWN_BIG];
    	}
    	else{
    		current = jump_table[current][DOWN_SMALL];
    	}
    	//cout<<"current down:  "<< current << endl;
    }

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
    last_best = min_index;
   // cout<<"min_index:  "<< min_index << endl;
    }
  }


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }

    for(int j = i+1; j<n-1; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
