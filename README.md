# Udacity-CarND-Path-Planning

## Path Planning Project
The goal of this project is to drive a car around a simulated highway including traffic. In all previous projects we just used to drive only our car around the track, but this was a bit different. The simulator tries to inact the actual highway situation and we just need to write our code in such a way that car drives better than we drive!!! As always there are a number of constraints-:

* The car must not exceed the speed limit of 50mph on the highway
* The car does not exceed a total acceleration of 10 m/s^2
* The car does not exceed a total jerk of 10 m/s^3
* The car does not have collisions with any of the cars while changing lanes or driving inside a lane
* The car stays in its lane, except for the time between changing lanes.
* If the car ahead is moving slow, then we should be able to change lanes.



## Rubric Points

1. *The Code compiles correctly.*- The code complied successfully. I used spline approach for smoothening and I have added the spline.h file in the folder.

2. *The car is able to drive at least 4.32 miles without incident.* - The car drove around the highway easily. I tested for 9.5 miles and the car drove without any incidents.

3. *The car drives according to the speed limit.*- As I mentioned in Point 2 the car drove without any incidents, and maintaining speed limit for one of the incidents. The car drives at a max speed of 49.5 if there is no slow moving traffic ahead of car, if the car encounters a slow moving traffic, it tries to change the lane. If lane change is successful, then well and good otherwise car will lower its speed and keep moving in the same lane.

4. *Max Acceleration and Jerk are not Exceeded.* - True, the acceleration and jerk were in prescribed limits. 

5. *Car does not have collisions.*- There were no collisions, there were one or two incidents where there were collisions while changing lanes, but proper code was then implemented which I will explain in the pipeline.

6. *The car stays in its lane, except for the time between changing lanes.*- The car stays inside the lane, we are just responsible for providing the lane number, the control section is handeled by the simulator.

7. *The car is able to change lanes*- Whenever there is a slow moving traffic the car tries to change the lane and it does it successfully if there are not cars nearby.

8. *There is a reflection on how to generate paths.* Described below.

## Model Documentation

The simulator provides information about Car's location, velocity, yaw rate, speed, frenet coordinates and sensor fusion data.  I used spline function, which fits a line to given points in a fairly smooth function. The classroom talks about the fifth order jerk minimizing trajectory but I decided to use spline as it is a well made library and it will reduce our computations. After fitting a line, I then feed points along that line back to the simulator.

### Prediction

The data from the sensor fusion and simulator is used to generate the prediction of what the other vehicles are likely to do. The sensor fusion data gives us data about the other cars nearby and we predict what they are likely to do. We change our behaviour on the basis of the Prediction of other cars behaviour

### Behaviour Planning 


My algo for behaviour planning is as follows-:

1. The car checks 25 m ahead if there is any car present or not.
2. If the senses a car 25 m ahead and it is slow moving so it tries to change the lane.
3. If the car ahead is too close then we fetch the successor states for the current lane. For this I have implemented `getSuccessorStates()` method.
4. For each successor state we calcualte cost, to calculate cost I have considered the position and velocity of the vehicles in the lane for which I am calculating the cost. I am checking the nearest car positions and based on their position and veloctity I am calculating the cost. However for the current lane I am not considering the vehicles behind our car. The car will move to lane, in which the vehicles are not close to the car and they are moving faster than vehicles in other lanes.
5. Once I have cost for each state then I make trasition to the state which has minimum cost.


### Trajectory Generation

The car calucates the trajectory based on its own speed, the speed of surrounding cars, current lane, intended lane and past points. To make trajectory smoother, we add last two points from the previous path points provided by the simulator. If there are no previous points then we calculate the previous points from the current yaw and current car coordinates. Also the 3 points are added in next 30, 60, 90 meters to trajecotry. All these points are shifted to car reference angle.


### Final Logic for Switching Lanes and to calculate Cost
```
// function that calls the getCost funtion and make lane changes



if(too_close){
	ref_vel-=0.35;
	vector<int> states=getSuccessorStates(lane);
	vector<double> costs;
	bool is_current=false;
	for(int index=0;index<states.size();index++){
		int check_for=states[index];
		if(check_for==lane) is_current=true;
		else is_current=false;
		costs.push_back(getCost(check_for,is_current,50.0,car_s,car_d,sensor_fusion,prev_size,ref_vel));
	}
	int min_cost_index=0;
	for(int index=1;index<costs.size();index++){
		if(costs[min_cost_index]>costs[index]) min_cost_index=index;
	}

	lane=states[min_cost_index];
	}
else if(ref_vel<49.0){
	ref_vel+=0.35;
}


// function that Calculate cost for each successor state.

double getCost(int check_lane,bool is_current,double ref_vel,double car_s,double car_d,const vector<vector<double>> &sensor_fusion,int prev_size,double car_vel){

	double cost=0;
	int nearest_car_ahead=-1;
	int nearest_car_behind=-1;
	
	for(int i=0;i<sensor_fusion.size();i++){
		double d=sensor_fusion[i][6];
		
		if(d<(2+4*check_lane+2) && d>(2+4*check_lane-2)){
			double veh_s=sensor_fusion[i][5];
			if(nearest_car_ahead==-1 && car_s<veh_s) nearest_car_ahead=i;
			else if(veh_s>=car_s && veh_s<sensor_fusion[nearest_car_ahead][5]) nearest_car_ahead=i;

			if(nearest_car_behind==-1 && car_s>veh_s) nearest_car_behind=i;
			else if(veh_s<=car_s && veh_s>sensor_fusion[nearest_car_behind][5]) nearest_car_behind=i;
		}
	}

	if(nearest_car_ahead!=-1){
		
		double veh_s=sensor_fusion[nearest_car_ahead][5];
		double vx=sensor_fusion[nearest_car_ahead][3];
		double vy=sensor_fusion[nearest_car_ahead][4];
		double veh_vel=sqrt(vx*vx+vy*vy);

		veh_s+=(double(prev_size)*0.02*veh_vel);
		
		double vel_delta;
		
		if(veh_vel>=ref_vel) vel_delta=1.0;
		else vel_delta=ref_vel-veh_vel;
				
		double delta_s=(veh_s-car_s);
		
		if(is_current){
			cost+=1-exp(-(vel_delta/ref_vel)/(delta_s/25));
			
			return cost;
			
		}
		
		if(abs(delta_s)<12) cost+=10;
		
		cost+=1-exp(-(vel_delta/delta_s));
		
	}
// In current lane dont consider cars behind you;
if(!is_current){
	if(nearest_car_behind!=-1){
		double veh_s=sensor_fusion[nearest_car_behind][5];
		double vx=sensor_fusion[nearest_car_behind][3];
		double vy=sensor_fusion[nearest_car_behind][4];
		double veh_vel=sqrt(vx*vx+vy*vy);

		veh_s+=(double(prev_size)*(0.02*veh_vel)+0.05*(veh_vel));
		
		double vel_delta;
		if(veh_vel<=car_vel) vel_delta=0.0001;
		else vel_delta=veh_vel-car_vel;
		
		double delta_s=abs(car_s-veh_s);
		
		if(abs(delta_s)<12) cost+=10;
		cost+=1-exp(-(vel_delta/delta_s));
	}
}
	
	return cost;

}

```
