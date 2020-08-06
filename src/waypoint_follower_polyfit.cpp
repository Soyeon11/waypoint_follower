#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_avoidance/DynamicControl.h>
#include <stop_line_detector/StopControl.h>
#include <std_msgs/Bool.h>


#include <vector>
#include <cmath>
#include <math.h>
#include "ros/time.h"

using namespace std;

class WaypointFollower {
private:
double prev_steer_;
bool isfirst_steer_;
bool parking_trigger_;
int waypoints_size_;
int mission_state_;
//int parking_point_;
//int after_parking_point_;

// 0. before parking state
// 1. after parking state(backward)
// 2. intersection state A
/*======================*/
// 3. intersection state B
// 4. normal state
// 5. static avoidance state
// 6. normal state(제어권 회수.)
// 7. intersection state C

/*======================*/
// 8. intersection state D
// 9. intersection state E
// 10. intersection state F
// 11. intersection state G
// 12. goalpoint

vector<waypoint_maker::Waypoint> waypoints_;


double init_speed_;
double decelate_speed_;
double accelate_speed_;
double parking_speed_;
double backward_movement_speed_;

double lookahead_dist_;
double init_lookahead_dist_;
double decelate_lookahead_dist_;
double accelate_lookahead_dist_;
double waypoints_polyfit_;

int current_mission_state_;
int next_mission_state_;
int next_mission_index_;
int next_waypoint_index_;
int target_index_;
int waypoint_target_index_;
int lane_number_;

int first_state_index_;
int second_state_index_;
int third_state_index_;
int fourth_state_index_;
int fifth_state_index_;
int sixth_state_index_;
int seventh_state_index_;
int eighth_state_index_;
int nineth_state_index_;
int tenth_state_index_;
int eleventh_state_index_;
int twelveth_state_index_;

int stop_line;

int parking_count_;

bool is_backward_;
bool is_retrieve_;

bool is_pose_;
bool is_course_;
bool is_lane_;
bool is_traffic_light_;
bool is_state_change_;
bool is_control_;
bool is_stop_;
bool is_go_;
bool is_gl_;
bool is_stop_line_;
bool is_stop_control_;
bool is_parking_area_;
bool is_parking_test_;


geometry_msgs::PoseStamped cur_pose_;

double cur_course_;
              
ros::NodeHandle nh_;
ros::NodeHandle private_nh_;
ros::Publisher ackermann_pub_;
ros::Publisher index_pub_;
ros::Publisher stop_control_pub_;

ros::Subscriber stop_control_sub_;

ros::Subscriber pose_sub_;
ros::Subscriber course_sub_;
ros::Subscriber lane_sub_;
ros::Subscriber traffic_light_sub_;
ros::Subscriber parking_area_sub_;

ackermann_msgs::AckermannDriveStamped ackermann_msg_;
//std_msgs::Bool operate_msg_;
waypoint_maker::Waypoint index_msg_;
stop_line_detector::StopControl stop_control_msg_;

public:
WaypointFollower() {
        initSetup();
}

~WaypointFollower() {
        waypoints_.clear();
}

void initSetup() {
        ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);
	index_pub_ = nh_.advertise<waypoint_maker::Waypoint>("target_state", 10);
	stop_control_pub_ = nh_.advertise<stop_line_detector::StopControl>("stop_operate", 10);

        pose_sub_ = nh_.subscribe("current_pose", 10, &WaypointFollower::PoseCallback, this);
        course_sub_ = nh_.subscribe("course", 10, &WaypointFollower::CourseCallback, this);
        lane_sub_ = nh_.subscribe("final_waypoints", 10, &WaypointFollower::LaneCallback, this);
       // traffic_light_sub_ = nh_.subscribe("traffic_light", 10, &WaypointFollower::TrafficLightCallback, this);
	stop_control_sub_ = nh_.subscribe("stop_line",10,&WaypointFollower::StopControlCallback,this);
	parking_area_sub_ = nh_.subscribe("parking_area",10,&WaypointFollower::ParkingAreaCallback,this);


	private_nh_.getParam("/waypoint_follower_node/init_speed", init_speed_);
        private_nh_.getParam("/waypoint_follower_node/decelate_speed", decelate_speed_);
        private_nh_.getParam("/waypoint_follower_node/accelate_speed", accelate_speed_);
	private_nh_.getParam("/waypoint_follower_node/backward_movement_speed", backward_movement_speed_);

        private_nh_.getParam("/waypoint_follower_node/init_lookahead_distance", init_lookahead_dist_);
        private_nh_.getParam("/waypoint_follower_node/decelate_lookahead_distance", decelate_lookahead_dist_);
        private_nh_.getParam("/waypoint_follower_node/accelate_lookahead_distance", accelate_lookahead_dist_);

	private_nh_.getParam("/waypoint_follower_node/current_mission_state", current_mission_state_);
	
	private_nh_.getParam("/waypoint_follower_node/first_state_index", first_state_index_);
	private_nh_.getParam("/waypoint_follower_node/second_state_index", second_state_index_);
	private_nh_.getParam("/waypoint_follower_node/third_state_index", third_state_index_);
	private_nh_.getParam("/waypoint_follower_node/fourth_state_index", fourth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/fifth_state_index", fifth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/sixth_state_index", sixth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/seventh_state_index", seventh_state_index_);
	private_nh_.getParam("/waypoint_follower_node/eighth_state_index", eighth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/nineth_state_index", nineth_state_index_);	
	private_nh_.getParam("/waypoint_follower_node/tenth_state_index", tenth_state_index_);
	private_nh_.getParam("/waypoint_follower_node/eleventh_state_index", eleventh_state_index_);
        private_nh_.getParam("/waypoint_follower_node/twelveth_state_index", twelveth_state_index_);

	ROS_INFO("WAYPOINT FOLLOWER INITIALIZED.");


	parking_count_ = 0;

        isfirst_steer_ = true;
        prev_steer_ = 0;

	next_mission_state_ = current_mission_state_ + 1;
        is_pose_ = false;
        is_course_ = false;
        is_lane_ = false;
	is_traffic_light_ = false;
	is_state_change_ = false;
	is_control_ = true;
	parking_trigger_ = false;
	is_backward_ = false;
	is_retrieve_ = false;
        is_stop_line_= false;
        is_stop_control_= false;
	is_parking_area_ = true; //is_parking_area true이면 경로 유지
	is_parking_test_ = false; //callback이 들어오면 true
	stop_line = 0;

	// traffic sign variables
	is_stop_ = false;
	is_go_ = false;
	is_gl_ = false;

	lane_number_ = 0;
}

float calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
        float dist = sqrtf(powf(pose1.pose.position.x - pose2.pose.position.x, 2) + powf(pose1.pose.position.y - pose2.pose.position.y, 2));
        return dist;
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
	cur_pose_ = *pose_msg;
        is_pose_ = true;
}

void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
	cur_course_ = course_msg->drive.steering_angle;
        is_course_ = true;
}
void StopControlCallback(const stop_line_detector::StopControl::ConstPtr &stop_control_msg) {
	is_stop_line_ = stop_control_msg->is_stop_line;

	is_stop_control_ = true;
}

void ParkingAreaCallback(const std_msgs::Bool::ConstPtr &parking_area_msg) {
	is_parking_area_ = parking_area_msg->data;
	is_parking_test_ = true;

}

void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg) {
        waypoints_.clear();
        waypoints_ = lane_msg->waypoints;
        waypoints_size_ = waypoints_.size();
	//stop_line = lane_msg->stop_line;

        // ROS_INFO("%d WAYPOINTS RECEIVED.", waypoints_size_);
        
	is_lane_ = true;

	ROS_INFO("-------------------------------------------");
	for(int i=0;i<waypoints_size_;i++) {
		ROS_INFO("%d WAYPOINTS INDEX=%d", i, waypoints_[i].waypoint_index);
	}
	ROS_INFO("-------------------------------------------");
	

        for(int i=0;i<waypoints_size_;i++) {
		int index = waypoints_[i].waypoint_index;
                if( (index == first_state_index_ || index == second_state_index_ || index == third_state_index_ || index == fourth_state_index_ || index == fifth_state_index_ || index == sixth_state_index_ || index == seventh_state_index_ || index == eighth_state_index_ || index == nineth_state_index_ || index == tenth_state_index_ || index == eleventh_state_index_ || index == twelveth_state_index_)) {
			next_mission_state_ = waypoints_[i].mission_state;
			next_mission_index_ = index;
			next_waypoint_index_ = i;
			is_state_change_ = true;
			ROS_INFO("%d STATE CHANGE DETECTED LOOP 1.", next_mission_state_);
			break;
		}

        }

	if (waypoints_size_ > 10) {		
		tranformUtmtoLocal();		
		waypoints_polyfit_ = calcWaypointsPolyfit();
	}
}

/*void TrafficLightCallback(const traffic_sign_checker::TrafficLight::ConstPtr &traffic_light_msg) {
	is_stop_ = false;
	is_go_ = false;
	is_gl_ = false;

	is_stop_ = traffic_light_msg->is_stop;
	is_go_ = traffic_light_msg->is_go;
	is_gl_ = traffic_light_msg->is_gl;

	is_traffic_light_ = true;
}*/

double calcSteeringAngle() {

        for(int i=0;i<waypoints_size_;i++) {
                double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
		//lookahead_dist_ = 1.5;
                if(dist>lookahead_dist_){
                    target_index_=i;
                    waypoint_target_index_ = waypoints_[i].waypoint_index;
		    ROS_INFO("target_index: %d ld: %f",target_index_,lookahead_dist_);
                    if(/*is_retrieve_||*/ parking_count_ == 1 ){
                        if((waypoints_[i].waypoint_index - waypoints_[0].waypoint_index == 1) || (waypoints_[i].waypoint_index - waypoints_[0].waypoint_index == 2  )) {
                            ROS_INFO("1");
                            target_index_ = i;
                            waypoint_target_index_ = waypoints_[i].waypoint_index;
                            break;
                        }
                    }
		    if(waypoints_[i].mission_state == 3){
			    is_retrieve_ = false;}
		    break;
                }

        }
	
        if(parking_trigger_ || is_retrieve_){
		if(cur_course_ < 180) cur_course_ += 180;
		else cur_course_ -= 180;
		ROS_INFO("CURRENT COURSE INVERTED.");
	}

        double steering_angle;

        double target_x = waypoints_[target_index_].pose.pose.position.x;
        double target_y = waypoints_[target_index_].pose.pose.position.y;

        ROS_INFO("TARGET X=%f, TARGET Y=%f", target_x, target_y);

        double dx = target_x - cur_pose_.pose.position.x +0.000000001;
        double dy = target_y - cur_pose_.pose.position.y;

        double heading = atan(dy/dx);
        double angle = heading * 180 / 3.141592;
        double true_angle;

        if(dx>=0 && dy>0) true_angle = 90.0 - angle;
        else if(dx>=0 && dy<0) true_angle = 90.0 - angle;
        else if(dx<0 && dy<0) true_angle = 270.0 - angle;
        else if(dx<0 && dy>0) true_angle = 270.0 - angle;

        double cur_steer = true_angle - cur_course_ ;

        if(cur_steer<-180) cur_steer = 360 -cur_course_ + true_angle;
        else if(cur_steer>180) cur_steer = true_angle - cur_course_ -360 ;
	

        if(isfirst_steer_) {
                prev_steer_ = cur_steer;
                isfirst_steer_ = false;
        }
        else {
                if(abs(cur_steer - prev_steer_) < 5.0) cur_steer = prev_steer_;
        }

        return cur_steer;
}

void transformUtmToLocal() {
	
	float x_list[15] ={0,};
	float y_list[15] ={0,};

	for (int i=0;i<15;i++){
		float local_x = 0;
		float local_y = 0;

		float point_x = 0;
		float point_y = 0;

		float local_r = 0;
		float local_theta = 0;

		int point_x = waypoints_[i].pose.pose.position.x - cur_pose_.pose.position.x;
		int point_y = waypoints_[i].pose.pose.position.y - cur_pose_.pose.position.y;

		if (point_x = 0) point_x =0.000001;
		if (point_y = 0) point_y =0.000001;	

	//transform to r,theta system
		int local_r= sqrtf(powf(point_x, 2) + powf(point_y, 2));

		if ((point_x > 0) && (point_y > 0)){
			local_theta = atan(point_x/point_y) - cur_course_;
		}
		else if ((point_x > 0) && (point_y < 0)){
			local_theta = 90 - atan(point_y/point_x)  - cur_course_;
		}
		else if ((point_x < 0) && (point_y < 0)){
			local_theta = 180 + atan(point_x/point_y) - cur_course_;
		}
		else if ((point_x < 0) && (point_y > 0)){
			local_theta = 360 + atan(point_x/point_y) - cur_course_;
		}
		
	
		if (local_theta > 270) local_theta = local_theta - 360;
		if (local_theta < 270) local_theta = local_theta + 360;
	
	
		local_x = local_r * cos(local_theta);
		local_y = -(local_r * sin(local_theta));
	
		x_list[i] = local_x;
		y_list[i] = local_y;

        return;
}

double calcWaypointsPolyfit() {
	
	int i,j,k,n,N;
    	cout.precision(4);                        //set precision
    	cout.setf(ios::fixed);
	N = 15;
	n = 2;                             // n is the degree of Polynomial 
    	double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    	for (i=0;i<2*n+1;i++){
        	X[i]=0;
        	for (j=0;j<N;j++) 
			X[i]=X[i]+pow(x_list[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    	}
    	double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    	for (i=0;i<=n;i++)
        	for (j=0;j<=n;j++)
        		B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    	double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    	for (i=0;i<n+1;i++){    
        	Y[i]=0;
        	for (j=0;j<N;j++) 
			Y[i]=Y[i]+pow(x_list[j],i)*y_list[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    	}
    	for (i=0;i<=n;i++)
        	B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    	n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    	for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        	for (k=i+1;k<n;k++)
            		if (B[i][i]<B[k][i])
                		for (j=0;j<=n;j++){
                    			double temp=B[i][j];
                    			B[i][j]=B[k][j];
                    			B[k][j]=temp;
                		}
     
    	for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
        	for (k=i+1;k<n;k++){
                	double t=B[k][i]/B[i][i];
                	for (j=0;j<=n;j++)
                    		B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            	}
    	for (i=n-1;i>=0;i--)                //back-substitution
	{                        //x is an array whose values correspond to the values of x,y,z..
        	a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        	for (j=0;j<n;j++)
            		if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
               			a[i]=a[i]-B[i][j]*a[j];
       		a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    	}
  
    	cout<<"\nHence the fitted Polynomial is given by:\ny=";
    	for (i=0;i<n;i++)
        	cout<<" + ("<<a[i]<<")"<<"x^"<<i;
    	cout<<"\n";
	double waypoints_polyfit = a[2];

        return waypoints_polyfit;
}

void process() {
	is_control_ = true;
        //operate_msg_.data=false;
        double speed;
	double dist;
        if(is_pose_ && is_course_ && is_lane_ /*&& is_traffic_sign_&&is_stopline_*/) {
                if(is_state_change_) {
                	dist = calcPlaneDist(cur_pose_, waypoints_[next_waypoint_index_].pose);
			// ROS_INFO("CURRENT POSE X=%f, Y=%f", cur_pose_.pose.position.x, cur_pose_.pose.position.y);
			// ROS_INFO("CURRENT TARGET X=%f, Y=%f", waypoints_[next_waypoint_index_].pose.pose.position.x, waypoints_[next_waypoint_index_].pose.pose.position.y);
			ROS_INFO("CURRENT TARGET STATE INDEX=%d, MISSION_INDEX=%d, DIST=%f, PARKING_COUNT=%d", next_mission_index_, next_mission_state_, dist,parking_count_);	
			

                        if(dist < 2.0 && next_mission_state_ == 1) {
				while(1){
					ROS_INFO("PARKING SIGN DETECTED. WAITING FOR SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					
					if(!is_parking_area_){
						lane_number_ += 1;
						break;
					}

					else if(is_parking_area_){
						break;
					}
				}

			}
		
                        else if( dist < 1.5 && next_mission_state_ == 2) {

				if(parking_count_ == 0) {
					parking_trigger_ = true;
					parking_count_++;
					is_backward_ = true;
					ROS_INFO("CURRENTLY ARRIVED AT PARKING POINT.");
					ROS_INFO("STOP FOR 4SECONDS.");

					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);


					is_control_ = false;
					ros::Time::init();
					ros::Duration(7,0).sleep();

					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);

				}				
				is_control_ = true;


			}

			else if( dist < 2.5 && next_mission_state_ == 3) {
				ROS_INFO("PARKING MISSION IS DONE.");
				if(parking_count_ == 1) {

					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);

                                        parking_trigger_ = false;
					parking_count_++;
					is_retrieve_ = true;
                                        is_backward_=false;					
				}

			}
			
                        else if( dist < 2.0 && next_mission_state_ == 4) {
                            //operate_msg_.data=true;
                            //if(is_gl_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
					//ros::shutdown();
				//}
				//else ROS_INFO("FORWARD SIGN DETECTED.");

			}
			
                        else if(dist < 6.0 && next_mission_state_ == 5) {
                            //operate_msg_.data=true;
                            if(!is_go_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR FORWARD SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("FORWARD SIGN DETECTED.");

				ROS_INFO("NORMAL STATE UNTIL AVOIDANCE.");
				
			}

			else if(dist < 2.0 && next_mission_state_ == 6) {
					ROS_INFO("PASS CONTROL TO STATIC AVOIDANCE NODE.");
					is_control_ = false;

					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
	
			}


                        else if(dist<6.0 && next_mission_state_ == 7){
                                //operate_msg_.data=true;
                                if(!is_go_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR FORWARD SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("FORWARD SIGN DETECTED.");


			}
                        else if(dist < 6.0 && next_mission_state_ == 8) {
                                //operate_msg_.data=true;
                                if(!is_gl_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR LEFT SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("LEFT SIGN DETECTED.");
			}

                        else if(dist < 6.0 && next_mission_state_ == 9) {
                                //operate_msg_.data=true;
                                if(!is_gl_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR LEFT SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("LEFT SIGN DETECTED.");
			}

                        else if(dist < 6.0 && next_mission_state_ == 10) {
                                //operate_msg_.data=true;
                                if(!is_go_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR FORWARD SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("FORWARD SIGN DETECTED.");

			}

                        else if(dist < 6.0 && next_mission_state_ == 11) {
                                //operate_msg_.data=true;
                                if(!is_go_&&(stop_line==1)){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR FORWARD SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("FORWARD SIGN DETECTED.");

			}

			else if(dist < 1.0 && next_mission_state_ == 12){
					ROS_INFO("GOAL POINT READCHED. TERMINATING WAYPOINT 	FOLLOWER.");		
					
					is_control_ = false;
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;
                			
					ackermann_pub_.publish(ackermann_msg_);
					
					ros::shutdown();
			}
			
		}

                if(is_control_) {
			
			double cur_steer = calcSteeringAngle();
			
                        if((parking_count_==0)&&(is_backward_==false)){
                                speed = decelate_speed_;
                                lookahead_dist_= decelate_lookahead_dist_;
                        }
                        else if(parking_count_ == 1 || is_backward_ ) {
				speed = backward_movement_speed_;
				cur_steer = -cur_steer;
			}
                        else if(parking_count_ ==2) {
                            	speed = init_speed_;
				lookahead_dist_=5.0;
			}
                        else if((waypoints_[target_index_].mission_state)==10) {
                                speed=accelate_speed_;
                                lookahead_dist_=accelate_lookahead_dist_;
                        }

                        else {
                            speed = init_speed_;
                            lookahead_dist_=init_lookahead_dist_;
                        }


                        ROS_INFO("SPEED=%f, STEER=%f", speed, cur_steer);

			ackermann_msg_.header.stamp = ros::Time::now();
              		ackermann_msg_.drive.speed = speed;
               		ackermann_msg_.drive.steering_angle = cur_steer;

                	ackermann_pub_.publish(ackermann_msg_);
                }
                //stop_control_pub_.publish(operate_msg_);
		parking_trigger_ = false;	
		is_pose_ = false;
                is_course_ = false;
                is_lane_ = false;
		is_traffic_light_ = false;	
		is_state_change_ = false;
                is_retrieve_ = false;
		is_parking_test_ = false;
		
		index_msg_.waypoint_index = waypoints_[target_index_].waypoint_index;
		index_msg_.lane_number = lane_number_;
		index_msg_.mission_state = waypoints_[target_index_].mission_state;
		index_pub_.publish(index_msg_);
        }

}
};

int main(int argc, char **argv) {
        ros::init(argc, argv, "waypoint_follower");
        WaypointFollower wf;
	
	while(ros::ok()) {
		wf.process();
		ros::spinOnce();
	}
	
	return 0;
}

