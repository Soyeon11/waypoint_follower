#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <waypoint_maker/Lane.h>
#include <std_msgs/Int32.h>
#include <waypoint_maker/State.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_avoidance/DynamicControl.h>
#include <stop_line_detector/StopControl.h>

#include <vector>
#include <cmath>
#include <math.h>

using namespace std;

class WaypointFollower {
private:
double prev_steer_;
bool isfirst_steer_;
int waypoints_size_;
int mission_state_;

// 0. initial state
// 1. static avoidance
// 2. normal state
// 3. intersection A
/*======================*/
// 4. schoolzone state
// 5. intersection B
/*======================*/
// 6. dynamic avoidance
// 7. intersection C
// 8. intersection D
// 9. goal point

vector<waypoint_maker::Waypoint> waypoints_;

double init_speed_;
double decelate_speed_;
double accelate_speed_;

double lookahead_dist_;
double init_lookahead_dist_;
double decelate_lookahead_dist_;
double accelate_lookahead_dist_;

int current_mission_state_;
int next_mission_state_;
int next_mission_index_;
int next_waypoint_index_;
int target_index_;
int waypoint_target_index_;

int first_state_index_;
int second_state_index_;
int third_state_index_;
int fourth_state_index_;
int fifth_state_index_;
int sixth_state_index_;
int seventh_state_index_;
int eighth_state_index_;
int nineth_state_index_;

bool is_pose_;
bool is_course_;
bool is_lane_;
bool is_traffic_light_;
bool is_state_change_;
bool is_control_;
bool is_first_avoidance_;

bool is_stop_;
bool is_go_;
bool is_gl_;
bool is_sl_;

bool is_dynamic_control_;
bool is_brake_;

bool is_stop_control_;
bool is_stop_line_;


geometry_msgs::PoseStamped cur_pose_;

double cur_course_;
              
ros::NodeHandle nh_;
ros::NodeHandle private_nh_;

ros::Publisher ackermann_pub_;
ros::Publisher state_pub_;
ros::Publisher dynamic_control_pub_;
ros::Publisher stop_control_pub_;

ros::Subscriber pose_sub_;
ros::Subscriber course_sub_;
ros::Subscriber lane_sub_;
ros::Subscriber traffic_light_sub_;
ros::Subscriber stop_control_sub_;
ros::Subscriber dynamic_control_sub_;

ackermann_msgs::AckermannDriveStamped ackermann_msg_;
waypoint_maker::State state_msg_;
dynamic_avoidance::DynamicControl dynamic_control_msg_;
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
	state_pub_ = nh_.advertise<waypoint_maker::State>("target_state", 10);
	dynamic_control_pub_ = nh_.advertise<dynamic_avoidance::DynamicControl>("dynamic_operate", 10);
	stop_control_pub_ = nh_.advertise<stop_line_detector::StopControl>("stop_operate", 10);

	
	stop_control_sub_ = nh_.subscribe("stop_line",10,&WaypointFollower::StopControlCallback,this);
        pose_sub_ = nh_.subscribe("current_pose", 10, &WaypointFollower::PoseCallback, this);
        course_sub_ = nh_.subscribe("course", 10, &WaypointFollower::CourseCallback, this);
        lane_sub_ = nh_.subscribe("final_waypoints", 10, &WaypointFollower::LaneCallback, this);
       // traffic_light_sub_ = nh_.subscribe("traffic_light", 10, &WaypointFollower::TrafficLightCallback, this);
	dynamic_control_sub_ = nh_.subscribe("dynamic_brake", 10, &WaypointFollower::DynamicControlCallback, this);

	private_nh_.getParam("/waypoint_follower_node/init_speed", init_speed_);
	private_nh_.getParam("/waypoint_follower_node/decelate_speed", decelate_speed_);
        private_nh_.getParam("/waypoint_follower_node/accelate_speed", accelate_speed_);
        private_nh_.getParam("/waypoint_follower_node/init_lookahead_distance", init_lookahead_dist_);
        private_nh_.getParam("/waypoint_follower_node/accelate_lookahead_distance", accelate_lookahead_dist_);
        private_nh_.getParam("/waypoint_follower_node/decelate_lookahead_distance", decelate_lookahead_dist_);
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

	ROS_INFO("WAYPOINT FOLLOWER INITIALIZED.");

	is_dynamic_control_ = false;
	is_brake_ = false;

	is_stop_control_ = false;
	is_stop_line_ = false;

        isfirst_steer_ = true;
        prev_steer_ = 0;

	next_mission_state_ = current_mission_state_ + 1;
        is_pose_ = false;
        is_course_ = false;
        is_lane_ = false;
	is_traffic_light_ = false;
	is_state_change_ = false;
	is_control_ = true;
	is_first_avoidance_ = true;

	// traffic sign variables
	is_stop_ = false;
	is_go_ = false;
	is_gl_ = false;
	is_sl_ = false;
}

float calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
        float dist = sqrtf(powf(pose1.pose.position.x - pose2.pose.position.x, 2) + powf(pose1.pose.position.y - pose2.pose.position.y, 2));
        return dist;
}
void StopControlCallback(const stop_line_detector::StopControl::ConstPtr &stop_control_msg) {
	is_stop_line_ = stop_control_msg->is_stop_line;

	is_stop_control_ = true;
}


void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
	cur_pose_ = *pose_msg;
        is_pose_ = true;
}

void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
	cur_course_ = course_msg->drive.steering_angle;
        is_course_ = true;
}

void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg) {
        waypoints_.clear();
        waypoints_ = lane_msg->waypoints;
        waypoints_size_ = waypoints_.size();

	is_lane_ = true;
	
	for(int i=0;i<waypoints_size_;i++) {
		int index = waypoints_[i].waypoint_index;
		if(index == first_state_index_ || index == second_state_index_ || index == third_state_index_ || index == fourth_state_index_ || index == fifth_state_index_ || index == sixth_state_index_ || index == seventh_state_index_ || index == eighth_state_index_ || index == nineth_state_index_) {
			next_mission_state_ = waypoints_[i].mission_state;
			next_mission_index_ = index;
			next_waypoint_index_ = i;
			is_state_change_ = true;
			ROS_INFO("%d STATE CHANGE DETECTED.", next_mission_state_);
			return;
		}	
	}
}

/*void TrafficLightCallback(const traffic_sign_checker::TrafficLight::ConstPtr &traffic_light_msg) {
	is_stop_ = false;
	is_go_ = false;
	is_gl_ = false;
	is_sl_ = false;
	
	is_stop_ = traffic_light_msg->is_stop;
	is_go_ = traffic_light_msg->is_go;
	is_gl_ = traffic_light_msg->is_gl;
	is_sl_ = traffic_light_msg->is_sl;

	is_traffic_light_ = true;
}*/

void DynamicControlCallback(const dynamic_avoidance::DynamicControl::ConstPtr &dynamic_control_msg) {
	is_brake_ = dynamic_control_msg->is_brake;
	
	is_dynamic_control_ = true;
}

double calcSteeringAngle() {
        for(int i=0;i<waypoints_size_;i++) {
                double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
                if(dist > lookahead_dist_) {
                        target_index_ = i;
			waypoint_target_index_ = waypoints_[i].waypoint_index;	
                        break;
                }
        }

	state_msg_.current_state = waypoints_[target_index_].mission_state;
	state_pub_.publish(state_msg_);

        double steering_angle;

        double target_x = waypoints_[target_index_].pose.pose.position.x;
        double target_y = waypoints_[target_index_].pose.pose.position.y;

        ROS_INFO("TARGET X=%f", target_x);
        ROS_INFO("TARGET Y=%f", target_y);

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

void process() {
	is_control_ = true;
	dynamic_control_msg_.is_operate = false;
	stop_control_msg_.is_operate = false;

	double speed;
        if(is_pose_ && is_course_ && is_lane_ /*&& is_traffic_light_ && is_dynamic_control_ && is_stop_control_*/) {
                if(is_state_change_) {
                	double dist = calcPlaneDist(cur_pose_, waypoints_[next_waypoint_index_].pose);
			ROS_INFO("CURRENT POSE X=%f, Y=%f", cur_pose_.pose.position.x, cur_pose_.pose.position.y);
			ROS_INFO("CURRENT TARGET X=%f, Y=%f", waypoints_[next_waypoint_index_].pose.pose.position.x, waypoints_[next_waypoint_index_].pose.pose.position.y);
			ROS_INFO("CURRENT TARGET INDEX=%d, MISSION_INDEX=%d, DIST=%f", next_mission_index_, next_mission_state_, dist);	
			if(dist < 1.5 && next_mission_state_ ==	1) {
				ROS_INFO("PASS CONTROL TO STATIC AVOIDANCE NODE.");
				is_control_ = false;
				if(is_first_avoidance_) {
                			ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_first_avoidance_ = false;
				}
			}
			else if(next_mission_state_ == 2) {
				if(dist > 1.5) is_control_ = false;
				else {
					ROS_INFO("WITHDRAWAL CONTROL FROM STATIC AVOIDANCE NODE.");
					is_first_avoidance_ = true;
				}
			}
			else if(dist < 10.0 && next_mission_state_ == 3) {
				stop_control_msg_.is_operate = true;
				if(is_stop_ && is_stop_line_){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR FORWARD SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("FORWARD SIGN DETECTED.");
			}
			else if(dist < 2.0 && next_mission_state_ == 4) {
				ROS_INFO("ENTERING SCHOOL ZONE. DECELATION.");
				speed = decelate_speed_;
			}
			else if(dist < 10.0 && next_mission_state_ == 5) {
				stop_control_msg_.is_operate = true;
				if(is_stop_ && is_stop_line_){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR LEFT SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("LEFT SIGN DETECTED.");
			}
			else if(dist < 5.0 && next_mission_state_ == 6) {	
				ROS_INFO("ENTERING DYNAMIC AVOIDANCE MISSION STATE, DETECTING OBSTACLE.");	
				dynamic_control_msg_.is_operate = true;
				if(is_brake_) {
					ROS_INFO("WAITING FOR OBSTACLE REMOVED.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}	
					
			}
			else if(dist < 10.0 && next_mission_state_ == 7) {
				stop_control_msg_.is_operate = true;
				ROS_INFO("HI");
				if(is_stop_ && is_stop_line_){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR LEFT SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("LEFT SIGN DETECTED.");
			}
			else if(dist < 10.0 && next_mission_state_ == 8) {
				stop_control_msg_.is_operate = true;
				if(is_stop_ && is_stop_line_){
					ROS_INFO("STOP SIGN DETECTED. WAITING FOR FORWARD SIGN.");
					ackermann_msg_.header.stamp = ros::Time::now();
              				ackermann_msg_.drive.speed = 0.0;
                			ackermann_msg_.drive.steering_angle = 0.0;

                			ackermann_pub_.publish(ackermann_msg_);
					is_control_ = false;
				}
				else ROS_INFO("FORWARD SIGN DETECTED.");
			}
			else if(dist < 1.0 && next_mission_state_ == 9){
					ROS_INFO("GOAL POINT READCHED. TERMINATING WAYPOINT FOLLOWER.");		
					
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
			
                        if(((waypoint_target_index_ > fourth_state_index_) && (waypoint_target_index_ < sixth_state_index_))) {
				speed = decelate_speed_;
                                lookahead_dist_ = decelate_lookahead_dist_;
			}
                        else if((waypoints_[target_index_].mission_state)==8) {
                                speed = accelate_speed_;
                                lookahead_dist_ = accelate_lookahead_dist_;
                        }
                        else {
                            speed = init_speed_;
                            lookahead_dist_ = init_lookahead_dist_;
                        }
			if(is_brake_){
				speed = 0.0;
				cur_steer = 0.0;
			}

	                ROS_INFO("SPEED=%f, STEER=%f", speed, cur_steer);

			ackermann_msg_.header.stamp = ros::Time::now();
              		ackermann_msg_.drive.speed = speed;
               		ackermann_msg_.drive.steering_angle = cur_steer;

                	ackermann_pub_.publish(ackermann_msg_);
                }

		dynamic_control_pub_.publish(dynamic_control_msg_);
		stop_control_pub_.publish(stop_control_msg_);
		
		is_pose_ = false;
                is_course_ = false;
                is_lane_ = false;
		is_traffic_light_ = false;
		is_dynamic_control_ = false;	
		is_stop_control_ = false;
		is_state_change_ = false;
	}
        else {
                // ROS_INFO("SOME TOPIC NOT READY YET.");
        }
}
};

int main(int argc, char **argv) {
        ros::init(argc, argv, "waypoint_follower");
        WaypointFollower wf;
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		wf.process();
		ros::spinOnce();

		loop_rate.sleep();
	}
	
	return 0;
}
