//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1
#define PI 3.14159265358979323846

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <ctime>
#include <chrono>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//map spec
cv::Mat map;
cv::Mat map_margin;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 6;
int K = 5000;
double MaxStep = 2;

//way points
std::vector<point> waypoints;

//path
//std::vector<point> path_RRT;
std::vector<traj> path_RRT;

std::vector<point> entering;

std::vector<int> path_length;
//control
//std::vector<control> control_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
gazebo_msgs::ModelStatesConstPtr model_states;
sensor_msgs::ImageConstPtr images;
cv::Mat dImg;
std::vector<float> scan;
float steer;
//FSM state
int state;

//function definition
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_image(sensor_msgs::ImageConstPtr msgs);
void callback_laser(sensor_msgs::LaserScanConstPtr msgs);
void callback_steer_network(std_msgs::Float32 temp);
void setcmdvel(double v, double w);
int main(int argc, char** argv){
    srand((unsigned int)time(0));
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;
 
    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states", 1, callback_state);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, callback_laser);
    ros::Subscriber image_sub = n.subscribe("/camera/zed/rgb/image_rect_color", 1, callback_image);
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/output", 1);
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	ros::Subscriber steer_by_network = n.subscribe("steering_by_network",1,callback_steer_network);
    printf("Initialize topics\n");

    // Load Map
    //char* user = getlogin();
    map = cv::imread((std::string("/home/wooseoko/")+
                      std::string("catkin_ws/src/project2/worlds/large_world1.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -16;
    world_x_max = 16;
    world_y_min = -16;
    world_y_max = 16;
    /*
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    */
    res = 0.05;
    printf("Load map\n");

    if(!map.data) {
        printf("Could not open or find the image\n");
        return -1;
    }
	point x;
	point y;
    rrtTree *tree = new rrtTree(x, y, map, map_origin_x, map_origin_y, res, 25);
	map_margin = tree->map;
	delete tree;

    ros::Rate control_rate(60);
    for(int iter = 0; iter < 2000; iter++) {

		while(1){
		set_waypoints();
//			if( ( (waypoints[0].x - waypoints[1].x) * (waypoints[0].x - waypoints[1].x)  + (waypoints[0].y - waypoints[1].y) * (waypoints[0].y - waypoints[1].y) ) > 64 &&  ( (waypoints[0].x - waypoints[2].x) * (waypoints[0].x - waypoints[2].x)  + (waypoints[0].y - waypoints[2].y) * (waypoints[0].y - waypoints[2].y) ) > 64 ) break;
//		}
//        printf("Set way points\n");


//		generate_path_RRT();

/*        bool valid = generate_path_RRT();
        if(valid == true) printf("Generate RRT\n");
        else {
            iter -= 1;
            printf("Cannot generate RRT\n");
            continue;
        }*/

        state = INIT;
        bool running = true;
//        int look_ahead_idx;
    
//        std::ostringstream oss;

//        std::ostringstream oss1;
//		int inum = rand() % 10000;
//        oss << "/home/wooseoko/catkin_ws/src/project2/data/" << inum;
//		oss1 << "/home/wooseoko/catkin_ws/src/project2/data/";
//        std::ostringstream text_name;
//        text_name << oss.str() <<  ".txt";
//    	std::ofstream ofs(text_name.str().c_str());
        while(running) {
            switch (state) {
            case INIT: {
//                look_ahead_idx = 0;
//	            printf("path size : %d\n", path_RRT.size());
                //visualize path
	            ros::spinOnce();
	
                //initialize robot position
                geometry_msgs::Pose model_pose;
                model_pose.position.x = waypoints[0].x;
                model_pose.position.y = waypoints[0].y;
                model_pose.position.z = 0.3;
                model_pose.orientation.x = 0.0;
                model_pose.orientation.y = 0.0;
                model_pose.orientation.z = -sin(-waypoints[0].th / 2);//0.0;
                model_pose.orientation.w = cos(-waypoints[0].th / 2);//1.0;

                geometry_msgs::Twist model_twist;
                model_twist.linear.x = 0.0;
                model_twist.linear.y = 0.0;
                model_twist.linear.z = 0.0;
                model_twist.angular.x = 0.0;
                model_twist.angular.y = 0.0;
                model_twist.angular.z = 0.0;

                gazebo_msgs::ModelState modelstate;
                modelstate.model_name = "racecar";
                modelstate.reference_frame = "world";
                modelstate.pose = model_pose;
                modelstate.twist = model_twist;

                gazebo_msgs::SetModelState setmodelstate;
                setmodelstate.request.model_state = modelstate;

                gazebo_set.call(setmodelstate);
                ros::spinOnce();
                ros::Rate(0.33).sleep();
		cmd.drive.steering_angle = 0;
                printf("Initialize ROBOT\n");
                state = RUNNING;
            } break;

            case RUNNING: {
//		        int current_goal = 1;
//           		PID pid_ctrl;
           		int timestep = 0;
           		int length = 0;
           		ros::spinOnce();
			    control_rate.sleep();
			cmd.drive.speed = 1.0;
		        while(ros::ok()) {
		            ros::spinOnce();
			        point temp_goal;
//			        temp_goal.x = path_RRT[current_goal].x;
//			        temp_goal.y = path_RRT[current_goal].y;
//			        temp_goal.th = path_RRT[current_goal].th;

			cmd.drive.steering_angle_velocity = 1.0;
			if(cmd.drive.steering_angle * steer < 0 )
			    cmd.drive.steering_angle = cmd.drive.steering_angle * 0.9 + steer * 0.1;
			else if(cmd.drive.steering_angle * steer > 0 )
			    cmd.drive.steering_angle = cmd.drive.steering_angle * 0.1 + steer * 0.9;
			else
			    cmd.drive.steering_angle = cmd.drive.steering_angle * 0.5 + steer * 0.5;
		    cmd_vel_pub.publish(cmd);

//			        float check_x = robot_pose.x - path_RRT[current_goal].x;
//			        float check_y = robot_pose.y - path_RRT[current_goal].y;
		    printf(" steer_desired : %.2f \n", steer);
		    printf(" steer : %.2f \n\n", cmd.drive.steering_angle);
//			        if(timestep % 1 == 0 || cmd.drive.steering_angle != 0.0) {
//			        length++;
			        //printf("angle: %f\n", cmd.drive.steering_angle);


//			    	if(ofs.is_open()) {
//			       		ofs << cmd.drive.steering_angle << ",";  
//	                }
  //  	            else printf("Unable to open file");
    //    	        ofs << "\n";
      //      	    std::ostringstream img_name;
//					if(cmd.drive.steering_angle > 0.5){
//                		img_name << oss1.str() << "1/" << "_" << inum << "_" << timestep << ".jpg";
//					}


//					else if(cmd.drive.steering_angle < -0.5){
//                		img_name << oss1.str() << "-1/" << "_"  << inum << "_" << timestep << ".jpg";
//					}

//					else{
//                		img_name << oss1.str() << "0/" << "_"  << inum << "_" << timestep << ".jpg";
//					}
//					cv::resize(dImg, dImg, cv::Size(), 0.25, 0.25);
//	                imwrite(img_name.str(), dImg);

                    
                    
                    /*
                    if(collision == true) {
                        iter--;
                        printf("collision! recollect!\n");
                        state = FINISH;
                        break;
                    }*/
                    
//                    timestep++;
/*                    if(timestep > 1000) {
                        iter--;
                        state = FINISH;
                        printf("time over!\n");
                        break;
                    }
                    */
/*			        if (fabs(check_x) < 1.0 && fabs(check_y) < 1.0) {
			            printf("arrived goal : %d with x : %f, y : %f \n", current_goal, fabs(check_x), fabs(check_y));
			            pid_ctrl.reset();
			            current_goal++;
			            if (current_goal == path_RRT.size()) {
				            printf("reached all point, total_length : %d\n", length);
			                state = FINISH;
				            break;
			            }
			        }*/
			        control_rate.sleep();
			        
				for(int iter = 0; iter < 1; iter++) {
			            ros::spinOnce();
			            cmd_vel_pub.publish(cmd);
			            control_rate.sleep();
			        }
		        }
            } break;

            case FINISH: {
                setcmdvel(0,0);
                cmd_vel_pub.publish(cmd);
                running = false;
                ros::spinOnce();
                control_rate.sleep();
            } break;

            default: {
            } break;
			}
        }
//        ofs.close();
		}
    }    
    return 0;
}


void generate_path_RRT()
{
    //TODO 1
	point start;
	int cnt = 0;
	start.x=0.0;
	start.y=0.0;
	start.th=0.0;


    path_RRT.clear();
	entering.clear();
		for(int i=0;i<waypoints.size();i++) 
		{
		    entering.push_back(start);
		    path_length.push_back(0);
		}	
//      printf("RRT\n");
        point lastp=waypoints[0];
		entering[0].th=waypoints[0].th;
		entering[0].x=waypoints[0].x;
		entering[0].y=waypoints[0].y;
		printf(" after entering \n");
        for(int i=0; i<waypoints.size()-1; i++){
//			lastp.x=waypoints[i].x;
//            lastp.y=waypoints[i].y;
            rrtTree *tree = new rrtTree(lastp, waypoints[i+1], map, map_origin_x, map_origin_y, res, margin);
			printf(" new rrt Tree \n");
            tree->generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
			printf(" generate RRT \n");
			std::vector<traj> vec;
			if(tree->nearestNeighbor(waypoints[i+1]) != 0){
	            vec = tree->backtracking_traj();
			}
			else{
				i--;
				cnt++;
				if(cnt>0){

					while(1){
						set_waypoints();

						if( ( (waypoints[0].x - waypoints[1].x) * (waypoints[0].x - waypoints[1].x)  + (waypoints[0].y - waypoints[1].y) * (waypoints[0].y - waypoints[1].y) ) > 64 &&  ( (waypoints[0].x - waypoints[2].x) * (waypoints[0].x - waypoints[2].x)  + (waypoints[0].y - waypoints[2].y) * (waypoints[0].y - waypoints[2].y) ) > 64 ) break;
					}
					cnt = 0;
					printf("re_ set _ waypoints \n");
					lastp=waypoints[0];

				}
				delete tree;
				continue;
			}
			printf(" vector traj vec \n");
//			printf(" %d\n", vec.size());
			printf(" %d\n", i);
			if( (vec.begin()->x-waypoints[i+1].x)*(vec.begin()->x-waypoints[i+1].x)+(vec.begin()->y-waypoints[i+1].y)*(vec.begin()->y-waypoints[i+1].y) > 1.0 ){
   			    printf("!!!REFIND THE WAY\n");
				cnt++;
			    printf(" x y %.2f %.2f waypoints %.2f %.2f\n", vec.begin()->x, vec.begin()->y,waypoints[i+1].x, waypoints[i+1].y);
			    if(i>0){ 
					i=i-2; 
			 		lastp.th = entering[i+1].th;
			 		lastp.x = entering[i+1].x;
			 		lastp.y = entering[i+1].y;
 					for(int j=0;j<path_length[i+1];j++) path_RRT.pop_back();
		    	}
				else i=i-1;
			}
//                std::reverse(vec.begin(),vec.end());
			else{
				printf(" else \n");
		    	lastp.x = vec.begin()->x;
                lastp.y = vec.begin()->y;
                lastp.th = vec.begin()->th;
		    	entering[i+1].th= lastp.th;		
				entering[i+1].x= lastp.x;		
		    	entering[i+1].y= lastp.y;		
				printf(" before visualize tree \n");
                tree->visualizeTree();
				printf("after visualize tree \n");
	    	    std::reverse(vec.begin(),vec.end());
				printf(" reverse \n");
                tree->visualizeTree(vec);
				printf(" visualize Tree (vec) \n");
		    	path_length[i] = vec.size();
				printf(" path_length \n");
                for(int j=0; j<vec.size();j++)
                	path_RRT.push_back(vec[j]);
			}
			printf("before delete tree \n");
            delete tree;
			printf("after delete tree \n");
			if(cnt>1){
				while(1){
					set_waypoints();

					if( ( (waypoints[0].x - waypoints[1].x) * (waypoints[0].x - waypoints[1].x)  + (waypoints[0].y - waypoints[1].y) * (waypoints[0].y - waypoints[1].y) ) > 64 &&  ( (waypoints[0].x - waypoints[2].x) * (waypoints[0].x - waypoints[2].x)  + (waypoints[0].y - waypoints[2].y) * (waypoints[0].y - waypoints[2].y) ) > 64 ) break;
				}
				cnt = 0;
				printf("re_ set _ waypoints \n");
				lastp=waypoints[0];

			}
        }
/*        traj lastpoint;
        lastpoint.x=waypoints[waypoints.size()-1].x;
        lastpoint.y=waypoints[waypoints.size()-1].y;
        lastpoint.th=waypoints[waypoints.size()-1].th; 
        path_RRT.push_back(lastpoint);*/
}



/*
bool generate_path_RRT() {
    rrtTree temp_tree;
    path_RRT.clear();
    traj temp_traj;
    temp_traj.x = waypoints[0].x;
    temp_traj.y = waypoints[0].y;
    temp_traj.th = waypoints[0].th;	
    path_RRT.push_back(temp_traj);
    for(int i = 0; i < waypoints.size() - 1; i++) {
        temp_tree = rrtTree(waypoints[i], waypoints[i+1], map, map_origin_x, map_origin_y, res, margin); 
	    int valid = temp_tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
	    if(valid == 0) return false;
	    std::vector<traj> temp_traj = temp_tree.backtracking_traj();
        if(temp_traj.size() > 0) {
		    point temp_point;
		    temp_point.x = temp_traj[0].x;
		    temp_point.y = temp_traj[0].y;
		    temp_point.th = temp_traj[0].th;
		    waypoints[i+1] = temp_point;
		    for(int j = 0; j < temp_traj.size(); j++) {
		        path_RRT.push_back(temp_traj[temp_traj.size() - j - 1]);
		    }
        }
    }
    temp_tree.visualizeTree(path_RRT);
    sleep(1);
    return true;
}
*/
void set_waypoints() {
    waypoints.clear();
    point waypoint_candid[8];

	while(1)
	{
		
		waypoint_candid[0].x = -16.0 + 32.0 * std::rand() / RAND_MAX;
	    waypoint_candid[0].y = -16.0 + 32.0 * std::rand() / RAND_MAX;
		waypoint_candid[0].th = -PI + 2 * PI * std::rand() / RAND_MAX;
		int x = waypoint_candid[0].x/res + map_origin_x;
		int y = waypoint_candid[0].y/res + map_origin_y;
		if(map_margin.at<uchar>(x,y) > 240)
			break;
	}
	while(1){
    	waypoint_candid[1].x = -16.0 + 32.0 * std::rand() / RAND_MAX;
    	waypoint_candid[1].y = -16.0 + 32.0 * std::rand() / RAND_MAX;
		int x = waypoint_candid[1].x/res + map_origin_x;
		int y = waypoint_candid[1].y/res + map_origin_y;
		if(map_margin.at<uchar>(x,y) > 240)
			break;

	}
	while(1){
    	waypoint_candid[2].x = -16.0 + 32.0 * std::rand() / RAND_MAX;
	    waypoint_candid[2].y = -16.0 + 32.0 * std::rand() / RAND_MAX;
		int x = waypoint_candid[2].x/res + map_origin_x;
		int y = waypoint_candid[2].y/res + map_origin_y;
		if(map_margin.at<uchar>(x,y) > 240)
			break;

	}
/*	waypoint_candid[0].x = 1;
	waypoint_candid[0].y = -1.5;
	waypoint_candid[0].th = 9*PI/5;
	waypoint_candid[1].x = -6.5;
	waypoint_candid[1].y = -6.5;
	waypoint_candid[2].x = -6.5;
	waypoint_candid[2].y = -6.5;*/
    int order[] = {0,1,2};
    //int order[] = {0,1,2,3,0};
    int order_size = 2;

    for(int i = 0; i < order_size; i++) {
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void callback_state(gazebo_msgs::ModelStatesConstPtr msgs) {
    model_states = msgs;
    for(int i; i < msgs->name.size(); i++) {
        if(std::strcmp(msgs->name[i].c_str(), "racecar") == 0) {
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void callback_image(sensor_msgs::ImageConstPtr msgs) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    images = msgs;
    dImg = cv_ptr->image;
    //int rows =  sizeof(images->data) / sizeof(images->data[0]); // 2 rows  
    //int cols = sizeof(images->data[0]) / sizeof(uint8_t);
}

void callback_laser(sensor_msgs::LaserScanConstPtr msgs) {
    scan = msgs->ranges;
}
void callback_steer_network(std_msgs::Float32 temp)
{
	steer = temp.data;
}
void setcmdvel(double vel, double deg) {
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}
