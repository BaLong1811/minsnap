#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <min_snap/min_snap_closeform.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "traj_utils/polynomial_traj.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class TrajectoryGenerator {
public:
    TrajectoryGenerator() {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Subscribe to UAV odometry
        odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, &TrajectoryGenerator::odomCallback, this); // og /uav/odom

        // Subscribe to goal position
        goal_sub_ = nh.subscribe("/goal_position", 10, &TrajectoryGenerator::goalCallback, this);

        // Subscribe to mavros state
        mavros_state_sub_ = nh.subscribe("/mavros/state", 10, &TrajectoryGenerator::stateCallback, this);

        // Publisher for trajectory
        traj_pub_ = nh.advertise<nav_msgs::Path>("/trajectory", 10);

        // Publisher for coefficients
        coef_pub_ = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_coefs", 10);

        // Publisher for points on trjectory
        pose_array_pub_ = nh.advertise<geometry_msgs::PoseArray>("/points", 10);

        // Publisher for take off/land
        takeoff_land_pub_ = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);

        // Generate trajectory timer
        timer_gen = nh.createTimer(ros::Duration(8.0), &TrajectoryGenerator::timerCallback, this);

        // publish position command
        pose_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/px4ctrl_cmd", 10);

        // publish posestamped command to mavros and gazebo simulator
        // mavros_cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        // Define path structure
        desired_path.header.frame_id = "map";
        desired_path.header.stamp = ros::Time::now();

        // Define pose array structure
        pose_array.header.frame_id = "map";
        pose_array.header.stamp = ros::Time::now();

        // Polynomial 
        poly_pub_topic.num_order = 7;
        poly_pub_topic.start_yaw = 0;
        poly_pub_topic.final_yaw = 0;
        poly_pub_topic.mag_coeff = 0;
        poly_pub_topic.order.push_back(0);

        // Command type
        nh.getParam("/trajectory_generator_node/command_type", cmd_type);
        cout << ("command type : ") << cmd_type << endl; 
        
        // Mean velocity
        nh.getParam("/trajectory_generator_node/mean_vel", mean_vel);
        cout << ("mean velocity : ") << mean_vel << endl; 

        count = 0;
        loop_count = 0;

        ros::Duration(2.0).sleep();   
    }

    // Check FCU connection
    void checkFCU()
    {
        while (!current_state.connected)
        {
            ROS_INFO("Waiting for FCU connection...");
            // cout << "FCU status: " << current_state.connected << endl;
            ros::Duration(0.2).sleep();
        }
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;

        // check FCU connection
        if (count == 0)
        {
            checkFCU();
            ROS_INFO("FCU connected!");

            setMode("OFFBOARD");

            armVehicle(true);

            takeoff_land_data.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
            takeoff_land_pub_.publish(takeoff_land_data);

            ros::Duration(10.0).sleep();

            count++;
            loop_count = 1;
        }

    }

    // Function to store values in a CSV file
    void storeValuesInCSV(const std::string& filename, const std::vector<Eigen::Vector3d>& data) 
    {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        for (const auto& vec : data) {
            file << vec[0] << "," << vec[1] << "," << vec[2] << "\n";
        }

        file.close();
        std::cout << "Data successfully written to " << filename << std::endl;
    }

    // Function to store values in a CSV file
    void storeCoefInCSV(const std::string& filename, const std::vector<std::vector<double>>& data) {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        for (const auto& row : data) {
            for (size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                if (i < row.size() - 1) {
                    file << ",";
                }
            }
            file << "\n";
        }

        file.close();
        std::cout << "Data successfully written to " << filename << std::endl;
    }

    bool armVehicle(bool arm)
    {
        mavros_msgs::CommandBool srv;
        srv.request.value = arm;
        int attempt_count = 0;
        const int max_attempts = 5; // you can set your desired max attempts
        bool armed = false;

        while (!current_state.armed && (attempt_count <= max_attempts))
        {
            if (!armed)
            {
                arming_client.call(srv);
                armed = srv.response.success;
                ROS_INFO("Arming drone ...");
                cout << "Armed? : " << armed << endl; 
                ros::Duration(0.2).sleep();
            }
            else 
            {
                ROS_INFO("Waiting for arming drone ...");
                cout << "Armed? : " << armed << endl; 
                attempt_count++;
            }
            ros::Duration(0.05).sleep();

            if (armed)
            {
                ROS_INFO("Drone armed!!!");
                return true;
            }
            else
            {
                ROS_INFO("Fail to arm drone");
                return false;
            }
        }
    }

    bool setMode(const std::string mode)
    {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = mode;
        int attempt_count = 0;
        const int max_attempts = 5; // you can set your desired max attempts

        // while (!(current_state.mode == "OFFBOARD") && (attempt_count < max_attempts))
        while (!(current_state.mode == "OFFBOARD"))
        {
            if (set_mode_client.call(srv))
            {
                if (srv.response.mode_sent)
                {
                    ROS_INFO("Changed mode to %s", mode.c_str());
                    ros::Duration(0.2).sleep(); // Wait for 1 second before the next attempt
                    return true;
                }
                else
                {
                    ROS_WARN("Failed to change mode to %s. Attempt %d/%d", mode.c_str(), attempt_count + 1, max_attempts);
                }
            }
            else
            {
                ROS_ERROR("Failed to call service set_mode. Attempt %d/%d", attempt_count + 1, max_attempts);
            }
            ++attempt_count;
            ros::Duration(1.0).sleep(); // Wait for 1 second before the next attempt
        }

        ROS_ERROR("Max attempts reached. Could not change mode to %s", mode.c_str());
        return false;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_position_ = *msg;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (loop_count == 1)
        {            
            generateTrajectory();
            loop_count = 0;
        }
    }
    
    void pub_poly_coefs(int num_point, int num_seg)
	{
	    Eigen::MatrixXd poly_coef = minsnap.getPolyCoef();
	    Eigen::MatrixXd dec_vel = minsnap.getDecVel();
	    Eigen::VectorXd time = minsnap.getTime();
	    
	    geometry_msgs::PoseStamped state_pose;

	    poly_pub_topic.num_segment = num_seg;
	    poly_pub_topic.coef_x.clear();
	    poly_pub_topic.coef_y.clear();
	    poly_pub_topic.coef_z.clear();
	    poly_pub_topic.time.clear();
	    poly_pub_topic.trajectory_id = 0;

	    // display decision variable
	    ROS_WARN("decision variable:");
	    cout << "Time size: " << time.size() << endl;
	    for (int i = 0; i < num_point; i++)
	    {
            cout << "Point number = " << i + 1 << endl
                << dec_vel.middleRows(i * 4, 4) << endl;
	    }

        Poly_traj.reset();

        // Set segment 
	    for (int i = 0; i < time.size(); i++)
	    {
            // Poly_traj.reset();
            for (int j = (i + 1) * 8 - 1; j >= i * 8; j--)
            {
                poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
                poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
                poly_pub_topic.coef_z.push_back(poly_coef(j, 2));            
            }

		    poly_pub_topic.time.push_back(time(i));
            Poly_traj.addSegment(poly_pub_topic.coef_x, poly_pub_topic.coef_y, poly_pub_topic.coef_z, time(i));
	    }

        // calculate
        Poly_traj.init();
	    cout << "calculate positions of points using the above coefficients " << endl;
        cout << "Time jump : " << Poly_traj.getTimeSum()/1000 << endl;

        double t = 0;
        while(t <= Poly_traj.getTimeSum())
        {
            double t_cur = t;
            Eigen::Vector3d pos = Poly_traj.evaluate(t_cur);
            Eigen::Vector3d vel = Poly_traj.evaluateVel(t_cur);
            Eigen::Vector3d acc = Poly_traj.evaluateAcc(t_cur);
            Eigen::Vector3d jerk = Poly_traj.evaluateJerk(t_cur);
            std::pair<double, double> yaw_yawdot(0, 0);
            
            state_pose.header.frame_id = "map";
            state_pose.header.stamp = ros::Time::now();
            state_pose.pose.position.x = pos(0);
            state_pose.pose.position.y = pos(1);
            state_pose.pose.position.z = pos(2);

            // store pos, vel and acc in csv(s) file
            pos_store.push_back(pos);
            vel_store.push_back(vel);
            acc_store.push_back(acc);

            // pose array
            pose_array.poses.push_back(state_pose.pose);
            desired_path.poses.push_back(state_pose);

            if (cmd_type == 0)
            {
                // publish position command

                cmd.header.stamp = ros::Time::now();
                cmd.header.frame_id = "map";
                cmd.trajectory_id = poly_pub_topic.trajectory_id;

                cmd.position.x = pos(0);
                cmd.position.y = pos(1);
                cmd.position.z = pos(2);

                cmd.velocity.x = vel(0);
                cmd.velocity.y = vel(1);
                cmd.velocity.z = vel(2);

                cmd.acceleration.x = acc(0);
                cmd.acceleration.y = acc(1);
                cmd.acceleration.z = acc(2);

                cmd.jerk.x = jerk(0);
                cmd.jerk.y = jerk(1);
                cmd.jerk.z = jerk(2);

                cmd.yaw = yaw_yawdot.first;
                cmd.yaw_dot = yaw_yawdot.second;

                pose_cmd_pub_.publish(cmd);
                // ROS_INFO("Published position command!");
            }
            else if (cmd_type == 1)
            {
                ros::Time time_now(t_cur);

                demand_pose.header.stamp = ros::Time::now();
                demand_pose.header.frame_id = "map";
                demand_pose.pose.position.x = pos(0);
                demand_pose.pose.position.y = pos(1);
                demand_pose.pose.position.z = pos(2);

                // mavros_cmd_pub_.publish(demand_pose);
            }                        

            // cout << "cur time : " << t << endl;
            // cout << "pos = " << pos << endl;
            // cout << "vel = " << vel << endl;
            // cout << "acc = " << acc << endl;
            // cout << "jerk = " << jerk << endl;

            t = t + Poly_traj.getTimeSum()/1000;
            
            ros::Duration(Poly_traj.getTimeSum()/1000).sleep(); // og 0.05
        }
	    
	    cout << "Finish calculation process" << endl; 
        // count++;

	    poly_pub_topic.header.frame_id = "map";
	    poly_pub_topic.header.stamp = ros::Time::now();
	    
	    // calculate positions of points using the above coefficients
        traj_pub_.publish(desired_path);
        pose_array_pub_.publish(pose_array);
        coef_pub_.publish(poly_pub_topic);

        // store values
        storeValuesInCSV("position_values.csv",pos_store);
        storeValuesInCSV("vel_values.csv",vel_store);
        storeValuesInCSV("acc_values.csv",acc_store);

        // get coefficients
        storeCoefInCSV("coefficients_x.csv", Poly_traj.getCoef(0));
        storeCoefInCSV("coefficients_y.csv", Poly_traj.getCoef(1));
        storeCoefInCSV("coefficients_z.csv", Poly_traj.getCoef(2));
	}

    void generateTrajectory() 
    {
        // Define waypoints
        std::vector<Eigen::Vector3d> waypoints;
        waypoints.clear();
        waypoints.push_back(Eigen::Vector3d(current_odom_.pose.pose.position.x,
                                            current_odom_.pose.pose.position.y,
                                            current_odom_.pose.pose.position.z));       
        // extra point
        waypoints.push_back(Eigen::Vector3d(current_odom_.pose.pose.position.x,
                                            current_odom_.pose.pose.position.y,
                                            current_odom_.pose.pose.position.z + 3.0));
        // extra point
        waypoints.push_back(Eigen::Vector3d(goal_position_.pose.position.x, 
                                            goal_position_.pose.position.y, 
                                            goal_position_.pose.position.z - 3.0));
        // goal point
        waypoints.push_back(Eigen::Vector3d(goal_position_.pose.position.x, 
                                            goal_position_.pose.position.y, 
                                            goal_position_.pose.position.z));        
        // // home point
        // waypoints.push_back(Eigen::Vector3d(current_odom_.pose.pose.position.x,
        //                                     current_odom_.pose.pose.position.y,
        //                                     current_odom_.pose.pose.position.z));                                       
        
        // number of segment, points
        num_point_total = waypoints.size();
        num_seg_total = waypoints.size() - 1;

        // separate segments
        // Define small waypoints
        std::vector<Eigen::Vector3d> small_waypoints;
        for(int i = 0; i < num_point_total; i++)
        {
            small_waypoints.push_back(waypoints[i]);

            if (small_waypoints.size() == 2)
            {
                // number of segment, points
                num_point = small_waypoints.size();
                num_seg = small_waypoints.size() - 1;

                // Generate minimum snap trajectory
                minsnap.Init(small_waypoints, mean_vel); // Adjust mean value on top
                minsnap.calMinsnap_polycoef();
                ROS_INFO("START");
                pub_poly_coefs(num_point, num_seg);

                // Reset count
                small_waypoints.clear();
                small_waypoints.push_back(waypoints[i]);

                cout << "segment : " << i << endl;
            }
        }

        // reset pose array
        cout << "number of points : " << pose_array.poses.size() << endl;
        pose_array.poses.clear();
    }

private:
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Publisher traj_pub_;
    ros::Publisher coef_pub_;
    ros::Publisher pose_array_pub_;
    ros::Publisher pose_cmd_pub_;
    ros::Publisher mavros_cmd_pub_;
    ros::Publisher takeoff_land_pub_;
    ros::Timer timer_gen;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    nav_msgs::Odometry current_odom_;
    nav_msgs::Path desired_path;
    geometry_msgs::PoseStamped goal_position_, demand_pose;
    geometry_msgs::PoseArray pose_array;
    quadrotor_msgs::PolynomialTrajectory poly_pub_topic;
    quadrotor_msgs::PositionCommand cmd;
    quadrotor_msgs::TakeoffLand takeoff_land_data;
    PolynomialTraj Poly_traj;
    my_planner::minsnapCloseform minsnap;
    int num_seg, num_point, num_seg_total, num_point_total, cmd_type, count, loop_count;
    float mean_vel;
    std::vector<Eigen::Vector3d> pos_store, vel_store, acc_store;
    mavros_msgs::State current_state;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");  
    TrajectoryGenerator traj_gen;
    ros::spin();
    return 0;
}

