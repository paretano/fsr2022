#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
std::string sep = "\n----------------------------------------\n";



        class trajectoryPlanner {

            public:
                ros::NodeHandle nh;
                ros::Subscriber state; //Current State
                ros::Publisher stated;  //State Desidered
                ros::Subscriber sub_map;
                ros::Timer timer;
                Eigen::Vector3d p;     // Position
                Eigen::Vector3d pd;     // Position desidered
                Eigen::Vector3d p_sent; //Position sent
                nav_msgs::OccupancyGrid map;
                double hz;
                trajectoryPlanner();
                void state_(const nav_msgs::Odometry& cur_state);
                void map_(const nav_msgs::OccupancyGrid& mapp);
                void controlLoop(const ros::TimerEvent& t);
                bool isObstacle(const int& i, const int& j);
            private:
                double ka = 10;
                double kr = 500;
        };
        //Constructor;
        trajectoryPlanner::trajectoryPlanner(){
                hz=1000;
                pd<<0.8,2,0.3;
                state = nh.subscribe("/hummingbird/ground_truth/odometry", 1, &trajectoryPlanner::state_, this);
                sub_map = nh.subscribe("map", 1, &trajectoryPlanner::map_, this);
                stated = nh.advertise<nav_msgs::Odometry>("hummingbird/command/trajectory", 1);
                timer = nh.createTimer(ros::Rate(hz), &trajectoryPlanner::controlLoop, this);
                }
        //Current State Subscriber
        void trajectoryPlanner::state_(const nav_msgs::Odometry& cur_state){
        p << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
        }
        //Read Map Subscriber
        void trajectoryPlanner::map_(const nav_msgs::OccupancyGrid& mapp){
            map=mapp;
        }
        //Check if  is obstacle,-1 unknow, 0 free and  1-100 obstacles
        bool trajectoryPlanner::isObstacle(const int& i, const int& j){
            if(map.data[i*map.info.width + j]>0)
            return true;
            else 
            return false;
        }


        void trajectoryPlanner::controlLoop(const ros::TimerEvent& t){
           Eigen::Vector3d attraction_force; 
           Eigen::Vector3d total_force;
           Eigen::Vector3d min(0.1,0.1,0.1); 
           Eigen::Vector3d err_p(pd-p);
           Eigen::Vector2d repulsion_force(0,0);
           double r = 0.3;
            // Calculate attraction force
           attraction_force=(pd-p)*ka;
           // Calculate repulsion force
              for(int i = 0; i<map.info.height; i++){
                for(int j = 0; j<map.info.width; j++){
                    if(this->isObstacle(i,j)){ 
                        Eigen::Vector2d obsVector(j*map.info.resolution-10, i*map.info.resolution-10);
                        Eigen::Vector2d diffVector(p(0)-obsVector(0),p(1)-obsVector(1)); //Distance from obstacles
                        if(diffVector.norm()<=0.3){
                            //if there a obstacles near UAV calculate repulsion force
                            repulsion_force = repulsion_force + diffVector.normalized()*kr*((1.0 / diffVector.norm() - 1.0 / r)/(diffVector.norm()*diffVector.norm()));
                        }
                    }
                }    
            }
            // We have 2d map 
            total_force(0)=attraction_force(0)+repulsion_force(0);
            total_force(1)=attraction_force(1)+repulsion_force(1);
            total_force(2)=attraction_force(2);

            


            std::cout<<repulsion_force<<sep;
            //If we are near goal, send goal
            err_p.cwiseAbs();
            if(err_p(0)<0.1 && err_p(1)<0.1  && err_p(2)<0.1  )
                p_sent=pd;
            else
            {p_sent(0)=(total_force(0)*(1/sqrt(total_force(1)*total_force(1)+total_force(0)*total_force(0))))*0.2+p(0);
            p_sent(1)=(total_force(1)*(1/sqrt(total_force(1)*total_force(1)+total_force(0)*total_force(0))))*0.2+p(1);
            p_sent(2)=total_force(2)/sqrt(total_force(2)*total_force(2))*0.2+p(2);}
           nav_msgs::Odometry traj;
           traj.pose.pose.position.x=p_sent(0);
           traj.pose.pose.position.y=p_sent(1);
           traj.pose.pose.position.z=p_sent(2);
           stated.publish(traj);   
        }

          int main(int argc, char** argv){
                ros::init(argc, argv, "traasaj");
                trajectoryPlanner pla;
                ros::spin();
                }
