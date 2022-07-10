#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

std::string sep = "\n----------------------------------------\n";

static Eigen::Vector3d Skew(const Eigen::Matrix3d& m){
    Eigen::Vector3d v;
    v << m(2,1), m(0,2), m(1,0);
    return v;
  }


  class geometricControllerNode{
    ros::NodeHandle nh;
    ros::Subscriber state,stated;
    ros::Publisher error_p;
    ros::Publisher wrench_p;
    
    ros::Publisher prop;
    ros::Timer timer;
    double kv, kp, m, g, l, ct,cq,tau_t,tau_m; // Gain,Mass,Gravity,Length,Thrust Constant and Drag Coefficient
    Eigen::Matrix3d kr;     //Controller Gain
    Eigen::Matrix3d komega;  //Controller Gain


    float Ts;


    Eigen::Matrix3d J;     // Inertia Matrix
    Eigen::Vector3d e3;    // [0,0,1]


    Eigen::Matrix3d R;     //Rotation Matrix
    Eigen::Matrix3d Rd;    //Rotation Matrix Desidered
    Eigen::Matrix3d Rdtemp;//Rotation Matrix Desidered K-1
    
    Eigen::Vector3d p;     // Position
    Eigen::Vector3d dotp;  // Velocity
    Eigen::Vector3d omega; // Angular Velocity

    Eigen::Vector3d pd;    // Position Desidered
    Eigen::Vector3d dotpd; // Velocity Desidered
    Eigen::Vector3d ad;    // Acceleration Desidered
    Eigen::Vector3d omegadtemp; // Angular Velocity Desidered K-1
    Eigen::Vector3d omegad;     // Angular Velocity Desidered    
    Eigen::Vector3d dotomegad; // Angular Acceleration Desidered

    double yawd;           // Yaw Angle Desidered
    double yawdtemp;
    double hz;             // Frequency   
  

  public:
  geometricControllerNode();
  void state_(const nav_msgs::Odometry& cur_state);
  void controlLoop(const ros::TimerEvent& t);
  void stated_(const nav_msgs::Odometry& des_state);
  };


geometricControllerNode::geometricControllerNode():e3(0,0,1),hz(1000.0){
      /*pd <<0.0, 0.0, 2.0;
      dotpd <<0.0, 0.0, 0.0;
      ad <<0.0, 0.0, 0.0;
      yawd=0.0;*/
      Ts = 1/hz; 

      stated = nh.subscribe("hummingbird/command/trajectory", 1, &geometricControllerNode::stated_, this);
      state = nh.subscribe("/hummingbird/ground_truth/odometry", 1, &geometricControllerNode::state_, this);
      prop = nh.advertise<mav_msgs::Actuators>("hummingbird/command/motor_speed", 1);
      timer = nh.createTimer(ros::Rate(hz), &geometricControllerNode::controlLoop, this);
      error_p = nh.advertise<nav_msgs::Odometry>("error", 1);
      wrench_p = nh.advertise<geometry_msgs::Wrench>("wrench", 1);


      m = 0.716;
      g = -9.81;
      l = 0.17;

      kp = 6;
      kv = 4;
      kr<<  0.7,0.0,0.0,
            0.0,0.7,0.0,
            0.0,0.0,0.35;

      komega<<  0.1,0.0,0.0,
            0.0,0.1,0.0,
            0.0,0.0,0.25;


      J <<  0.007,0.0,0.0,
            0.0,0.007,0.0,
            0.0,0.0,0.012;

      kr= kr.transpose()*J.inverse();
      komega = komega.transpose()*J.inverse();

      tau_t=8.54858e-06;
      tau_m=1.6e-2;
  }

   void geometricControllerNode::state_(const nav_msgs::Odometry& cur_state){

      p << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
      dotp << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
      omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;

      float x,y,z,w;
      x = cur_state.pose.pose.orientation.x;
      y = cur_state.pose.pose.orientation.y;
      z = cur_state.pose.pose.orientation.z;
      w = cur_state.pose.pose.orientation.w;
      Eigen::Quaterniond q(w,x,y,z);
      R = q.toRotationMatrix();
      dotp=R*dotp;
  }

  void geometricControllerNode::stated_(const nav_msgs::Odometry& des_state){ 
      // Posizione 
      pd << des_state.pose.pose.position.x,des_state.pose.pose.position.y,des_state.pose.pose.position.z;
      dotpd << 0,0,0;
      ad<<0,0,0;
      yawd = 0;
  }


  void geometricControllerNode::controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ep, edotp, ep1,ev1, er, eomega;
    ep = p - pd; //Position Error
    edotp = dotp - dotpd; //Velocity Error

    nav_msgs::Odometry error;
      error.pose.pose.position.x=ep(0);
      error.pose.pose.position.y=ep(1);
      error.pose.pose.position.z=ep(2);
    error_p.publish(error);


    // Equation 12 from paper
    Eigen::Vector3d accn;
    accn=(kp*ep + kv*edotp)/m + g*e3 - ad;
    Eigen::Vector3d b_3d = -accn/accn.norm();

    //Create Temp b_1d
    Eigen::Vector3d b_1d_t(cos(yawd), sin(yawd), 0);
    //Now i can calculate b_2d
    Eigen::Vector3d b_2d = b_3d.cross(b_1d_t);
    b_2d.normalize();
    //Now i can calculate b_1d true
    Eigen::Vector3d b_1d = b_2d.cross(b_3d);
    //Rotation Matrix Desidered
    Rd.col(0)=b_1d;
    Rd.col(1)=b_2d;
    Rd.col(2)=b_3d;


   
    
    //Calculate Dot Rotation Matrix Desidered
    Eigen::Matrix3d Rddot;
    Rddot= (Rd-Rdtemp)/Ts;


    double dotyawd;
    dotyawd = (yawd-yawdtemp)/Ts;
   
    //Calculate omega_d and dot_omega_d
    omegad= Eigen::Vector3d::Zero();
    omegad(2)=dotyawd;
    dotomegad = (omegad-omegadtemp)/Ts;


    //Equation 10 from paper
    er = 0.5 * Skew(Rd.transpose()*R - R.transpose()*Rd);    
    //Equation 11 from paper
    eomega = omega - (R.transpose()*Rd*omegad);
    //- (R.transpose()*Rd*omegad);
    
    //Equation 16 From Paper
    Eigen::Vector3d torques = -kr*er -komega*eomega + omega.cross(omega);

    //Equation 15 From Paper
    double f = -m*accn.dot(R.col(2));


    // Now we have torques and total thrust we can convert to angular velocity rotors.
    Eigen::Vector4d wrench;
    wrench.block<3, 1>(0, 0) = torques;
    wrench(3) = f;

    geometry_msgs::Wrench wrenchs;
    wrenchs.force.x=f; //is all not only x
    wrenchs.torque.x=torques(0);
    wrenchs.torque.y=torques(1);
    wrenchs.torque.z=torques(2);

    wrench_p.publish(wrenchs);



    Eigen::Matrix4d allocation_matrix;

     allocation_matrix<<0,l*tau_t,0,-l*tau_t,
                      -l*tau_t,0,l*tau_t,0,
                      tau_t*tau_m,-tau_t*tau_m,tau_t*tau_m,-tau_t*tau_m,
                      tau_t,tau_t,tau_t,tau_t;


    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = J;
    I(3, 3) = 1;

    //Eigen::Matrix4d angular_acc_to_rotor_velocities;
    //angular_acc_to_rotor_velocities= allocation_matrix.transpose()*(allocation_matrix*allocation_matrix.transpose()).inverse()*I;



    Eigen::Vector4d rotor_velocities;
    rotor_velocities = allocation_matrix.inverse() *I * wrench;
    rotor_velocities = rotor_velocities.cwiseMax(Eigen::VectorXd::Zero(rotor_velocities.rows()));
    rotor_velocities = rotor_velocities.cwiseSqrt();
    

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = rotor_velocities(0);
    msg.angular_velocities[1] = rotor_velocities(1);
    msg.angular_velocities[2] = rotor_velocities(2);
    msg.angular_velocities[3] =  rotor_velocities(3);
    prop.publish(msg);

    //Storage Old Rd and Omega_d
    Rdtemp=Rd;
    omegadtemp=omegad;
    yawdtemp = yawd;

  }

  int main(int argc, char** argv){
  ros::init(argc, argv, "node");
  geometricControllerNode n;
  ros::spin();
}
