#include <ros/ros.h>

#include "assignment_3_4.hpp"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <trajectory_msgs/JointTrajectory.h>

class Trajectory {

private:

  ros::NodeHandle nh;
  ros::Subscriber sub_setpoint;
  ros::Subscriber sub_jointstate;
  ros::Publisher  pub_trajectory;

  double shoulder_pan_joint;
  double shoulder_lift_joint;
  double elbow_joint;

  tf::TransformListener listener;

  // This reads the first three joints
  void JointState( const sensor_msgs::JointState& jointstate ){ 
    for( size_t i=0; i<jointstate.name.size(); i++ ){
      if( jointstate.name[i] == "shoulder_pan_joint" )
	{ shoulder_pan_joint = jointstate.position[i]; }
      if( jointstate.name[i] == "shoulder_lift_joint" )
	{ shoulder_lift_joint = jointstate.position[i]; }
      if( jointstate.name[i] == "elbow_joint" )
	{ elbow_joint = jointstate.position[i]; }
    }
  }

  // Main callback used when a new setpoint is received
  void SetPoint( const geometry_msgs::Point& newgoal ){

    double positionincrement = 1e-3; // how much we move between steps

    // The name of all the joints
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back( "shoulder_pan_joint" );
    trajectory.joint_names.push_back( "shoulder_lift_joint" );
    trajectory.joint_names.push_back( "elbow_joint" );
    trajectory.joint_names.push_back( "wrist_1_joint" );
    trajectory.joint_names.push_back( "wrist_2_joint" );
    trajectory.joint_names.push_back( "wrist_3_joint" );

    // The trajectory point. Initialized with current values
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.push_back( shoulder_pan_joint );
    trajectory_point.positions.push_back( shoulder_lift_joint );
    trajectory_point.positions.push_back( elbow_joint );
    trajectory_point.positions.push_back( -M_PI_2 );
    trajectory_point.positions.push_back( M_PI_2 );
    trajectory_point.positions.push_back( 0.0 );
    trajectory_point.time_from_start = ros::Duration( positionincrement );

    double E[4][4];
    // update the current position/orientation of the end effector
    ForwardKinematics( trajectory_point.positions[0],
                       trajectory_point.positions[1],
                       trajectory_point.positions[2],
                       E );
    
    // The current position of the end effector
    tf::Point current;
    current.setX( E[0][3] );
    current.setY( E[1][3] );
    current.setZ( E[2][3] );

    // The goal position of the end effector
    tf::Point goal;
    goal.setX( newgoal.x );
    goal.setY( newgoal.y );
    goal.setZ( newgoal.z );

    // This is the translation left for the trajectory
    tf::Point translation = goal - current;

    // loop until the translation is small enough
    while( positionincrement < translation.length() ){

      // Determine a desired cartesian linear velocity. 
      // We will command the robot to move with this velocity
      tf::Point vb = ( translation / translation.length() )*positionincrement;

      double Js[6][3], J[3][3], Ji[3][3];

      // Compute the Jacobian
      Jacobian( trajectory_point.positions[0],
		trajectory_point.positions[1],
		trajectory_point.positions[2],
		Js );

      for( int r=0; r<3; r++ ){
          for( int c=0; c<3; c++ ){
              J[r][c] = Js[r][c];
              }
          }
      
      // update the current position/orientation of the end effector
      ForwardKinematics( trajectory_point.positions[0],
			 trajectory_point.positions[1],
			 trajectory_point.positions[2],
			 E );

      current.setX( E[0][3] );
      current.setY( E[1][3] );
      current.setZ( E[2][3] );
      
      // Compute the inverse Jacobian. The inverse return the 
      // value of the determinant.
      if( fabs(Inverse( J, Ji )) < 1e-09 )
	{ std::cout << "Jacobian is near singular." << std::endl; }

      // Compute the joint velocity by multiplying the (Ji v)
      double qd[3];
      qd[0] = Ji[0][0]*vb[0] + Ji[0][1]*vb[1] + Ji[0][2]*vb[2];
      qd[1] = Ji[1][0]*vb[0] + Ji[1][1]*vb[1] + Ji[1][2]*vb[2];
      qd[2] = Ji[2][0]*vb[0] + Ji[2][1]*vb[1] + Ji[2][2]*vb[2];

      // increment the joint positions
      trajectory_point.positions[0] += (qd[0]);
      trajectory_point.positions[1] += (qd[1]);
      trajectory_point.positions[2] += (qd[2]);
      trajectory_point.time_from_start += ros::Duration( positionincrement*10 );

      // push the new point in the trajectory
      trajectory.points.push_back( trajectory_point );

      goal.setX( newgoal.x );
      goal.setY( newgoal.y );
      goal.setZ( newgoal.z );
      
      translation = goal - current;

   }

    pub_trajectory.publish( trajectory );
    
  }

public:

  Trajectory( ros::NodeHandle& nh ):
    nh( nh ){
    sub_setpoint=nh.subscribe( "setpoint",1,&Trajectory::SetPoint,this );
    sub_jointstate=nh.subscribe( "joint_states",1,&Trajectory::JointState,this );
    pub_trajectory=nh.advertise<trajectory_msgs::JointTrajectory>( "/arm_controller/command", 1 );
  }

private:

  // Inverse a 3x3 matrix
  // input: A 3x3 matrix
  // output: A 3x3 matrix inverse
  // return the determinant inverse
  double Inverse( double A[3][3], double Ainverse[3][3] ){
    
    double determinant = (  A[0][0]*( A[1][1]*A[2][2]-A[2][1]*A[1][2] ) -
			    A[0][1]*( A[1][0]*A[2][2]-A[1][2]*A[2][0] ) +
			    A[0][2]*( A[1][0]*A[2][1]-A[1][1]*A[2][0] ) );
    
    double invdet = 1.0/determinant;
    
    Ainverse[0][0] =  ( A[1][1]*A[2][2] - A[2][1]*A[1][2] )*invdet;
    Ainverse[0][1] = -( A[0][1]*A[2][2] - A[0][2]*A[2][1] )*invdet;
    Ainverse[0][2] =  ( A[0][1]*A[1][2] - A[0][2]*A[1][1] )*invdet;
    
    Ainverse[1][0] = -( A[1][0]*A[2][2] - A[1][2]*A[2][0] )*invdet;
    Ainverse[1][1] =  ( A[0][0]*A[2][2] - A[0][2]*A[2][0] )*invdet;
    Ainverse[1][2] = -( A[0][0]*A[1][2] - A[1][0]*A[0][2] )*invdet;
    
    Ainverse[2][0] =  ( A[1][0]*A[2][1] - A[2][0]*A[1][1] )*invdet;
    Ainverse[2][1] = -( A[0][0]*A[2][1] - A[2][0]*A[0][1] )*invdet;
    Ainverse[2][2] =  ( A[0][0]*A[1][1] - A[1][0]*A[0][1] )*invdet;
    
    return determinant;
    
  }

};



int main( int argc, char** argv ){

  // This must be called for every node
  ros::init( argc, argv, "trajectory" );

  // Create a node handle
  ros::NodeHandle nh;

  Trajectory trajectory( nh );

  ros::spin();

  return 0;

}

