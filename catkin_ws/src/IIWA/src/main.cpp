/**************************************************************************
***    Copyright (c) 2014 S. Mohammad Khansari, Stnford Robotics,       ***
***                      Stanford University, USA                       ***
***************************************************************************
*
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Stanford University nor the name of the author may
#       be used to endorse or promote products derived from this software without
#       specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY MOHAMMAD KHANSARI ''AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL MOHAMMAD KHANSARI BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* To get latest upadate of the software please visit:
*                          http://cs.stanford.edu/people/khansari
*
* Please send your feedbacks or questions to:
*                           khansari_at_cs.stanford.edu
***************************************************************************/
#define OPTIMIZE
#include "signal.h"
//#include <QThread>
#include "IIWARobot.h"
#include <OptiTrack.h>
#include <Eigen/Geometry>

//scl lib
//#include <scl/DataTypes.hpp>
//#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
//#include <scl/util/DatabaseUtils.hpp>
//global database of everything
//#include <scl/Singletons.hpp>
#include <scl/util/DatabaseUtils.hpp>
//#include <scl/util/HelperFunctions.hpp>
//control
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>
//Freeglut windowing environment
#include <GL/freeglut.h>

#ifdef OPTIMIZE
//Ipopt
#include "IpIpoptApplication.hpp"
#include "IpBlas.hpp"
#include "Trajectory_nlp.hpp"
// optimizer globals
Ipopt::SmartPtr<Trajectory_NLP>  aiming_nlp;
Ipopt::SmartPtr<IpoptApplication> solver_app = new IpoptApplication();
Ipopt::Number dart_vel = 12; // m/s
#endif

using namespace std;

//globals - bad style
bool b_useCalibration = true;
double currentPosition[3];
double **currentRotationMatrix;
OptiTrack *objects;
int nbObjects;
scl::SRobotIO robotIO;
chai3d::cGenericObject* graphics_quad_pos;  //pointer for quadrotor graphics object
scl::SRigidBodyDyn *end_effector;
scl::STaskOpPos* rtask_hand;       //Will need to set hand desired positions etc.
scl::CControllerMultiTask rctr;    //A multi-task controller
scl::CDynamicsTao world_dynamics;
scl::SGcModel robot_model;
scl::CDynamicsScl robot_dynamics;
bool control_init = false;
scl::SRobotParsed robot_properties;
float default_joint_pos[IIWA_DOF_JOINTS] = { 0,-0.524,0,-1.571,0,-1.047,0};
ros::Time old_time;
IIWA::IIWAMsg startState;
int init_position_idx = 0;
bool got_start_state = false;

#define JOINT_ANGLE_REACHED_THRESH 0.1 //~5 deg
#define JOINT_LIMIT_THRESH 0.1 //~5 deg
#define WORKSPACE_X_LIMIT 1000 // mm
#define WORKSPACE_Y_LIMIT 1000 // mm
#define WORKSPACE_Z_MAX 1150 // mm
#define WORKSPACE_Z_MIN 400 // mm
#define INIT_STEPS (IIWACONTROLFREQUENCY*5)

class IIWAController: public IIWARobot{
public:
    void Controller(const IIWA::IIWAMsg &currentState, IIWA::IIWAMsg &desiredState);
    
private:
#ifdef OPTIMIZE  
  Ipopt::Number x[7];
  
  // dual variables, just guess zero at first
  Ipopt::Number z_L[7];
  Ipopt::Number z_U[7];
  Ipopt::Number lambda[4];
  
  double prevQuadPos[3];
  ros::Time prevVisionTime;
  
  bool sol_rcd;
#endif
};

void IIWAController::Controller(const IIWA::IIWAMsg &currentState, IIWA::IIWAMsg &desiredState){
 
    if(!got_start_state){
      startState = currentState;
      got_start_state=true;
    }

    bool useSimDataFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[1];
    bool simControlFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[2];
    bool sendRobotSimJointsFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[3];
    bool useOptiTrackDataFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[4];
    bool sendRobotCartPosFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[5];
    bool optimizeFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[6];
    bool targetFlag = scl::CDatabase::getData()->s_gui_.ui_flag_[7];

    bool print_debug = false;
    if(ros::Time::now() - old_time > ros::Duration(1)){
      old_time = ros::Time::now();
      print_debug = true;
      if(useSimDataFlag){
        std::cout<< "\nUsing Sim data\n";
      }else{
        std::cout<< "\nUsing Robot data\n";
      }
      if(simControlFlag){
        std::cout<< "Sim control follow point\n";
      }else{
        std::cout<< "Sim control float\n";
      }
      if(sendRobotSimJointsFlag){
        std::cout<< "Send robot sim joint angles\n";
      }else{
        std::cout<< "Don't send robot sim joint angles\n";
      }
      if(useOptiTrackDataFlag){
        std::cout<< "Using Optitrack target: ";
	if(targetFlag){
          std::cout<< "heli\n";
        }else{
          std::cout<< "quad\n";
        }
      }else{
        std::cout<< "Using Sim target\n";
      }
      if(sendRobotCartPosFlag){
        std::cout<< "Send robot desired cart position\n";
      }else{
        std::cout<< "Don't send robot desired cart position\n";
      }
      if(optimizeFlag){
        std::cout<< "Running optimizer\n";
      }else{
        std::cout<< "Not running optimizer\n";
      }
      std::cout<< "\n";
    }

    Eigen::Matrix3d Ree;
    Eigen::Vector3d eePos;
    IIWA::IIWAMsg robotState = currentState;
    
    // '1' sets whether we get input data from the simulation or from the robot - default at startup is from simulation
    if( useSimDataFlag){
      Ree = end_effector->T_o_lnk_.rotation();
      eePos = end_effector->T_o_lnk_.translation()*1000;
      for(int i=0;i<IIWA_DOF_JOINTS;i++){
        robotState.jointAngles[i] = robotIO.sensors_.q_[i];
        if(i<3){
          robotState.cartPosition[i] = eePos[i];
        }
      }
      robotState.jointAngles[3] *= -1;
    }else{
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          Ree(i,j) = currentState.cartOrientation[i*3+j];
        }
      }
      eePos << currentState.cartPosition[0],currentState.cartPosition[1],currentState.cartPosition[2];
      for(int i=0;i<IIWA_DOF_JOINTS;i++){
        robotIO.sensors_.q_[i] = currentState.jointAngles[i];
      }
      robotIO.sensors_.q_[3] *= -1;
    }

    if(print_debug){
      std::cout << "End Effector Position:\n" << eePos << "\nEnd Effector Rotation:\n" << Ree << "\n\n";
    }
  
    //go to initial position
    if(!control_init){
      bool reached_start_pos = true;
      for(int i=0;i<IIWA_DOF_JOINTS;i++){
        if(std::abs(robotState.jointAngles[i]-default_joint_pos[i]) > JOINT_ANGLE_REACHED_THRESH){
          reached_start_pos = false;
        }
      }
      if(reached_start_pos){
        std::cout << "START POSITION REACHED\n";
        //startState = currentState;
        control_init = reached_start_pos;
        return;
      }else{
        if(print_debug){
          std::cout << "GOING TO START\n";
          for(int i=0;i<IIWA_DOF_JOINTS;i++){
            std::cout << robotState.jointAngles[i] << " , ";
          }
          std::cout << "\n\n";
        }
      }

      desiredState.jointAngles.resize(IIWA_DOF_JOINTS);
      desiredState.isJointControl = true;
      desiredState.jointStiffness.resize(IIWA_DOF_JOINTS);
      for(int i=0;i<IIWA_DOF_JOINTS;i++){
        //float err = startState.jointAngles[i]-default_joint_pos[i];
        //desiredState.jointAngles[i] = robotState.jointAngles[i]-std::min(std::max(err,-0.1f),0.1f);i
        desiredState.jointAngles[i] = default_joint_pos[i]*init_position_idx/INIT_STEPS + startState.jointAngles[i]*(INIT_STEPS-init_position_idx)/INIT_STEPS;
        //if(print_debug){ std::cout << robotState.jointStiffness[i] <<"\n";}
      }
      init_position_idx++;
      init_position_idx = std::min(init_position_idx,INIT_STEPS);
      desiredState.cartPosition.resize(3);
      desiredState.cartOrientation.resize(9);
      desiredState.cartPosition = currentState.cartPosition;
      desiredState.cartOrientation = currentState.cartOrientation;
      
      return;
    }
    
    //Check if joint limits are exceeded
    for( int i=0;i<IIWA_DOF_JOINTS;i++){
      if((robot_properties.gc_pos_limit_max_[i] - robotState.jointAngles[i] < JOINT_LIMIT_THRESH) || (robotState.jointAngles[i] - robot_properties.gc_pos_limit_min_[i] < JOINT_LIMIT_THRESH)){
        desiredState.isJointControl = false;
        if(print_debug){
          std::cout << robotState.jointAngles[i] << std::endl;
          std::cout << "JOINT LIMIT REACHED AT JOINT " << i+1 << std::endl;
        }
        sendRobotSimJointsFlag = false; // set robot control to float so it doesn't do anything if robot moves back within joint limits
        sendRobotCartPosFlag = false; // set robot control to float so it doesn't do anything if robot moves back within joint limits
        control_init = true;
        return;
      }
    }
    
    //Check if work space is exceeded
    if( std::abs(robotState.cartPosition[0]) > WORKSPACE_X_LIMIT || std::abs(robotState.cartPosition[1]) > WORKSPACE_Y_LIMIT || robotState.cartPosition[2] > WORKSPACE_Z_MAX || robotState.cartPosition[2] < WORKSPACE_Z_MIN ){
      desiredState.isJointControl = false;
      if(print_debug){
        std::cout << "WORKSPACE LIMIT REACHED " << std::endl;
      }
      sendRobotSimJointsFlag = false; // set robot control to float so it doesn't do anything if robot moves back within limits
      sendRobotCartPosFlag = false; // set robot control to float so it doesn't do anything if robot moves back within limits
      return;
    }

    //Get Optitrack data
    ros::Time visionTime;
    int tgt = 0;
    if(!targetFlag){
      tgt = 1;
    }
        if (objects[tgt].IsEnabled()){
            //reading new values, if there is any
            objects[tgt].Update();

            // Getting the object position
            objects[tgt].GetPosition(currentPosition, b_useCalibration);

            // Getting the object orientation
            objects[tgt].GetRotationMatrix(currentRotationMatrix, b_useCalibration);
            visionTime = ros::Time::now();
            //printing
/*            std::cout << "Object name: " << objects[i].GetObjectName() << std::endl;
            std::cout << "Position:    [" << currentPosition[0] << "," << currentPosition[1] << "," << currentPosition[2] << "]" << std::endl;

            std::cout << "Orientation: [" << currentRotationMatrix[0][0] << "," << currentRotationMatrix[0][1] << "," << currentRotationMatrix[0][2] << std::endl;
            std::cout << "              " << currentRotationMatrix[1][0] << "," << currentRotationMatrix[1][1] << "," << currentRotationMatrix[1][2] << std::endl;
            std::cout << "              " << currentRotationMatrix[2][0] << "," << currentRotationMatrix[2][1] << "," << currentRotationMatrix[2][2] << "]" << std::endl;*/
        }
    
    Eigen::Vector3d quad_pos, objPos;
    double uavHeight = 0;
    if(targetFlag){
      uavHeight = 0.04;
    }else{
      uavHeight = -0.04;
    }
    Eigen::Vector3d optitrack2KukaOffset(0.975,-1.047,0.000+uavHeight);
    
    // '4' sets whether we get our target position from optitrack or simulation - default at startup is from simulation
    if ( useOptiTrackDataFlag ) {
      quad_pos[0] = currentPosition[1];
      quad_pos[1] = -currentPosition[0];
      quad_pos[2] = currentPosition[2];
      quad_pos = quad_pos+optitrack2KukaOffset;
      objPos = quad_pos*1000;
    } else {
      quad_pos = scl::CDatabase::getData()->s_gui_.ui_point_[0];
      objPos = quad_pos*1000;
    }
    //display quadrotor position
    graphics_quad_pos->setLocalPos ( quad_pos[0],quad_pos[1],quad_pos[2] );

    //Compute desired position and orientation
    Eigen::Vector3d launcher_offset(49.1,13.6,81);
    Eigen::Vector3d desPos;// = objPos + Eigen::Vector3d(-500,0,470);
    Eigen::Vector3d curPos = Ree*launcher_offset + eePos;
    //objPos[0] = 10600; objPos[1] = 0; objPos[2] = 575;
    Eigen::Vector3d desBearing = objPos - curPos;
    double range = desBearing.norm()/1000;  //[m]
    desBearing.normalize();
    Eigen::Vector3d aimAxis(1,0,0);
    //cout << "TEST" << endl;
    //cout << objPos[0] << " " << objPos[1] << " " << objPos[2] << endl;
    //write your controller here
    //cout << currentState.cartPosition[0] << " "
    //     << currentState.cartPosition[1] << " "
    //     << currentState.cartPosition[2] << endl;

    desPos = (objPos-curPos)*0.01 + curPos - Ree*launcher_offset;
    desPos[0] = std::min(std::max(desPos[0],100.0),200.0);
    desPos[1] = std::min(std::max(desPos[1],-100.0),100.0);
    desPos[2] = std::min(std::max(desPos[2],900.0),1050.0);
    Eigen::Vector3d bearingEig = Ree*aimAxis;
#ifdef OPTIMIZE
    //Optimizer
    Ipopt::Number quadVel[3];
    Ipopt::Number quadPosition[3];
    Ipopt::Number bearing[3];
    Ipopt::Number curEEPos[3];
    for(int i=0; i<3; i++){
      // put Eig vector into double array for solver
      bearing[i] = bearingEig[i];
      // update current position in kuka frame
      quadPosition[i] = objPos[i]/1000.0;
      // calculate average velocity between measurements
      quadVel[i] = quadVel[i] + 0.1*((quadPosition[i] - prevQuadPos[i])/((visionTime - prevVisionTime).toSec()) - quadVel[i]);
      //quadVel[i] = 0.0;
      // done with previous position, overwrite with current
      prevQuadPos[i] = quadPosition[i];
      
    }
    // done with previous time, overwrite with current
    prevVisionTime = visionTime;
    
    if (optimizeFlag){
      
      //if this is the first time the optimizer has been run:
      // initialize the x, z_L, z_U, lambda variables
      if (!sol_rcd){
        double len = 0.0;
        // initial guess for optimal position is current position
        for (int i=0; i<3; i++){
          x[i] = curPos[i]/1000.0;
          // initial guess for optimal bearing is current bearing to target
          x[i+3] = desBearing[i];
        }
        // initial guess for flight time
        x[6] = range/dart_vel;
        //initial dual variables
        for (int i=0;i<7;i++){
          z_L[i] = 0;
          z_U[i] = 0;
        }
        for (int i=0;i<4;i++){
          lambda[i] = 0;
        }
      }
      // if the optimizer has been run, the guesses will be the previous solution instead
      for(int i =0;i<3;i++){
        curEEPos[i] = x[i];
      }
      // update start point info
      aiming_nlp->set_starting_point(curEEPos, bearing, true, x, false, z_L, z_U, false, lambda);
      // update quad position and velocity
      aiming_nlp->set_q_info(quadPosition, quadVel);
      //aiming_nlp->print_params();
      // Ask Ipopt to solve the problem
      Ipopt::ApplicationReturnStatus status = solver_app->OptimizeTNLP(aiming_nlp);
      
      // if the solver converged to an optimum copy the solution
      if (status == Ipopt::Solve_Succeeded){
        // write the solution to class member variables to initialize next pass
        sol_rcd = aiming_nlp->get_solution(x, z_L, z_U, lambda);
      }
      else{
        sol_rcd = false;
      }
      //copy solution into Eigen structures for final computation and setting desired
      // if the optimizer failed to solve, this should command the previous solution.
      //   if this was also the first solver run, it should command current position, pointing at quad
      for (int i=0; i<3; i++){
        desPos[i] = x[i]*1000;
        desBearing[i] = x[i+3];
      }
      desPos -= Ree*launcher_offset;
      //aiming_nlp->print_params();
      //std::cout << desPos << "\n\n" << desBearing << "\n\n";
    }
#endif    
    Eigen::Quaternion<double> rotQuat;
    rotQuat.setFromTwoVectors(bearingEig,desBearing);
    rotQuat.normalize();
    Eigen::Matrix3d R = rotQuat.toRotationMatrix()*Ree;
    
    if(print_debug){
      cout << desPos <<"\n\n";
      //cout << rotQuat.coeffs() << "\n\n";
      //cout << R << "\n\n";
    }
    //write your desired command in the variable desiredState.
    //You do not need to fill all the variables, just set the ones that you want to change now.
    //Important: Don't FORGET to resize them first.
    
    // '2' sets whether simulation control follows a point or just floats - default at startup is float
    if( useSimDataFlag || sendRobotSimJointsFlag ){
      if( simControlFlag){
        rtask_hand->x_goal_ = desPos/1000;
        rctr.computeDynamics();
        rctr.computeControlForces();
      }else{
        robot_dynamics.computeGCModel ( &robotIO.sensors_,&robot_model );
        robotIO.actuators_.force_gc_commanded_ = -robot_model.force_gc_grav_ - 100*robotIO.sensors_.dq_;
      }
      //limit simulation forces
      robotIO.actuators_.force_gc_commanded_[0] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[0],-150.0),150.0);
      robotIO.actuators_.force_gc_commanded_[1] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[1],-150.0),150.0);
      robotIO.actuators_.force_gc_commanded_[2] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[2],-90.0),90.0);
      robotIO.actuators_.force_gc_commanded_[3] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[3],-90.0),90.0);
      robotIO.actuators_.force_gc_commanded_[4] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[4],-90.0),90.0);
      robotIO.actuators_.force_gc_commanded_[5] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[5],-30.0),30.0);
      robotIO.actuators_.force_gc_commanded_[6] = std::min(std::max(robotIO.actuators_.force_gc_commanded_[6],-30.0),30.0);
    
      world_dynamics.integrate ( robotIO, 0.0001 );
      robot_dynamics.computeGCModel ( &robotIO.sensors_,&robot_model);
    }

    
    // '3' sends simulation joint angles to the robot as control input. 
    // We require that the input to the simulation be robot data and direct robot control is disabled
    if( sendRobotSimJointsFlag && !useSimDataFlag && !sendRobotCartPosFlag){
      // Try controlling with joint values - simulated joint values too close to original values, controller gains too low, ends up floating
      desiredState.isJointControl = true;
      desiredState.jointAngles.resize(IIWA_DOF_JOINTS);
      desiredState.cartPosition.resize(3);
      desiredState.cartOrientation.resize(9);
      desiredState.cartPosition = currentState.cartPosition;
      desiredState.cartOrientation = currentState.cartOrientation;
      for(int i=0;i<IIWA_DOF_JOINTS;i++){
        desiredState.jointAngles[i] = robotIO.sensors_.q_[i];
      }
      desiredState.jointAngles[3] *= -1;
      if(print_debug){
        std::cout << "sending sim joints to robot for control\n";
      }

      // Try controlling with simulator end effector position - doesn't work because of mismatch between simulator position and actual robot position, simulator model does not match kuka
      /*desiredState.isJointControl = false;
      desiredState.jointAngles.resize(IIWA_DOF_JOINTS);
      desiredState.jointAngles = currentState.jointAngles;
      desiredState.cartPosition.resize(3);
      desiredState.cartOrientation.resize(9);
      Eigen::Matrix3d simRot = end_effector->T_o_lnk_.rotation();
      Eigen::Vector3d simPos = end_effector->T_o_lnk_.translation()*1000;
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          desiredState.cartOrientation[i*3+j] = simRot(i,j);
        }
        desiredState.cartPosition[i] = simPos[i];
      }*/

      return;
    }
    
    // '5' sends the desired position/orientation directly to the robot cartesian controller
    // We require that the input is from robot and the simulation joint angle control is disabled
    if( sendRobotCartPosFlag && !sendRobotSimJointsFlag && !useSimDataFlag){
      //You need to set this flag to define the control type
      desiredState.isJointControl = false;

      desiredState.cartPosition.resize(3);
      desiredState.cartOrientation.resize(9);
      desiredState.jointAngles.resize(IIWA_DOF_JOINTS);
      desiredState.jointAngles = currentState.jointAngles;

      //cout << "following optitrack" << endl;
      //cout<< desPos << endl;
      for(int i=0;i<3;i++){
        desiredState.cartPosition[i] = desPos[i];
        for(int j=0;j<3;j++){
          desiredState.cartOrientation[i*3+j] = R(i,j);
        }
      }
      if(print_debug){
        std::cout<< "sending cart pos to robot for control\n";
      }
      return;
    }

    // Float the robot if no other control mode is specified
    //desiredState.isJointControl = true;
    //desiredState.jointAngles.resize(IIWA_DOF_JOINTS);
    //desiredState.jointAngles = currentState.jointAngles;
}


void sighandle(int signal)
{
    exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_core");

    IIWAController myIIWAController;

    std::string vrpn_server_ip = "172.24.68.48:3883"; //you should not change this ip address
    std::cout << "vrpn_server_ip:" << vrpn_server_ip << std::endl;

    //The name of the objects that you want to track
    std::string target_name[] = {"heli","quad"};

    //computing the number of strings in the target_name
    nbObjects = sizeof( target_name ) / sizeof( std::string );

    //initializing the optitrack object(s)
    objects = new OptiTrack[nbObjects];
    for (int i=0; i<nbObjects; i++){
        objects[i].Init(vrpn_server_ip, target_name[i]);

        //if you want to use your own coordinate system T, you could load it here
        //T is a homogeneous matrix from OptiTrack coordinate system to your coordinate system
        objects[i].loadCalibrationMatrix("/home/student/group4/cs225a/OptiTrack/VisionCalib.txt");
    }

    currentRotationMatrix = new double*[3];
    for (int i=0; i<3; i++)
        currentRotationMatrix[i] = new double[3];

    signal(SIGINT, sighandle);
    signal(SIGQUIT, sighandle);
    
    //SCL STUFF
    scl::CParserScl parser;
    
    //parsed the robot
    const std::string fname ( "/home/student/group4/cs225a/catkin_ws/src/IIWA/src/iiwaCfg.xml" );
    parser.readRobotFromFile ( fname,"","iiwaBot",robot_properties );
    
    //sensors, input/outputs
    robotIO.init ( robot_properties.name_, robot_properties.dof_ );
    
    //set initial sensor values
    Eigen::VectorXd q_ready ( robot_properties.dof_ );
    for ( unsigned int i=0; i<robot_properties.dof_; ++i ) {
      scl::SRigidBody* rigid_body = robot_properties.rb_tree_.at ( i );
      q_ready[i] = rigid_body->joint_default_pos_;
    }
    robotIO.sensors_.q_=q_ready;
    
    //create physics engine
    world_dynamics.init ( robot_properties );
    
    //dynamic robot
    robot_model.init ( robot_properties );
    
    //our control dynamics engine
    robot_dynamics.init ( robot_properties );
    robot_dynamics.computeGCModel ( &robotIO.sensors_,&robot_model );
    end_effector = robot_model.rbdyn_tree_.at ( "end-effector" );
    
    //graphics
    glutInit ( &argc, argv );
    scl::SGraphicsParsed graphics_properties;
    parser.readGraphicsFromFile ( fname, "iiwaBotStdView", graphics_properties );
    scl::CGraphicsChai graphics;
    graphics.initGraphics ( &graphics_properties );
    graphics.addRobotToRender ( &robot_properties, &robotIO );
    scl_chai_glut_interface::initializeGlutForChai ( &graphics_properties, &graphics );
    graphics.addSphereToRender ( Eigen::Vector3d(0,0,0), graphics_quad_pos, 0.05 );
    scl::CDatabase::getData()->s_gui_.ui_point_[0] = Eigen::Vector3d(1.1,0,0.1);

    //scl task space controller
    bool flag;
    scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
    std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
    std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
    std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
    /******************************Set up Controller Specification************************************/
    // Read xml file info into task specifications.
    flag = parser.readTaskControllerFromFile ( fname,"opc",rtasks,rtasks_nc,ctrl_params );
    flag = flag && rctr_ds.init ( "opc",&robot_properties,&robotIO,&robot_model ); //Set up the control data structure..
    // Tasks are initialized after find their type with dynamic typing.
    flag = flag && scl_registry::registerNativeDynamicTypes();
    flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks ( rtasks,rtasks_nc,rctr_ds );
    flag = flag && rctr.init ( &rctr_ds,&robot_dynamics );     //Set up the controller (needs parsed data and a dyn object)
    if ( false == flag ) {
      return 1;    //Error check.
    }
    rtask_hand = dynamic_cast<scl::STaskOpPos*> ( * ( rctr_ds.tasks_.at ( "hand" ) ) );
    if ( NULL == rtask_hand )  {
      return 1;   //Error check
    }
#ifdef OPTIMIZE
    //Optimizer initialization:
    //initializ parameters
    Ipopt::Index N = 7;  //number of optimizer variables
    // cost weighting for translation, rotation, and time-of-flight
    const Ipopt::Number alph = 0.9;
    const Ipopt::Number beta = 0.03;
    const Ipopt::Number gam = 0.01;
    
    const Ipopt::Number g = -7;
    // directional weights for translation
    Ipopt::Number A [9] = {1,0,0,
                           0,1,0,
                           0,0,1};
    // directional weights for rotation
    Ipopt::Number B [9] = {1,0,0,
                           0,1,0,
                           0,0,1};
    // upper bounds for optimizer variables x
    Ipopt::Number x_u [7] = {0.25, 0.1, 1.10, INFINITY, INFINITY, INFINITY, INFINITY};
    // lower bounds for optimizer variables x
    Ipopt::Number x_l [7] = {0.15, -.1, 0.95, -INFINITY, -INFINITY, -INFINITY, 0};
    // create a new instance of the nlp
    //  (use a SmartPtr, not raw)
    aiming_nlp = new Trajectory_NLP(N, g, dart_vel, alph, beta, gam);
    aiming_nlp->set_x_bounds_info(x_l, x_u);
    aiming_nlp->set_direction_weights(A, B);
    // Change some options
    solver_app->Options()->SetNumericValue("print_level",0);
    solver_app->Options()->SetNumericValue("print_frequency_time",1e10);
    solver_app->Options()->SetNumericValue("print_frequency_iter",100000);
    solver_app->Options()->SetNumericValue("tol", 2e-3);
    solver_app->Options()->SetNumericValue("constr_viol_tol", 5e-3);
    //app->Options()->SetNumericValue("tol", INF);
    solver_app->Options()->SetStringValue("mu_strategy", "adaptive");
    // Intialize the IpoptApplication and process the options
    solver_app->Initialize();
#endif
    //create two threads
    omp_set_num_threads ( 2 );
    int thread_id;
    
    old_time = ros::Time::now();
    #pragma omp parallel private(thread_id)
    {
      thread_id = omp_get_thread_num();
      
      if ( thread_id==1 ) {
        //control loop is here
        while(true)
        {
            myIIWAController.RobotUpdate();
        }
      } else { //Read the rio data structure and updated rendered robot..
        while ( true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running ) {
          glutMainLoopEvent();
          const timespec ts = {0, 15000000};/*15ms*/
          nanosleep ( &ts,NULL );
        }
      }
    }

    return 0;
}


