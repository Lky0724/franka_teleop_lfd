//============================================================================
// Name        : leader.cpp
// Author      : Basak Gülecyüz (basak.guelecyuez@tum.de)
// Version     : April 2023
// Description : leader node for teleoperation between force dimensiion haptic devices <-> franka arm 
//============================================================================

// General
#include <assert.h>
#include <gtest/gtest.h>
#include <pthread.h>
#include <time.h>
#include <queue>
#include <string>
#include <vector>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// ROS
//#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <ros/package.h>

// Haptics
#include "dhdc.h"
#include "drdc.h"



////////////////////////////////////////////////////////////////
// Declared Variables for ROS
////////////////////////////////////////////////////////////////
ros::NodeHandle* n;                 // Leader ROS Node
ros::Publisher* L_Pub;              // Leader publisher
ros::Subscriber* L_Sub;             // Leader subscriber
ros::Timer TimerRecorder;           // Timer for recorder thread
double TimerPeriodHaptic = 0.001;   // Comm. rate (1/T)
double CurrentTime = 0;

////////////////////////////////////////////////////////////////
// Declared Variables for Bilateral Teleoperation
////////////////////////////////////////////////////////////////

// Path to package
std::string package_path = ros::package::getPath("franka_teleop_lmt");

// Demo number for saving session
std::string demo_num;


double ws = 2;   // Work space scaling factor btw. haptic device and robot

// Variables to be communicated between leader and follower
std::vector<double> command(3, 0.0);        // Vm[3] to be sent to follower
std::vector<double> feedback(3, 0.0);       // Fm[3] received from follower
std::queue<std::vector<double>>feedbackQ;   // Force feedback queue
bool first_packet = 0;    // True if first force feedback is received

// Leader side Variables
double Vl[3] = {0, 0, 0};             // Leader velocity
double Xl[3] = {0.25, 0.25, 0.4};     // Leader position
double Fl[3] = {0,0,0};               // Leader force

// Variables for Kalman filtering of Velocity 
struct KalmanState {
  double CurrentEstimation[3] = {0, 0, 0};
  double PreviousEstimation[3] = {0, 0, 0};
  double PredictionErrorVar[3] = {0, 0, 0};
};
KalmanState kalman_state;
bool FlagVelocityKalmanFilter = 1;

// Data recording
typedef struct {
  double time;
  bool first_packet;
  double Xl[3];
  double Vl[3];
  double Fl[3];
} RecordData;         // Record data struct
std::queue<RecordData> RecordQueue;   // Queue what to record 
FILE* fidRecord;      // File for saving


std::mutex recordMutex;               // Mutex for preventing data overwriting in record
std::mutex feedbackMutex;             // Mutex for preventing data overwriting in subscriber
////////////////////////////////////////////////////////////////
// Declared Functions for Haptics
////////////////////////////////////////////////////////////////
void* HapticsLoop(void* ptr);                // Main haptics thread 
void RecordSignals(const ros::TimerEvent&);  // Thread to record the signals
void ForceCallback(const std_msgs::Float64MultiArrayConstPtr&);  // Subscriber callback
void ApplyKalmanFilter_Velocity(double* CurrentSample);    // Kalman fitler velocity
int  InitHaptics();  // Haptic device initialization


int main(int argc, char** argv) {



  ////////////////////////////////////////////////////////////////
  // Initialize ROS
  ////////////////////////////////////////////////////////////////

  ros::init(argc, argv, "leader");
  n = new ros::NodeHandle;

  // Demo number
  demo_num= argv[1];

  // Leader publisher for commands
  L_Pub = new ros::Publisher;
  *L_Pub = n->advertise<std_msgs::Float64MultiArray>("/cartesian_impedance_example_controller/LFcommand", 1, false);

  // Leader subscriber for feedback
  L_Sub = new ros::Subscriber;
  *L_Sub = n->subscribe<std_msgs::Float64MultiArray>(
      "/cartesian_impedance_example_controller/FLfeedback", 1, &ForceCallback, ros::TransportHints().tcpNoDelay(true));

  // Sleep for 5 seconds 
  ros::Duration(5.0).sleep();
  ROS_INFO("Publisher & Subscriber are set! ");
  ROS_INFO("Publisher & Subscriber are set! ");
  ////////////////////////////////////////////////////////////////
  // Initialize Recording
  ////////////////////////////////////////////////////////////////

  std::string record_path =  package_path + "/TeleopData/leader_" + demo_num + ".txt"; 
  fidRecord = fopen(record_path.c_str(), "w");
  if (fidRecord== NULL) {
    ROS_INFO("Failed: ");
    return 1;
}
  fprintf(fidRecord, "time\t");                // timestamp
  fprintf(fidRecord, "Vl_x\tVl_y\tVl_z\t");    // Leader Velocity
  fprintf(fidRecord, "Xl_x\tXl_y\tXl_z\t");    // Leader Position
  fprintf(fidRecord, "Fl_x\tFl_y\tFl_z\t");  // Leader Force
	fprintf(fidRecord, "first_packet\n"); // first packet received
  fclose(fidRecord);
  TimerRecorder = n->createTimer(ros::Duration(TimerPeriodHaptic),
                                 RecordSignals);  // create a timer callback for data record

  ////////////////////////////////////////////////////////////////
  // Initialize Haptics
  ////////////////////////////////////////////////////////////////

  // Initialize the haptic device
  InitHaptics();

  ros::Duration(5.0).sleep();
  ROS_INFO("Haptic Device is initialized! ");


  ////////////////////////////////////////////////////////////////
  // Main haptics loop @1 kHz 
  ////////////////////////////////////////////////////////////////

  pthread_t HapticsLoop_t;
  pthread_create(&HapticsLoop_t, NULL, HapticsLoop, NULL);
  pthread_join(HapticsLoop_t, NULL);

  ////////////////////////////////////////////////////////////////
  // Close everything safely
  ////////////////////////////////////////////////////////////////

  return 0;
}

void* HapticsLoop(void* ptr) {

    // Enable force on Haptic Device
  dhdEnableForce (DHD_ON);


  // Loop at 1 kHz
  ros::Rate loop_rate(1000);
  ros::AsyncSpinner spinner(3);
  spinner.start();

  while (ros::ok()) {

  // 1. Get velocity from haptic device
  // ---> Kalman filtering on velocity to remove high frequency fluctuations
  // 2. Receive force from follower 
  // ---> Additional damping on force to stabilize
  // 3. Display forces to haptic device
  // 4. Publish velocity to follower
  // 5. Push into RecordQueue for saving

    /////////////////////////////////////////////////////////
    // TO-DO: 1. Get velocity & position from haptic device
    /////////////////////////////////////////////////////////
    
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    dhdGetLinearVelocity(&Vl[0], &Vl[1], &Vl[2]);
    

    //  workspace scaling
    for (int i = 0; i < 3; ++i) {
      Vl[i] = Vl[i] * ws ;  
    }

    /////////////////////////////////////////////////////////
    // ---> Kalman filtering on velocity
    /////////////////////////////////////////////////////////
    if (FlagVelocityKalmanFilter == 1) {
      ApplyKalmanFilter_Velocity(Vl);
      Vl[0] = kalman_state.CurrentEstimation[0];
      Vl[1] = kalman_state.CurrentEstimation[1];
      Vl[2] = kalman_state.CurrentEstimation[2];
    }

    // Update Leader position
    for (int i = 0; i < 3; ++i) {
      Xl[i] = Xl[i] + Vl[i] * 0.001;
    }


	  /////////////////////////////////////////////////////////////////////
		// TO-DO: 2. Check receive queues for force feedback
		/////////////////////////////////////////////////////////////////////
  if (feedbackMutex.try_lock()) {
      if (feedbackQ.size()) {
        
        // Get the front value from feedbackQ into variable feedback.
        // Pop from queue feedbackQ
        /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Your code here!
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
        feedback = feedbackQ.front();
        feedbackQ.pop();

        // Copy the vector feedback into Fl
        /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Your code here!
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
        for (int i = 0; i < 3; ++i) {
          Fl[i] = feedback[i];
        }

      }
    feedbackMutex.unlock(); 
  }

		/////////////////////////////////////////////////////////////////////
		//--->  Damping on Force for stability
		/////////////////////////////////////////////////////////////////////

    // Extra Damping
    double damping_factor = 0;
    double Fl_norm = 0;
    for (int i = 0; i < 3; ++i) {
        Fl_norm += Fl[i] * Fl[i] ;
      }
    Fl_norm = sqrt(Fl_norm);

    if(Fl_norm>2.0){
        //ROS_INFO("Damping! ");
        for (int i = 0; i < 3; ++i) {
        Fl[i] = Fl[i] + damping_factor*Vl[i]/ws;
      }
    }

    /////////////////////////////////////////////////////////
    // TO-DO 3. Display forces to haptic device
    /////////////////////////////////////////////////////////

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    double factor = 5;
    dhdSetForce(-Fl[0]/factor, -Fl[1]/factor, -Fl[2]/factor);

    /////////////////////////////////////////////////////////
    // TO-DO: 4. Publish command
    /////////////////////////////////////////////////////////

    std_msgs::Float64MultiArray command_msg;
    command.clear();
    command.insert(command.end(), Vl, Vl+3);

    // Send new command
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    command_msg.data = command;
    L_Pub->publish(command_msg);
  
    /////////////////////////////////////////////////////////
    // 5. Record Signals for analysis
    /////////////////////////////////////////////////////////

    RecordData* tmp = new RecordData;  // temporary variable storing the signals

    // time
    tmp->time = CurrentTime;


    // Leader states
    for (int k = 0; k < 3; k++) {
      tmp->Vl[k] = Vl[k];
      tmp->Xl[k] = Xl[k];
      tmp->Fl[k] = Fl[k];
  
    }

		// First packet received
		tmp->first_packet = first_packet;
    
    // Push to record queue
    if (recordMutex.try_lock()) {
      RecordQueue.push(*tmp);
      delete tmp;
      recordMutex.unlock(); 
    }


    CurrentTime = CurrentTime + TimerPeriodHaptic;
    loop_rate.sleep();

    }
  return NULL;
}

void RecordSignals(const ros::TimerEvent&) {

  std::string record_path =  package_path + "/TeleopData/leader_" + demo_num + ".txt"; 
  fidRecord = fopen(record_path.c_str(), "a");
  if (fidRecord == NULL) {
    ROS_INFO("File error");
  }
  bool DataArrived = false;
  RecordData* tmp = new RecordData;

  const std::lock_guard<std::mutex> lock(recordMutex);
  //ROS_INFO("Mutex Lock Record Queue 2! ");
  if (RecordQueue.size() > 0) {

    *tmp = RecordQueue.front();
    RecordQueue.pop();

    DataArrived = true;
  } else {
    DataArrived = false;
  }
  if (DataArrived == true) {
    fprintf(fidRecord, "%f\t", tmp->time);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Vl[0], tmp->Vl[1], tmp->Vl[2]);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Xl[0], tmp->Xl[1], tmp->Xl[2]);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Fl[0], tmp->Fl[1], tmp->Fl[2]);
		fprintf(fidRecord, "%d\n", tmp->first_packet);
  }
  fclose(fidRecord);
}


void ForceCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {

const std::lock_guard<std::mutex> lock(feedbackMutex);
  // Receive feedback into queue
  feedbackQ.push(msg->data);

	if(feedbackQ.size()>0){
		first_packet = 1;
	}

}

void ApplyKalmanFilter_Velocity(double* CurrentSample) {
  double Innovation[3];     // I
  double InnovationVar[3];  // S
  double NoiseVar[3];       // R
  double Gain[3];           // K
  double ProcNoiseVar[3];   // Q

  NoiseVar[0] = 2000.0;
  NoiseVar[1] = 2000.0;
  NoiseVar[2] = 2000.0;

  ProcNoiseVar[0] = 1.0;
  ProcNoiseVar[1] = 1.0;
  ProcNoiseVar[2] = 1.0;

  for (int i = 0; i < 3; i++) {
    Innovation[i] = CurrentSample[i] - kalman_state.PreviousEstimation[i];
    InnovationVar[i] = kalman_state.PredictionErrorVar[i] + NoiseVar[i];
    Gain[i] = kalman_state.PredictionErrorVar[i] / InnovationVar[i];

    kalman_state.CurrentEstimation[i] =
        kalman_state.PreviousEstimation[i] + Gain[i] * Innovation[i];

    // update values for next iteration
    kalman_state.PredictionErrorVar[i] = kalman_state.PredictionErrorVar[i] + ProcNoiseVar[i] -
                                         Gain[i] * kalman_state.PredictionErrorVar[i];
    kalman_state.PreviousEstimation[i] = kalman_state.CurrentEstimation[i];
  }
}

// haptic device initialization
int InitHaptics() {
  if (dhdOpen() >= 0) {
    printf("%s device detected\n", dhdGetSystemName());
  }

  else {
    printf("no device detected\n");
    dhdSleep(2.0);
    exit(0);
  }
  printf("\n");

  return 0;
}
