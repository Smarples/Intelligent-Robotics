/*
 * File:          epuck_ne.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <webots/differential_wheels.h>
#include <webots/supervisor.h>

#include "NeuralNetwork.h"

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

WbDeviceTag receiver; 
static int time_step;

/* Declaring distance sensors */
#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};

static void get_sensor_input() {
  int i;
  for (i=0; i<DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);
    
    // scale the data in order to have a value between 0.0 and 1.0
    // 1.0 representing something to avoid, 0.0 representing nothing to avoid
    distance_sensors_values[i] /=  4096;
    //printf("%f\n", distance_sensors_values[i]);
  }
 }
  
  static void init_devices() {
  int i;
  for (i=0; i<DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i]=wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], time_step);
  }
}
  
  
void run() {
  get_sensor_input();
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  time_step = wb_robot_get_basic_time_step();
  init_devices();
  init_network();
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
    receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  printf("Receiver enabled\n");
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   int rec = 0;
  while (wb_robot_step(time_step) != -1) {
  //printf("processed\n");
        while (wb_receiver_get_queue_length(receiver) > 0) {
        int i, j;
        //printf("receiving\n");
         double* weightst = malloc(((num_input * num_hidden) + (num_hidden * num_output)) * sizeof(double));
         weightst = wb_receiver_get_data(receiver);
        //printf("Received %f\n", weightst[0]);
        double* we0 = malloc((num_input * num_hidden) * sizeof(double));
        double* we1 = malloc((num_hidden * num_output) * sizeof(double));
        for ( i = 0 ; i < (num_input * num_hidden); i++) {
          we0[i] = weightst[i];
        }
        for ( j = 0; j < (num_hidden * num_output); j++) {
          we1[j] = weightst[(num_input * num_hidden) + j];
          //printf("wo %f\n", we1[j]);
        }
        //printf("111: %f\n", weightst[20]);
        setWeight0(we0);
        setWeight1(we1);
        wb_receiver_next_packet(receiver);
//        const double *a = wb_receiver_get_data(receiver);
  //      printf("Received %f\n", a[0]);

        
        }        
          get_sensor_input();
        double *out = feed(distance_sensors_values);
        
        wb_differential_wheels_set_speed((500 *  out[0]), (500 * out[1]));
        

  };
  
  /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}
