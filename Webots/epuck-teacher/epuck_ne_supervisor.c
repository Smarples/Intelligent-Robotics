/*
 * File:          epuck_ne_supervisor.c
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
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <webots/differential_wheels.h>

#include "NeuralNetwork.h"

static int time_step;
static WbDeviceTag emitter;   // to send genes to robot
WbFieldRef trans_field;
WbFieldRef rot_field;
double *orient;

static const int POPULATION_SIZE = 30;
static const int NUM_GENERATIONS = 250;
static const int MUT_RATE = 0.2;
static const double INITIAL[3] = { 0, 0.7, 0 };
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

typedef struct Individual {
  double *weight0, *weight1;
  int fitness;
} individual;

individual* population;

int run_seconds(double seconds) {
  int i, n = 1000.0 * seconds / time_step;
  int f = 0;
  double xmax = .0, xmin = .0, ymax = .0, ymin = .0;
  for (i = 0; i < n; i++) {

    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
        if (i > 100 && trans[1] < 0.2) {
      f = -10000; 
    }
    if (trans[0] > xmax) {
      xmax = trans[0];
    }
    else if (trans[0] < xmin) {
      xmin = trans[0];
      }
      if (trans[2] > ymax) {
      ymax = trans[2];
      }
      else if (trans[2] < ymin) {
        ymin = trans[2];
        }
    //printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);
    wb_robot_step(time_step);
    
  }
  //printf("minmax %f %f %f %f\n", xmin, xmax, ymin, ymax);
  f += ((200 * (xmax - xmin)) + (100 * (ymax - ymin)));
  return f;
}

void evaluate_genotype(int pop_index) {
    /* Reset e-puck to default position */
  wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
  wb_supervisor_field_set_sf_rotation(rot_field, orient);
  // send genotype to robot for evaluation  
    double* temp = malloc(((num_input * num_hidden) + (num_hidden * num_output)) * sizeof(double));
    memcpy(temp, population[pop_index].weight0, (num_input * num_hidden) * sizeof(double));
    memcpy(temp + (num_input * num_hidden), population[pop_index].weight1, (num_hidden * num_output) * sizeof(double));
    //printf("%f\n", temp[(num_input * num_hidden) + 10]);
  wb_emitter_send(emitter, temp, (((num_input * num_hidden) + (num_hidden * num_output) )* sizeof(double)));
  // evaluation genotype during one minute
  population[pop_index].fitness =run_seconds(30.0);
  //printf("FITNESS: %i\n", population[pop_index].fitness);
}

double* randomWeights(int count) {
  int i;
  double r;    // random weights to return
  double *w;
  w = (double*) malloc(count * sizeof(double));
  for ( i = 0; i < count; i++) {
    r = (((double) rand() / (double) RAND_MAX) * 2) - 1;
    w[i] = r;
  }
  return w;
}

void gen_pop() {
  int i;

  struct individual *ind;
  /* allocate memory for population */
  population = (individual*) malloc(POPULATION_SIZE * sizeof(individual));
  
  for ( i = 0; i < POPULATION_SIZE; i++) {    // for each individual in population
    individual *ind = malloc(sizeof(individual));   // allocate struct memory
    
    ind->weight0 = malloc((num_input*num_hidden) * sizeof(double));  // alloc weight0 memory
    ind->weight0 = randomWeights(num_input * num_hidden);  // set weight0 randomly
    
    ind->weight1 = malloc((num_hidden * num_output) * sizeof(double));
    ind->weight1 = randomWeights(num_hidden * num_output);
    population[i] = *ind;
    //printf("%f %f\n", population[i].weight0[0], population[i].weight1[10]);
    
  }
  printf("Initial population of %i individuals generated.\n", POPULATION_SIZE);
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  int i, j, k;
  srand(33);
  /* necessary to initialize webots stuff */
  wb_robot_init();
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("EPUCK");
  trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
  orient = wb_supervisor_node_get_orientation(robot_node);
  init_network();
  
    // get simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // the emitter to send genotype to robot
  emitter = wb_robot_get_device("emitter");
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  
  /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   int maxfit = -9999999999;
   double randm;
   
   gen_pop();
   int fitind;
   for ( j = 0; j < NUM_GENERATIONS; j++) {
   individual *fittest = malloc(sizeof(individual));
   for ( i = 0; i < POPULATION_SIZE; i++) {
     //printf("Genotype %i\n", i);
       evaluate_genotype(i);  
       if (population[i].fitness > maxfit){
         maxfit = population[i].fitness;
         fitind = i;
         //printf("new max fit: %i\n", maxfit);
       }
     }
     memcpy(fittest, &population[fitind], sizeof(individual));
     population[0] = *fittest;
     for (i = 1; i < POPULATION_SIZE; i++) {
       
       for ( k = 0; k < (num_input * num_hidden); k++) {
         randm = rand() / (double) RAND_MAX;
         if (randm > MUT_RATE) {
           population[i].weight0[k] = ((rand() / (double) RAND_MAX) * 2 ) - 1;
           //printf("%f\n", ((rand() / (double) RAND_MAX) * 2 ) - 1);
         }
         else {
           population[i].weight0[k] = fittest->weight0[k];
         }
       }
       for (k = 0; k < (num_hidden * num_output); k++) {
         randm = rand() / (double) RAND_MAX;
         if (randm > MUT_RATE) {
           population[i].weight1[k] = ((rand() / (double) RAND_MAX) * 2 ) - 1;
         }
         else {
           population[i].weight1[k] = fittest->weight1[k];
         }
       }
     }
     printf("Generation %d completed. Highest fitness: %d\n", j, fittest->fitness);
     //printf("%d %d\n", fittest->weight0[10], fittest->weight1[10]);
    }
    
     
    /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}
