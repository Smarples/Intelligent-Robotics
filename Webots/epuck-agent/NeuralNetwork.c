#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "NeuralNetwork.h"

int num_input;
int num_hidden;
int num_output;

static double* weight0;
static double* weight1;

void init_network() {
  srand(time(NULL));   // random seed
  
  num_input = 2;
  num_hidden = 20;
  num_output = 2;
  
  weight0 = (double*) malloc((num_input * num_hidden) * sizeof(double));
  weight1 = (double*) malloc((num_hidden * num_output) * sizeof(double));
}

double* feed(double * inputs) {
  int i, j;
  double * hidden_act, *output_act;

  //printf("%f, %f\n",weight0[0], weight1[0]);
  inputs[0] = tanh(inputs[0] + .2);
  inputs[1] = tanh(inputs[1] + .2);
    //printf("inputs: %f %f\n", inputs[0], inputs[1]);
  /* calculate activations for the hidden layer */
  hidden_act = (double*) malloc(num_hidden * sizeof(double));  
  for ( i = 0; i < num_hidden; i++) {  // for each hidden layer node
    hidden_act[i] = 0;
    for ( j = 0; j < num_input; j++) {  // for each input layer node
      hidden_act[i] += inputs[j] * weight0[(i * num_input) + j];
    }
    hidden_act[i] = tanh(hidden_act[i]);
    //printf("Hidden activation %d : %f\n", i, hidden_act[i]);
  }
  
  /* calculate activations for the output layer */
  output_act = (double*) malloc(num_output * sizeof(double));
  /* calculate activations for the output nodes */
  for ( i = 0; i < num_output; i++) {  // for each output layer node
    output_act[i] = 0;
    for ( j = 0; j < num_hidden; j++) {  // for each hidden layer node
      output_act[i] += hidden_act[j] * weight1[(i * num_hidden) + j];
      //printf("OUTPUT %i is %f\nFROM %f, %f\n",  i, output_act[i], hidden_act[j], weight1[15]);
    }
    //printf("Before tanh %f\n", output_act[i]);
    output_act[i] = tanh(output_act[i]);
    //printf("Output activation %d : %f\n", i, output_act[i]);
  }
  
  return output_act;
}

void setWeight0(double* ws) {
  weight0 = ws;
  //printf("Weights updated\n");
}

void setWeight1(double* ws) {
  weight1 = ws;
  int i;
  for (i = 0; i < (num_hidden * num_output); i++) {
  //printf("%f\n", weight1[i]);
  }
  //printf("Weights updated\n");
}

