#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

int num_input;
int num_hidden;
int num_output;

void init_network();
double* feed(double* inputs);
void setWeight0(double* ws);
void setWeight1(double* ws);

#endif