/*
  Prepared by Gokul Subramanian, MSE, MEAM 2011
  Working under Professor Maxim Likhachev
  At The University of Pennsylvania
  ---14/04/2010---
*/

#ifndef _SBPL_MATH_
#define _SBPL_MATH_

#include <math.h>
#include <vector>

/* Basic Operations */
void multiply(double*, double*, int, int, double*, int);
void scalar_multiply(double*, double*, int, int, double);
void equate(double*, double*, int, int);
void matrix_add(double*, double*, double*, int, int);
void subtract(double*, double*, double*, int, int);
void transpose(double*, double*, int, int);

/* Vector Operations */
double dot_product(double*, double*, int);
void cross_product(double*, double*, double*);
double vect_norm(double*, int);
double vect_divide(double*, double*, int);
bool check_equality(double*, double*, int);

/* 3D Geometry */
void create_rotation_matrix(double*, double, double, double);
void rotate_vector(double (&result)[3], double*, double*, double);
double distance_between(double*,  double*, int);
double distance_between(std::vector<double>, double*, int);


#endif
