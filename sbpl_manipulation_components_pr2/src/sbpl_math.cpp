//Prepared by Gokul Subramanian, MSE, MEAM 2011
//Working under Professor Maxim Likhachev
//At The University of Pennsylvania
//---14/04/2010---

#include <sbpl_manipulation_components_pr2/sbpl_math.h>

/*************************************************************************/
//Basic operations
/*************************************************************************/

//Multiply two matrices of matching size
void multiply(double* C, double* A, int mrows, int ncols, double* B, int pcols) {
    
    int i,j,k;
    
    for(i=0; i<mrows; i++) {
        for(j=0; j<pcols; j++) {
            C[i+mrows*j]=0;
            for(k=0; k<ncols; k++) {
                C[i+mrows*j]+=A[i+mrows*k]*B[k+ncols*j];
            }
        }
    }
}

//Multiply a matrix with a scalar
void scalar_multiply(double* B, double*A, int mrows, int ncols, double num ) {
    
    int i, j;
    
    for(i=0; i<mrows; i++) {
        for(j=0; j<ncols; j++) {
            B[i+mrows*j]=num*A[i+mrows*j];
        }
    }
}

//Equate two matrices of equal size
void equate(double* B, double* A, int m, int n) {
    
    int i,j;
    
    for(i=0; i<m; i++) {
        for(j=0; j<n; j++) {
            B[i+m*j]=A[i+m*j];
        }
    }
}

//Add two matrices of equal size
void matrix_add(double* C, double* A, double* B, int m, int n) {
    
    int i, j;
    for(i=0; i<m; i++) {
        for(j=0; j<n; j++) {
            C[i+m*j]=A[i+m*j]+B[i+m*j];
        }
    }
}

//Subtract two matrices of equal size
void subtract(double* C, double* A, double* B, int m, int n) {
    
    int i, j;
    for(i=0; i<m; i++) {
        for(j=0; j<n; j++) {
            C[i+m*j]=A[i+m*j]-B[i+m*j];
        }
    }
}

//Transpose of any matrix. Use this to find inverse of orthogonal matrices
void transpose(double* B, double* A, int m, int n) {
    
    int i,j;
    
    for(i=0; i<m; i++) {
        for(j=0; j<n; j++) {
            B[j+m*i]=A[i+m*j];
        }
    }   
}

/*************************************************************************/
//Vector operations
/*************************************************************************/

//Dot (inner) product of two vectors of any length
double dot_product(double* A, double* B, int n) {
    double dp;
    int i;
    
    dp=0;
    for(i=0; i<n; i++) {
        dp+=A[i]*B[i];
    }
    
    return dp;
}

//Cross product of two 3D vectors
void cross_product(double* C, double* A, double* B) {
    
    C[0]=A[1]*B[2]-A[2]*B[1];
    C[1]=A[2]*B[0]-A[0]*B[2];
    C[2]=A[0]*B[1]-A[1]*B[0];
}

//Norm of a vector
double vect_norm(double* A, int n) {
    int i;
    double nm;
    
    nm=0;
    for(i=0; i<n; i++) {
        nm+=pow(A[i],2);
    }
    nm=sqrt(nm);
    return nm;
}

//Gives a ratio of the length of two vectors. The use of this function is
//meaningful only if the two vectors are (nearly) collinear
double vect_divide(double* A, double* B, int n) {
    double ratio;
    
    if(dot_product(A,B,n)>=0) {
        ratio=vect_norm(A,n)/vect_norm(B,n);
    }
    else {
        ratio=-vect_norm(A,n)/vect_norm(B,n);
    }
    
    return ratio;
}

//Checks to see if two vectors are the same. Returns 1 if true, else 0.

bool check_equality(double* A, double* B, int n) {
    int i;
    bool flag = 1;
    for(i=0; i<n; i++) {
        if(A[i]!=B[i]) {
            flag=0;
            break;
        }
    }
    return flag;
}
        
/*************************************************************************/
//3D geometry
/*************************************************************************/

//Create rotation matrix from yaw, pitch and roll
void create_rotation_matrix(double* A, double yaw, double pitch, double roll) {
    
    double phi=yaw;
    double theta=pitch;
    double psi=roll;
    
    double temp1[9];
    double temp2[9];
    double temp3[9];
    double temp4[9];
    
    temp1[0]=cos(phi);      temp1[3]=-sin(phi);     temp1[6]=0;
    temp1[1]=sin(phi);      temp1[4]=cos(phi);      temp1[7]=0;
    temp1[2]=0;             temp1[5]=0;             temp1[8]=1;
    
    temp2[0]=cos(theta);    temp2[3]=0;             temp2[6]=-sin(theta);
    temp2[1]=0;             temp2[4]=1;             temp2[7]=0;
    temp2[2]=sin(theta);    temp2[5]=0;             temp2[8]=cos(theta);
    
    temp3[0]=1;             temp3[3]=0;             temp3[6]=0;
    temp3[1]=0;             temp3[4]=cos(psi);      temp3[7]=-sin(psi);
    temp3[2]=0;             temp3[5]=sin(psi);      temp3[8]=cos(psi);
    
    multiply(temp4,temp2,3,3,temp3,3);
    multiply(A,temp1,3,3,temp4,3);
}

//Rotate a vector by a given angle about a given unit vector
void rotate_vector(double (&result)[3], double* vect, double* axis, double angle) {
	
	double identity[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	double w_hat[] = {0, *(axis+2), -*(axis+1), -*(axis+2), 0, *(axis+0), *(axis+1), -*(axis+0), 0};
	
	double temp[9], temp2[9], temp3[9];
	
	scalar_multiply(temp, w_hat, 3, 3, sin(angle));
	multiply(temp2, w_hat, 3, 3, w_hat, 3);
	scalar_multiply(temp3, temp2, 3, 3, 1-cos(angle));
	
	matrix_add(temp2, identity, temp, 3, 3);
	matrix_add(temp, temp2, temp3, 3, 3);
	
	multiply(result, temp, 3, 3, vect, 1);
}

//Distance between 2 points in any dimensions
double distance_between(double* A, double* B, int dim) {
	double distance = 0;	
	for (int i = 0; i < dim; i++) {
		distance += pow(*(A+i) - *(B+i), 2);
	}
	distance = sqrt(distance);
	return distance;
}

double distance_between(std::vector<double> A, double* B, int dim) {
	double distance = 0;	
	for (int i = 0; i < dim; i++) {
		distance += pow(A[i] - *(B+i), 2);
	}
	distance = sqrt(distance);
	return distance;
}

