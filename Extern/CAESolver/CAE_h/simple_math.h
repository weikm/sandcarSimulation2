#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#define PI 3.1415926
#define square(x) (x)*(x)
#ifndef MAX
#define MAX(a,b) (a > b ? a : b)
#endif
#ifndef MIN
#define MIN(a,b) (a < b ? a : b)
#endif
#ifndef SQUARE
#define SQUARE(x) ((x)*(x))
#endif

#define sign(x,y) fabs(x)*(y>=0?1:-1) 

extern __host__ __device__ void computeTetraVolume(double node1[3],double node2[3],double node3[3],double node4[3],double &vol);

extern __host__ __device__ void computeQuadrilaArea(double node1[3], double node2[3], double node3[3], double node4[3], double &area);

extern __host__ __device__ void computeTriangleArea(double node1[3],double node2[3],double node3[3],double &area);

extern __host__ __device__ void plusProductUnsym(double *a,double *b,double *an,double dv,int ra,int ca,int rb,int cb);

extern __host__ __device__ void plusDyadSymmUpper(double *answer,double *vec,double dv,int n);

extern __host__ __device__ void solveForRhs(double *A, double* b, double* answer, int rowa, int cola, int rowb, int colb, int rowan, int colan);

template<typename T>
extern __host__ __device__ T computeMatrix3rdInverse(T src[], T an[]);

template<typename T>
extern __host__ __device__ T computeMatrix2ndInverse(T src[], T an[]);

template<typename T>
extern __host__ __device__ T computeMatrix1stInverse(T src[], T an[]);

template<typename T>
extern  __host__ __device__ void computeGeneralMatrixInverse(T scr[], T an[], int cl);


template<typename T>
extern __host__ __device__ T computeMatrix3rdDeterminant(T x[3][3]);

template<typename T>
extern __host__ __device__ T computeMatrix2ndDeterminant(T x[2][2]);

template<typename anyType>
extern __host__ __device__ anyType vector_inner_product(int n, const anyType *a, const anyType *b);

template<typename anyType>
extern __host__ __device__ int vector_outer_product(anyType *a,const anyType *b, anyType *c);

template<typename anyType>
extern __host__ __device__ int unit_normal_vector(anyType *a, anyType *b, anyType *unit_vector);

template<typename anyType>
extern __host__ __device__ int axis_transformation(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *z_norVec, anyType *x_tangVec1, anyType *y_tangVec2);

template<typename anyType>
extern __host__ __device__ int axis_transformation(anyType *node1, anyType *node2, anyType *node3,
	anyType *z_norVec, anyType *x_tangVec1, anyType *y_tangVec2);

template<typename anyType>
extern __host__ __device__ int coordinate_transforma(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *node1_T, anyType *node2_T, anyType *node3_T, anyType *node4_T);

template<typename anyType>
extern __host__ __device__ int coordinate_transforma(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *node1_T, anyType *node2_T, anyType *node3_T, anyType *node4_T,
	anyType *norVec, anyType *tangVec1, anyType *tangVec2);

template<typename anyType>
extern __host__ __device__ int axis_transformation_1(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *z_norVec, anyType *x_tangVec1, anyType *y_tangVec2);

template<typename anyType>
extern __host__ __device__ int coordinate_transforma_1(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *node1_T, anyType *node2_T, anyType *node3_T, anyType *node4_T);

template<typename anyType>
extern __host__ __device__ int coordinate_transforma_1(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *norVec, anyType *tangVec1, anyType *tangVec2, anyType *node1_T, anyType *node2_T, anyType *node3_T, anyType *node4_T);

template<typename anyType>
extern __host__ __device__ int quad_element_area(anyType *node1, anyType *node2, anyType *node3, anyType *node4, anyType *area);

template<typename anyType>
extern __host__ __device__ int quad_element_area(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *node1_T, anyType *node2_T, anyType *node3_T, anyType *node4_T, anyType *area);

template<typename anyType>
extern __host__ __device__ int quad_element_area(anyType *node1, anyType *node2, anyType *node3, anyType *node4,
	anyType *node1_T, anyType *node2_T, anyType *node3_T, anyType *node4_T,
	anyType *norVec, anyType *tangVec1, anyType *tangVec2, anyType *area);

template<typename anyType>
extern __host__ __device__ int tri_element_area(anyType *node1, anyType *node2, anyType *node3, anyType *z_norVec,
	anyType *x_tangVec1, anyType *y_tangVec2, anyType *area);

template<typename anyType>
extern __host__ __device__ int mul_matrix(const anyType *a,const anyType *b, anyType *c, int ra, int ca, int rb, int cb);

template<typename anyType>
extern __host__ __device__ int scale_mul_matrix(anyType *a_in, anyType scale, anyType *b_out, int m, int n);

template<typename anyType>
extern __host__ __device__ int trans_matrix(anyType *a_in, anyType *b_out, int m, int n);

template<typename anyType>
extern __host__ __device__ int invers_matrix33(anyType a_in[3][3], anyType b_out[3][3], anyType &det);

template<typename anyType>
extern __host__ __device__ anyType vector_unit(anyType* a,const int num);

template<typename anyType2>
extern __host__ __device__ int mul_matrix1(const double *a, const anyType2 *b, anyType2 *c, int ra, int ca, int rb, int cb);