#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include"helper_cuda.h"

template<class T>
inline void allocZeroMemAllData(T* &gpu_ptr, T* &cpu_ptr, int num)
{
	checkCudaErrors(cudaHostAlloc(&cpu_ptr, sizeof(T)*num, cudaHostAllocMapped/*||cudaHostAllocWriteCombined*/));
	checkCudaErrors(cudaHostGetDevicePointer(&gpu_ptr, cpu_ptr, 0));
}