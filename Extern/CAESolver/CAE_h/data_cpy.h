#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
//#include"book.h"
#include"helper_cuda.h"

inline void sysGpuNodeCpy(NodeCuda **node_array, NodeCuda *sys_node_array_gpu, int node_store_id)
{
	NodeCuda* temp_node_ptr= sys_node_array_gpu + node_store_id;
	checkCudaErrors(cudaMemcpy(node_array, &temp_node_ptr, sizeof(NodeCuda*), cudaMemcpyHostToDevice));
}

inline void sysGpuContactNodeCudaCpyH2D(ContactNodeCuda **contact_node_array, ContactNodeCuda *sys_contact_node_array,int id)
{
	ContactNodeCuda* temp_node_ptr = sys_contact_node_array + id;
	checkCudaErrors(cudaMemcpy(contact_node_array, &temp_node_ptr, sizeof(ContactNodeCuda*), cudaMemcpyHostToDevice));
}

template<class T>
inline void sysGpyCopyH2D(T **dst, T *src, int id)
{
	T* temp_src = src + id;
	checkCudaErrors(cudaMemcpy(dst, &temp_src, sizeof(T*), cudaMemcpyHostToDevice));
}

template<class T>
inline void sysGpyCopyH2D(T **dst, T *src)
{
	checkCudaErrors(cudaMemcpy(dst, &src, sizeof(T*), cudaMemcpyHostToDevice));
}

template<class T>
inline void sysDataCopyH2D(T *dst,T *src,int num)
{
	checkCudaErrors(cudaMemcpy(dst, src, sizeof(T)*num, cudaMemcpyHostToDevice));
}