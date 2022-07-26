#pragma once
#include<vector>
#include<iostream>
#include<string>
#include <cuda_runtime.h>
#include"helper_cuda.h"

template<typename T>
struct BufferData
{
	/**
	������
	*/
	std::vector<T> dataArray_;

	/**
	cpu����ָ��
	*/
	T* ptrCpu_;

	/**
	gpu����ָ���У����ڴ�Ÿ���GPU�ϵ�����ָ��
	*/
	std::vector<T*> ptrGpuArray_;

	/**
	��gpu�������gpu�ϵ��м����
	*/
	std::vector<T*> ptrMediumGpuArray_;

	/**
	���ݳ���
	*/
	int size_;

	/**
	��������6.0��GPU�ı����
	*/
	static std::vector<int> gpuIdArray_;

	/**
	����GPU�ϵ�������̬��Ա���ýṹ�干��
	*/
	static std::vector<cudaStream_t> streamArray_;

	/**
	������gpu������gpu����Ĳ���id�洢��Gpu�ϵ�
	*/
	static std::vector<int*> sharedIdStoreGpu_;

	/**
	������gpu������gpu����Ĳ���id�洢��Cpu�ϵ�
	*/
	static std::vector<int*> sharedIdStoreCpu_;

	/**
	������gpu������gpu����Ĳ���id����
	*/
	static std::vector<int> sharedSize_;

	/**
	gpu��������̬��Ա���ýṹ�干��
	*/
	static int gpuNum;

	/**
	��gpu���ݿ�����cpu��
	*/
	void gpuToCpu(const int gpuId = 0)
	{
		cudaSetDevice(gpuId);
		if (ptrGpuArray_[gpuId] == nullptr)
		{
			std::cout << "ERROR: can't copy null data from GPU " << gpuId << "to CPU " << __FILE__ << __LINE__ << std::endl;
		}
		if (ptrCpu_ == nullptr)
		{
			allocateCpu();
		}
		checkCudaErrors(cudaMemcpyAsync(ptrGpuArray_[gpuId], ptrCpu_, size_ * sizeof(T), cudaMemcpyDeviceToHost,streamArray_[gpuId]));
	}

	/**
	��cpu���ݿ���������gpu��
	*/
	void cpuToAllGpu()
	{
		if (ptrCpu_ == nullptr)
		{
			std::cout << "ERROR: cpu ptr is nullptr" << __FILE__ << __LINE__ << std::endl;
		}
		for (int in = 0; in < gpuNum; in++)
		{
			cudaSetDevice(gpuIdArray_[in]);
			checkCudaErrors(cudaMemcpyAsync(ptrCpu_, ptrGpuArray_[in], size_ * sizeof(T), cudaMemcpyHostToDevice, streamArray_[in]));
		}
	}

	/**
	��cpu���ݿ������ض�gpu��
	*/
	void cpuToAssignGpu(const int gpuId)
	{
		cudaSetDevice(gpuIdArray_[gpuId]);
		if (ptrCpu_ == nullptr)
		{
			std::cout << "ERROR: cpu ptr is nullptr" << __FILE__ << __LINE__ << std::endl;
		}
		if (ptrGpuArray_[gpuId] == nullptr)
		{
			checkCudaErrors(cudaMalloc(&ptrGpuArray_[gpuId], size_ * sizeof(T)));
		}
		checkCudaErrors(cudaMemcpyAsync(ptrCpu_, ptrGpuArray_[gpuId], size_ * sizeof(T), cudaMemcpyHostToDevice, streamArray_[gpuId]));
	}

	/**
	����gpu�����ݸ��Ƶ���gpu��
	*/
	void mianGpuToAidGpu()
	{
		for (int in = 1; in < gpuNum; in++)
		{
			cudaSetDevice(gpuIdArray_[in]);
			checkCudaErrors(cudaMemcpyAsync(ptrGpuArray_[0], 
				ptrGpuArray_[in], size_ * sizeof(T), cudaMemcpyDeviceToDevice, streamArray_[in]));
		}
	}

	/**
	����gpu�����ݸ��Ƶ���Gpu��
	*/
	void aidGpuToMainGpu();

	/**
	�ͷ�����gpu����
	*/
	void releaseAllGpu()
	{
		for (int in = 0; in < gpuNum; in++)
		{
			checkCudaErrors(cudaFree(ptrGpuArray_[in]));
			ptrGpuArray_[in] = nullptr;
		}
	}

	/**
	�ͷ����и���gpu���м����
	*/
	void releaseAllMediumGpu()
	{
		for (int in = 0; in < gpuNum - 1; in++)
		{
			checkCudaErrors(cudaFree(ptrMediumGpuArray_[in]));
			ptrMediumGpuArray_[in] = nullptr;
		}
	}

	/**
	�ͷ�cpu����
	*/
	void releaseCpu()
	{
		if (ptrCpu_ != nullptr)
			dataArray_.clear();
		ptrCpu_ = nullptr;
	}

	/**
	cpu��������
	*/
	void setZeroCpu()
	{
		if (ptrCpu_ != nullptr)
		{
			memset(ptrCpu_, 0, size_ * sizeof(T));
		}
	}

	/**
	����Gpu��������
	*/
	void setZeroAllGpu()
	{
		for (int in = 0; in < gpuNum; in++)
		{
			cudaSetDevice(gpuIdArray_[in]);
			checkCudaErrors(cudaMemsetAsync(ptrGpuArray_[in], 0, size_ * sizeof(T), streamArray_[in]));
		}
	}

	/**
	����cpu����
	*/
	void allocateCpu()
	{
		if (ptrCpu_ == nullptr&&size_ > 0)
		{
			dataArray_.resize(size_);
			ptrCpu_ = dataArray_.data();
		}
	}

	/**
	����gpu����
	*/
	void allocateAllGpu()
	{
		ptrGpuArray_.resize(gpuNum);
		for (int in = 0; in < gpuNum; in++)
		{
			cudaSetDevice(gpuIdArray_[in]);
			checkCudaErrors(cudaMalloc(&ptrGpuArray_[in], size_ * sizeof(T)));
		}
	}

	/**
	������gpu�ϵ��м��������
	*/
	void allocateMediumPtrGpu()
	{
		ptrMediumGpuArray_.resize(gpuNum);
		ptrMediumGpuArray_[0] = nullptr;
		for (int in = 1; in < gpuNum; in++)
		{
			cudaSetDevice(0);
			checkCudaErrors(cudaMalloc(&ptrMediumGpuArray_[in],sharedSize_[in]*sizeof(T)));
		}
	}
};
