#pragma once
#include"cuda_runtime.h"
#include"device_launch_parameters.h"
#define TestPairOfNode 64
#define AssemblyClearFlag

struct ContactNodeCuda;
struct SegmentCuda;

/**
gpu���Զ��ж����ɵ�ֵ
*/
typedef struct TestJudgeReturn
{
	/**
	type=0�����γɽӴ��� =1����Կ� =2����Ա� =3����Ե�
	*/
	int type;
	int edge_or_node;
	int position;
	SegmentCuda *segment;
	double dist;

}TestJudgeReturn;

typedef struct TestPairCuda
{
	/*int contact_node_id;*////@�Ƿ񳹵�ɾ������Ҫ��һ���۲�

	/**
	��ɲ��ԶԵ��������
	*/
	int predict_cycle[TestPairOfNode];
	int position[TestPairOfNode];
	int relative_position[TestPairOfNode];
	SegmentCuda* test_segment[TestPairOfNode];

	/*int test_segment_id[TestPairOfNode];*////@�Ƿ񳹵�ɾ������Ҫ��һ���۲�
	/*double cos_value[TestPairOfNode];*////@�Ƿ񳹵�ɾ������Ҫ��һ���۲�

	/**
	��Ч�Ĳ��Զԣ�ֵ���ܳ���TestPairOfNode
	*/
	int effective_num;

	/**
	��ɲ��ԶԵĽӴ�����Ӵ���
	*/
	ContactNodeCuda* contactNode;

	/**
	���Զ���һ�������־
	*/
	int further_cal_sign;

	__host__ __device__ TestPairCuda();

	/**
	������ײ��
	*/
	__device__ void produceContactPairGpu(SegmentCuda *segment_temp, const int cycleNum, const int current_segment_id,
		TestJudgeReturn &judge_result, const int mstp);
	__host__ void produceContactPairCpu(SegmentCuda *segment_temp, const int cycleNum, const int current_segment_id,
		TestJudgeReturn &judge_result, const int mstp);

	/**
	���Զ����ɵ�Ա߻��Ե��ж�
	*/
	__device__ void nodeToEdgeJudgeGpu(SegmentCuda *segment_temp, const int current_segment_id, TestJudgeReturn &judge_result,
		const int numberOfEdge, const int mstp, const double normVec[3]);
	__host__ void nodeToEdgeJudgeCpu(SegmentCuda *segment_temp, const int current_segment_id, TestJudgeReturn &judge_result,
		const int numberOfEdge, const int mstp, const double normVec[3]);

	/**
	�����Ǳ�Ե�Ӵ��ԽӴ��жϵ�Ӱ��
	*/
	__device__ void nodeToNoThickEdgeJudgeGpu(SegmentCuda *segment_temp, const int current_segment_id, TestJudgeReturn &judge_result,
		const int numberOfEdge, const int mstp, const double normVec[3]);

	/**
	���Զ����ɵ�Կ��ж�
	*/
	__device__ void nodeToSegmentCudaJudgeGpu(SegmentCuda *segment_temp, const int current_segment_id, TestJudgeReturn &judge_result,
		const int mstp, const double normalVec[3]);
	__host__ void nodeToSegmentCudaJudgeCpu(SegmentCuda *segment_temp, const int current_segment_id, TestJudgeReturn &judge_result,
		const int mstp, const double normalVec[3]);

	/**
	���ԶԽ�һ��������
	*/
	__host__ __device__ bool furtherCalculateTest();

	/**
	���в��ԶԶ�������Ŀ���м��,����ͨ������true������false
	*/
	__host__ __device__ void testExpand();

	/**
	������ײ���Ƿ�Խ�˽Ӵ�������ƽ��
	*/
	__host__ __device__ void updatePositionAfterContactBreak(int cycNum);

	/**
	������ײ���Ƿ�Խ�˽Ӵ�������ƽ��
	*/
	__host__ __device__ void updatePositionAfterContactBreak(int tid, int cycnum);
}TestPairCuda;