#pragma once
#include"node.h"
#include"element.h"
//ls:2020-03-17 (���޸�)

struct NodeCuda;
struct Element;

typedef struct Accelermeter:Element
{
	int node_id_array[3];
	NodeCuda* node_array[3]; //ls:focus

	//���޸�
	int accelerometerId;

}Accelermeter;