#pragma once

/*
 * �߽׵�Ԫ��ͽ׵�Ԫ��ͬ
 * ����Abaqus analysis user manual 28.1.4-20
 * ����abaqusȡ�������ڷ����ڴ˵���˳�򣬸ĳ��ⷨ��
 */
static int tetFace[4][3] =
{
	{1,3,2},
	{1,2,4},
	{2,3,4},
	{3,1,4},
};

static int tetFace2nd[16][3]=
{
	{ 1,7,5 },
	{ 5,7,6 },
	{ 7,3,6 },
	{ 6,2,5 },

	{ 1,5,8 },
	{ 2,9,5 },
	{ 4,8,9 },
	{ 9,8,5 },

	{ 2,6,9 },
	{ 3,10,6 },
	{ 4,9,10 },
	{ 9,6,10 },

	{ 1,8,7 },
	{ 4,10,8 },
	{ 3,7,10 },
	{ 10,7,8 },
};

static int quadSpos[4] = { 1,2,3,4 };
static int triaSpos[3] = { 1,2,3 };

static int quadSneg[4] = { 1,4,3,2 };
static int triaSneg[3] = { 1,3,2 };

static int pentFace[5][4]=
{
	/*Abaqus������*/
	{1,3,2,2},
	{4,5,6,6},
	{1,2,5,4},
	{2,3,6,5},
	{3,1,4,6},
	/*Ls-dyna������*/
	//{1,4,3,2},
	//{1,2,6,5},
	//{2,3,7,6},
	//{3,4,8,7},
	//{4,1,5,8},
};

static int hexaFace[6][4]=
{
	{1,4,3,2},
	{5,6,7,8},
	{1,2,6,5},
	{2,3,7,6},
	{3,4,8,7},
	{4,1,5,8},
};

static int triaFace[3][2]
{
	{1,2},
	{2,3},
	{3,1},
};

static int quadFace[4][2]
{
	{1,2},
	{2,3},
	{3,4},
	{4,1},
};