#include "SandGrid.h"

#include "SSEUtil.h"
#include <iostream>
#include "Core/Utility/cuda_utilities.h"
#include "Core/Utility/CudaRand.h"

//#include <cuda_runtime.h>
//#ifndef __CUDACC__
//#define __CUDACC__
//#include "cuda_texture_types.h"
//#endif

namespace PhysIKA {
using namespace SSEUtil;

// texture memory reference can only be global variable
texture<float4, 2, cudaReadModeElementType> texture_grid;
// texture<float, 2, cudaReadModeElementType> texture_land;
texture<float4, 2, cudaReadModeElementType> texture_mu;

SandGrid::SandGrid(int in_x, int in_y, float slide1, float mu1, float drag1)
{
    this->resize(in_x, in_y);

    block_x = 16;
    block_y = 16;

    slide = slide1;
    mu    = mu1;
    drag  = drag1;

    //m_visualScale = make_float3(1.0, 1.0, 1.0);
    //m_visualTranslation = make_float3(0.0, 0.0, 0.0);
}

SandGrid::~SandGrid()
{
    m_sandGrid.Release();
    //m_landHeight.Release();
    //m_sandHeight.Release();
}

__host__ __device__ float4 d_gridpointToVertex(float4 gp, float x, float y, int g_Nx, int g_Ny, float dl)
{
    float  h = gp.x;
    float4 v;
    v.x = (x - g_Nx / 2) * dl;
    v.z = (y - g_Ny / 2) * dl;
    v.y = (h + gp.w);

    return v;
}

__host__ __device__ rgb d_gridpointToColor(float4 gp)
{
    rgb c;
    // 	c.x = min(130 + (gp.x + gp.w - NN) * 200.0f, 255);
    // 	c.y = min(80 + (gp.x + gp.w - NN) * 200.0f, 255);
    // 	c.z = min(40 + (gp.x + gp.w - NN) * 200.0f, 255);
    // 	c.w = -max(-50 * gp.x + 50, 0);
    c.x = 130;
    c.y = 80;
    c.z = 40;
    c.w = 255;

    return c;
}

//void make_dimension2D(int width, int height, int bx, int by, dim3 &gridDim, dim3 &blockDim)
//{
//	gridDim.x = (width + bx - 1) / bx;
//	gridDim.y = (height + by - 1) / by;
//	gridDim.z = 1;
//
//	blockDim.x = bx;
//	blockDim.y = by;
//	blockDim.z = 1;
//}

__global__ void g_initGrid(float4* grid, int pitch, float* wavegrid, int wavepitch, float* landgrid, int landpitch, int width, int height)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    if (x < width && y < height)
    {
        float4 gp;
        gp.x = grid2Dread(wavegrid, x, y, wavepitch) - grid2Dread(landgrid, x, y, landpitch);
        gp.x = gp.x < 0.0f ? 0.0f : gp.x;
        gp.y = 0.0f;
        gp.z = 0.0f;
        gp.w = grid2Dread(landgrid, x, y, landpitch);

        grid2Dwrite(grid, x, y, pitch, gp);
    }
}

__global__ void g_initGrid_mu(float4* grid, int pitch, float* wavegrid, int wavepitch, float* landgrid, int landpitch, float* mugrid, int mupitch, int width, int height)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x < width && y < height)
	{
		float4 gp;
		gp.x = grid2Dread(wavegrid, x, y, wavepitch) - grid2Dread(landgrid, x, y, landpitch);
		gp.x = gp.x < 0.0f ? 0.0f : gp.x;
		gp.y = 0.0f;//grid2Dread(mugrid, x, y, mupitch);
		gp.y = gp.y < 0.0f ? 0.0f : gp.y;
		gp.z = 0.0f;
		gp.w = grid2Dread(landgrid, x, y, landpitch);

		grid2Dwrite(grid, x, y, pitch, gp);
	}
}
//艹，先把这个看懂吧！
void SandGrid::initialize(float* land_height, float* sand_height)
{
    // allocate memory
    // m_sandGrid.Resize(Nx, Ny);

    // copy sand height data to GPU
    DeviceArrayPitch2D1f initSurface;//把形参1拷到这里来
    initSurface.Resize(Nx, Ny);
    cudaMemcpy2D(initSurface.GetDataPtr(), initSurface.Pitch() * sizeof(float), sand_height, Nx * sizeof(float), Nx * sizeof(float), Ny, cudaMemcpyHostToDevice);

    // copy land data to GPU
    DeviceArrayPitch2D1f landHeight;//把形参2拷到这里来
    landHeight.Resize(Nx, Ny);
    cudaMemcpy2D(landHeight.GetDataPtr(), landHeight.Pitch() * sizeof(float), land_height, Nx * sizeof(float), Nx * sizeof(float), Ny, cudaMemcpyHostToDevice);

    dim3 gridDims, blockDims;
    make_dimension2D(Nx, Ny, block_x, block_y, gridDims, blockDims);//16个作为一个block，block=16，grid是看看一个grid有多少block
    int sandGridPtich = m_sandGrid.Pitch();//128
	//printf("sandGridPtich=%d", sandGridPtich);
    int surfacePitch  = initSurface.Pitch();
    int landPitch     = landHeight.Pitch();
    g_initGrid<<<gridDims, blockDims>>>(m_sandGrid.GetDataPtr(), m_sandGrid.Pitch(), initSurface.GetDataPtr(), initSurface.Pitch(), landHeight.GetDataPtr(), landHeight.Pitch(), Nx, Ny);
	//上面这行，两个高度场写到m_sandGrid里。mu版本的话要把三个都写到这里！！！
	cuSynchronize();

    //SSEUtil::bindTexture2D(texture_grid, m_sandGrid);//这个可以去掉呀！

    m_landHeight.setSpace(dl, dl);
    m_sandHeight.setSpace(dl, dl);
    this->updateSandGridHeight();
    this->updateLandGridHeight();//m_sandGrid数据导入m_landHeight里
	
}
//网格初始化函数，在demo里面直接调用。现在就差这里没把mu导进去！
//这个函数有bug，，，跑不通就离谱？！！！！
void SandGrid::mud_initialize(float* land_height, float* sand_height, float* mud_mu)
{
	// allocate memory
	// m_sandGrid.Resize(Nx, Ny);

	// copy sand height data to GPU
	DeviceArrayPitch2D1f initSurface;
	initSurface.Resize(Nx, Ny);
	cudaMemcpy2D(initSurface.GetDataPtr(), initSurface.Pitch() * sizeof(float), sand_height, Nx * sizeof(float), Nx * sizeof(float), Ny, cudaMemcpyHostToDevice);

	// copy land data to GPU
	DeviceArrayPitch2D1f landHeight;
	landHeight.Resize(Nx, Ny);
	cudaMemcpy2D(landHeight.GetDataPtr(), landHeight.Pitch() * sizeof(float), land_height, Nx * sizeof(float), Nx * sizeof(float), Ny, cudaMemcpyHostToDevice);

	// copy mu data to GPU
	DeviceArrayPitch2D1f muValue;
	muValue.Resize(Nx, Ny);
	cudaMemcpy2D(muValue.GetDataPtr(), muValue.Pitch() * sizeof(float), mud_mu, Nx * sizeof(float), Nx * sizeof(float), Ny, cudaMemcpyHostToDevice);

	//下面只写了那俩高度场，没写mu高度场！所以没写进去
	dim3 gridDims, blockDims;
	make_dimension2D(Nx, Ny, block_x, block_y, gridDims, blockDims);
	int sandGridPtich = m_sandGrid.Pitch();
	int surfacePitch = initSurface.Pitch();
	int landPitch = landHeight.Pitch();
	int muPitch = muValue.Pitch();

	//g_initGrid <<<gridDims, blockDims >>> (m_sandGrid.GetDataPtr(), m_sandGrid.Pitch(), initSurface.GetDataPtr(), initSurface.Pitch(), landHeight.GetDataPtr(), landHeight.Pitch(), Nx, Ny);
	//cuSynchronize();//※※※※※上面这行与下面调用的updateSandGridHeight()具有必然联系，有了这行，下面才不报错。所以，建立全新的一套下面的update三联！做成m_muGrid配套的。

	//写到m_muGrid里面，替上面那行！！！！
	g_initGrid_mu <<<gridDims, blockDims >>> (m_muGrid.GetDataPtr(), m_muGrid.Pitch(), initSurface.GetDataPtr(), initSurface.Pitch(), landHeight.GetDataPtr(), landHeight.Pitch(), muValue.GetDataPtr(), muValue.Pitch(), Nx, Ny);
	//上面新加的这行，加了就有bug//问题在于m_muGrid好像没有初始化，上面的m_sandGrid是初始化了的！、//莫不如就把这套代码存一下，然后都在m_sandGrid变量上面改好了？！
	cuSynchronize();//注释掉了

	SSEUtil::bindTexture2D(texture_grid, m_sandGrid);//这个是把 m_sandGrid绑在texture_grid上了，我需要把mu的绑在texture_mu上
	//前后这两行可能没用。。
	SSEUtil::bindTexture2D(texture_mu, m_muGrid);//m_muGrid绑定到texture_mu

	m_landHeight.setSpace(dl, dl);
	m_sandHeight.setSpace(dl, dl);//总之是要把数值赋给m_sandHeight！
	m_muValue.setSpace(dl, dl);
	
	//纳闷下面为啥出错？//把那个bind去掉就好了！//但是，还是没导进去mu
	this->updateLandGridHeight();//这里出错！//因为函数里面更新的是sandgrid而不是mugrid，/***所以要新写一个这个函数的mu版本。***/
	this->updateSandGridHeight();//这里出错!
	//this->updatemuGridValue();//这里也出错！难道是texture那里？

	//0610现在就差最后这两行！！

}

//__global__ void g_visualize_sand_height(float4* watersurfacevertices, rgb* watersurfacecolors, int width, int height, float dl)
//{
//	int x = threadIdx.x + blockIdx.x * blockDim.x;
//	int y = threadIdx.y + blockIdx.y * blockDim.y;

//	if (x < width && y < height)
//	{
//		int gridx = x + 1;
//		int gridy = y + 1;

//		float4 gp = tex2D(texture_grid, gridx, gridy);
//		watersurfacevertices[y * width + x] = d_gridpointToVertex(gp, x, y, width, height, dl);
//		watersurfacecolors[y * width + x] = d_gridpointToColor(gp);
//	}
//}

//void SandGrid::vis(float4* watersurfacevertices, rgb* watersurfacecolors)
//{
//	dim3 gridDims, blockDims;
//	make_dimension2D(Nx, Ny, block_x, block_y, gridDims, blockDims);
//	g_visualize_sand_height << < gridDims, blockDims >> > (vis_grid.GetDataPtr(), vis_color.GetDataPtr(),
//		Nx, Ny, dl);
//	cudaThreadSynchronize();
//	cudaError_t err = cudaGetLastError();

//	// copy data
//	cudaMemcpy2D(watersurfacevertices, Nx * sizeof(float4), vis_grid.GetDataPtr(), vis_grid.Pitch() * sizeof(float4),
//		Nx * sizeof(float4), Ny, cudaMemcpyDeviceToHost);
//	cudaMemcpy2D(watersurfacecolors, Nx * sizeof(rgb), vis_color.GetDataPtr(), vis_color.Pitch() * sizeof(rgb),
//		Nx * sizeof(rgb), Ny, cudaMemcpyDeviceToHost);
//}

//__global__ void g_visualizeSurfaceCoord(DeviceArray<Vector3f> surcoord, int width, int height, float dl)
//{
//	int x = threadIdx.x + blockIdx.x * blockDim.x;
//	int y = threadIdx.y + blockIdx.y * blockDim.y;

//	if (x < width && y < height)
//	{
//		int gridx = x + 1;
//		int gridy = y + 1;

//		float4 gp = tex2D(texture_grid, gridx, gridy);
//		float4 pos = d_gridpointToVertex(gp, x, y, width, height, dl);
//		surcoord[y * width + x] = Vector3f(pos.x, pos.y, pos.z);
//	}
//}

//void SandGrid::vis(DeviceArray<Vector3f>& visualCoord)
//{
//	dim3 gridDims, blockDims;
//	make_dimension2D(Nx, Ny, block_x, block_y, gridDims, blockDims);
//	g_visualizeSurfaceCoord << <gridDims, blockDims >> > (visualCoord, Nx, Ny, dl);
//	cuSynchronize();
//}

void SandGrid::getSandGridInfo(SandGridInfo& info)
{
    info.nx    = Nx;
    info.ny    = Ny;
    info.pitch = m_sandGrid.Pitch();
    info.data  = m_sandGrid.GetDataPtr();

    info.sandRho = m_sandRho;
    info.mu      = mu;
    info.slide   = this->slide;
    info.drag    = this->drag;

    info.griddl = dl;
}

void SandGrid::setSandInfo(const SandGridInfo& sandinfo)
{
    this->resize(sandinfo.nx, sandinfo.ny);

    slide     = sandinfo.slide;
    drag      = sandinfo.drag;
    mu        = sandinfo.mu;
    m_sandRho = sandinfo.sandRho;

    dl = sandinfo.griddl;
}

bool SandGrid::resize(int nx, int ny)
{
    if (Nx != nx || Ny != ny)
    {
        Nx = nx;
        Ny = ny;

        m_sandGrid.Resize(nx, ny);
        //m_landHeight.Resize(nx, ny);
        //m_sandHeight.Resize(nx, ny);
        return true;
    }

    return false;
}

__global__ void SandGrid_updateSandGridHeight(
    DeviceHeightField1d  sandheight,
    DeviceArrayPitch2D4f grid)
{
    int tidx = threadIdx.x + blockDim.x * blockIdx.x;
    int tidy = threadIdx.y + blockDim.y * blockIdx.y;

    if (tidx >= grid.Nx() || tidy >= grid.Ny())
        return;
    gridpoint gp           = grid(tidx, tidy);
    sandheight(tidx, tidy) = gp.x;
}

__global__ void SandGrid_updateLandGridHeight(
	DeviceHeightField1d  landheight,
	DeviceArrayPitch2D4f grid)
{
	int tidx = threadIdx.x + blockDim.x * blockIdx.x;
	int tidy = threadIdx.y + blockDim.y * blockIdx.y;

	if (tidx >= grid.Nx() || tidy >= grid.Ny())
		return;
	gridpoint gp = grid(tidx, tidy);
	landheight(tidx, tidy) = gp.w;
}

__global__ void SandGrid_updateMuGridValue(
	DeviceHeightField1d muvalue,
	DeviceArrayPitch2D4f grid)
{
	int tidx = threadIdx.x + blockDim.x * blockIdx.x;
	int tidy = threadIdx.y + blockDim.y * blockIdx.y;

	if (tidx >= grid.Nx() || tidy >= grid.Ny())
		return;
	gridpoint gp = grid(tidx, tidy);
	muvalue(tidx, tidy) = gp.x;
}

void SandGrid::updateSandGridHeight()
{
    if (m_sandHeight.Nx() != m_sandGrid.Nx() || m_sandHeight.Ny() != m_sandGrid.Ny())
    {
        m_sandHeight.resize(m_sandGrid.Nx(), m_sandGrid.Ny());
    }

    uint3 gsize = { m_sandGrid.Nx(), m_sandGrid.Ny(), 1 };
    cuExecute2D(gsize, SandGrid_updateSandGridHeight, m_sandHeight, m_sandGrid);
}

void SandGrid::updateLandGridHeight()
{
	//resize
	if (m_landHeight.Nx() != m_sandGrid.Nx() || m_landHeight.Ny() != m_sandGrid.Ny())
	{
		m_landHeight.resize(m_sandGrid.Nx(), m_sandGrid.Ny());
	}

	uint3 gsize = { m_sandGrid.Nx(), m_sandGrid.Ny(), 1 };
	cuExecute2D(gsize, SandGrid_updateLandGridHeight, m_landHeight, m_sandGrid);
}

void SandGrid::updatemuGridValue()
{
	if (m_muValue.Nx() != m_sandGrid.Nx() || m_muValue.Ny() != m_sandGrid.Ny())
	{
		m_muValue.resize(m_sandGrid.Nx(), m_sandGrid.Ny());
	}

	uint3 gsize = { m_sandGrid.Nx(), m_sandGrid.Ny(), 1 };
	cuExecute2D(gsize, SandGrid_updateSandGridHeight, m_muValue, m_muGrid);
}



__global__ void SandGrid_setSandHeightToFloat4(
    DeviceArrayPitch2D4f grid,
    DeviceHeightField1d  sandheight)
{
    int tidx = threadIdx.x + blockDim.x * blockIdx.x;
    int tidy = threadIdx.y + blockDim.y * blockIdx.y;

    if (tidx >= grid.Nx() || tidy >= grid.Ny())
        return;
    gridpoint gp     = grid(tidx, tidy);
    gp.x             = sandheight(tidx, tidy);
    grid(tidx, tidy) = gp;
}

void SandGrid::setSandGridHeight(DeviceHeightField1d& sandheight)
{
    Function1Pt::copy(m_sandHeight, sandheight);

    uint3 gsize = { m_sandGrid.Nx(), m_sandGrid.Ny(), 1 };
    cuExecute2D(gsize, SandGrid_setSandHeightToFloat4, m_sandGrid, m_sandHeight);
}

void SandGrid::setSandGridHeight(HostHeightField1d& sandheight)
{
    Function1Pt::copy(m_sandHeight, sandheight);

    uint3 gsize = { m_sandGrid.Nx(), m_sandGrid.Ny(), 1 };
    cuExecute2D(gsize, SandGrid_setSandHeightToFloat4, m_sandGrid, m_sandHeight);
}

void SandGrid::setSandGridHeight(double* sandheight)
{
    int width = m_sandHeight.Nx() * sizeof(double);
    cuSafeCall(
        cudaMemcpy2D(m_sandHeight.GetDataPtr(), m_sandHeight.Pitch() * sizeof(double), sandheight, width, width, m_sandHeight.Ny(), cudaMemcpyHostToDevice));

    uint3 gsize = { m_sandGrid.Nx(), m_sandGrid.Ny(), 1 };
    cuExecute2D(gsize, SandGrid_setSandHeightToFloat4, m_sandGrid, m_sandHeight);
}

void SandGrid::getSandGridHeight(double* sandheight)
{
    int width = m_sandHeight.Nx() * sizeof(double);
    cuSafeCall(
        cudaMemcpy2D(sandheight, width, m_sandHeight.GetDataPtr(), m_sandHeight.Pitch() * sizeof(double), width, m_sandHeight.Ny(), cudaMemcpyDeviceToHost));
}

}  // namespace PhysIKA
