#pragma once
#ifndef _LIO_SAM_RBF_CUDA_WHEEL_GRADIENT_KERNELS_CUH_
#define _LIO_SAM_RBF_CUDA_WHEEL_GRADIENT_KERNELS_CUH_

#include <vector>

namespace lio_sam_rbf
{
namespace cuda_kernels
{
bool ComputeWheelResidualJacobianCuda(
    const std::vector<float>& rbf_x,
    const std::vector<float>& rbf_y,
    const std::vector<float>& rbf_w,
    float sigma_x,
    float sigma_y,
    const std::vector<float>& contact_xyz,
    const std::vector<float>& contact_jacobian,
    std::vector<float>& residuals,
    std::vector<float>& jacobians);
}
}

#endif
