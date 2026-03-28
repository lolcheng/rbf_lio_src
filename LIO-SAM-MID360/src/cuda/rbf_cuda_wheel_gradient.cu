#include "rbf_cuda_wheel_gradient_kernels.cuh"
#include <cuda_runtime.h>

namespace lio_sam_rbf
{
namespace cuda_kernels
{
__global__ void WheelResidualJacobianKernel(
    const float* rbf_x,
    const float* rbf_y,
    const float* rbf_w,
    int rbf_num,
    float inv_sigma_x2,
    float inv_sigma_y2,
    const float* contact_xyz,
    const float* contact_jacobian,
    float* residuals,
    float* gradients,
    int wheel_num)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= wheel_num)
        return;

    const float px = contact_xyz[idx * 3 + 0];
    const float py = contact_xyz[idx * 3 + 1];
    const float pz = contact_xyz[idx * 3 + 2];

    float h = 0.0f;
    float dhdx = 0.0f;
    float dhdy = 0.0f;
    for (int k = 0; k < rbf_num; ++k)
    {
        const float dx = px - rbf_x[k];
        const float dy = py - rbf_y[k];
        const float phi = expf(-0.5f * (dx * dx * inv_sigma_x2 + dy * dy * inv_sigma_y2));
        const float wk = rbf_w[k];
        h += wk * phi;
        dhdx += wk * phi * (-dx * inv_sigma_x2);
        dhdy += wk * phi * (-dy * inv_sigma_y2);
    }

    const float res = pz - h;
    residuals[idx] = res;

    const float* J = contact_jacobian + idx * 18;
    for (int j = 0; j < 6; ++j)
    {
        const float dpx = J[0 * 6 + j];
        const float dpy = J[1 * 6 + j];
        const float dpz = J[2 * 6 + j];
        gradients[idx * 6 + j] = dpz - dhdx * dpx - dhdy * dpy;
    }
}

bool ComputeWheelResidualJacobianCuda(
    const std::vector<float>& rbf_x,
    const std::vector<float>& rbf_y,
    const std::vector<float>& rbf_w,
    float sigma_x,
    float sigma_y,
    const std::vector<float>& contact_xyz,
    const std::vector<float>& contact_jacobian,
    std::vector<float>& residuals,
    std::vector<float>& jacobians)
{
    residuals.clear();
    jacobians.clear();

    const int rbf_num = static_cast<int>(rbf_x.size());
    if (rbf_num == 0 || rbf_y.size() != rbf_x.size() || rbf_w.size() != rbf_x.size())
        return false;
    if (contact_xyz.size() % 3 != 0 || contact_jacobian.size() % 18 != 0)
        return false;
    const int wheel_num = static_cast<int>(contact_xyz.size() / 3);
    if (wheel_num == 0 || static_cast<int>(contact_jacobian.size() / 18) != wheel_num)
        return false;
    if (sigma_x <= 1e-6f || sigma_y <= 1e-6f)
        return false;

    float *d_rbf_x = nullptr, *d_rbf_y = nullptr, *d_rbf_w = nullptr;
    float *d_contact_xyz = nullptr, *d_contact_jac = nullptr;
    float *d_residuals = nullptr, *d_grads = nullptr;

    cudaError_t err = cudaSuccess;
    err = cudaMalloc(&d_rbf_x, sizeof(float) * rbf_num);
    if (err != cudaSuccess) return false;
    err = cudaMalloc(&d_rbf_y, sizeof(float) * rbf_num);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMalloc(&d_rbf_w, sizeof(float) * rbf_num);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMalloc(&d_contact_xyz, sizeof(float) * wheel_num * 3);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMalloc(&d_contact_jac, sizeof(float) * wheel_num * 18);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMalloc(&d_residuals, sizeof(float) * wheel_num);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMalloc(&d_grads, sizeof(float) * wheel_num * 6);
    if (err != cudaSuccess) goto cleanup;

    err = cudaMemcpy(d_rbf_x, rbf_x.data(), sizeof(float) * rbf_num, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_rbf_y, rbf_y.data(), sizeof(float) * rbf_num, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_rbf_w, rbf_w.data(), sizeof(float) * rbf_num, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_contact_xyz, contact_xyz.data(), sizeof(float) * wheel_num * 3, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_contact_jac, contact_jacobian.data(), sizeof(float) * wheel_num * 18, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;

    {
        const int threads = 128;
        const int blocks = (wheel_num + threads - 1) / threads;
        const float inv_sigma_x2 = 1.0f / (sigma_x * sigma_x);
        const float inv_sigma_y2 = 1.0f / (sigma_y * sigma_y);
        WheelResidualJacobianKernel<<<blocks, threads>>>(
            d_rbf_x,
            d_rbf_y,
            d_rbf_w,
            rbf_num,
            inv_sigma_x2,
            inv_sigma_y2,
            d_contact_xyz,
            d_contact_jac,
            d_residuals,
            d_grads,
            wheel_num);
    }

    err = cudaGetLastError();
    if (err != cudaSuccess) goto cleanup;
    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) goto cleanup;

    residuals.resize(wheel_num);
    jacobians.resize(wheel_num * 6);

    err = cudaMemcpy(residuals.data(), d_residuals, sizeof(float) * wheel_num, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(jacobians.data(), d_grads, sizeof(float) * wheel_num * 6, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) goto cleanup;

cleanup:
    if (d_rbf_x) cudaFree(d_rbf_x);
    if (d_rbf_y) cudaFree(d_rbf_y);
    if (d_rbf_w) cudaFree(d_rbf_w);
    if (d_contact_xyz) cudaFree(d_contact_xyz);
    if (d_contact_jac) cudaFree(d_contact_jac);
    if (d_residuals) cudaFree(d_residuals);
    if (d_grads) cudaFree(d_grads);
    return err == cudaSuccess;
}
}
}
