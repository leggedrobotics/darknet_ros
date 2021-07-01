#include <stdio.h>

int main(int argc, char **argv){
    cudaDeviceProp dP;
    float min_cc = 3.0;
    int deviceCount, rc; 
    rc = cudaGetDeviceCount(&deviceCount);
    
    if(rc != cudaSuccess) {
        cudaError_t error = cudaGetLastError();
        printf("CUDA error: %s", cudaGetErrorString(error));
        return rc; /* Failure */
    }

    for (int i=0; i < deviceCount; i++)
    {
        rc = cudaGetDeviceProperties(&dP, 0);
        if(rc != cudaSuccess) {
            cudaError_t error = cudaGetLastError();
            printf("CUDA error: %s", cudaGetErrorString(error));
            return rc; /* Failure */
        }
        if((dP.major+(dP.minor/10)) < min_cc) {
            printf("Min Compute Capability of %2.1f required:  %d.%d found\n Not Building CUDA Code", min_cc, dP.major, dP.minor);
            return 1; /* Failure */
        } else {
            int v = dP.major*10 + dP.minor;
            if (i<deviceCount-1)
                printf("-gencode arch=compute_%d,code=sm_%d;",v,v);
            else
                printf("-gencode arch=compute_%d,code=sm_%d",v,v);
        }
    }
    return 0; /* Success */
}
