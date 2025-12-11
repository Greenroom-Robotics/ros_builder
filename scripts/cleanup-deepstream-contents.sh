#!/bin/bash
set -e

echo "Cleaning DeepStream image to reduce size..."
echo "Before cleanup:"
du -h -s / 2>/dev/null || true

# Remove unused Triton backends (~4.5GB)
echo "Removing unused Triton backends..."
rm -rf /opt/tritonserver/backends/pytorch
# rm -rf /opt/tritonserver/backends/onnxruntime
rm -rf /opt/tritonserver/backends/openvino
rm -rf /opt/tritonserver/backends/fil

# Remove sample files (~682MB)
echo "Removing DeepStream samples..."
rm -rf /opt/nvidia/deepstream/deepstream-*/samples/streams
rm -rf /opt/nvidia/deepstream/deepstream-*/samples/models

# Remove /opt development tools (~826MB)
echo "Removing development tools from /opt..."
rm -rf /opt/tritonserver/third-party-src
rm -rf /opt/nvidia/nsight-compute
rm -rf /opt/hpcx

# Remove validation/testing tools (~498MB)
echo "Removing NVIDIA validation suite..."
rm -rf /usr/share/nvidia-validation-suite

# Remove profiling tools (~453MB)
echo "Removing Nsight Systems profiler..."
rm -rf /usr/local/cuda-*/NsightSystems-cli-*

# Optional: Remove CUDA compat libraries (~193MB)
# Only remove if driver version matches CUDA version
# echo "Removing CUDA compat libraries..."
# rm -rf /usr/local/cuda-*/compat

echo "Cleanup complete!"
echo "After cleanup:"
du -h -s / 2>/dev/null || true
