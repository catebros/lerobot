FROM pytorch/pytorch:2.1.0-cuda12.1-cudnn8-runtime

# Set working directory
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*


ARG REPO_URL=https://github.com/catebros/lerobot
RUN git clone ${REPO_URL} /workspace/lerobot

# Install lerobot and dependencies
RUN cd /workspace/lerobot && \
    pip install --no-cache-dir -e . && \
    pip install --no-cache-dir datasets huggingface_hub

# Set up HuggingFace cache directory
ENV HF_HOME=/workspace/hf_cache
ENV HUGGINGFACE_HUB_CACHE=/workspace/hf_cache

# Create output directory
RUN mkdir -p /workspace/outputs

WORKDIR /workspace/lerobot

# Default command (can be overridden)
CMD ["bash"]
