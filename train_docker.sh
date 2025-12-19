#!/bin/bash

set -e  # Exit on error

echo "=========================================="
echo "LeRobot Training - OPTIMIZED FOR BEST PERFORMANCE"
echo "=========================================="
echo "Dataset: catebros/my_widowx_dataset (from HuggingFace Hub)"
echo "Policy: Diffusion Policy (ResNet34 backbone)"
echo "Training steps: 200,000"
echo "Output: /workspace/outputs/widowx_diffusion_best"
echo "=========================================="
echo ""
echo "Training will automatically:"
echo "  - Download dataset from HuggingFace Hub"
echo "  - Save checkpoints every 5,000 steps"
echo "  - Upload best model to HuggingFace Hub"
echo ""
echo "Starting training..."
echo ""

# Run the training
lerobot-train --config_path=/workspace/lerobot/train_diffusion_simple.yaml

echo ""
echo "=========================================="
echo "Training completed successfully!"
echo "=========================================="
echo "Model saved to: /workspace/outputs/widowx_diffusion_best"
echo "View logs: tensorboard --logdir=/workspace/outputs/widowx_diffusion_best/logs"
echo "=========================================="
