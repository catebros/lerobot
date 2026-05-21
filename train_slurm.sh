#!/bin/bash
#SBATCH --job-name=lerobot-bowl
#SBATCH --output=%x_%j.log
#SBATCH --error=%x_%j.err
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=24:00:00
# Adjust partition if needed: check available with `sinfo`
# #SBATCH --partition=gpu

export HUGGING_FACE_HUB_TOKEN="your_token_here"
export APPTAINER_CACHEDIR=$HOME/.apptainer_cache
export APPTAINER_TMPDIR=$HOME/.apptainer_tmp

SIF=$HOME/apptainer/lerobot-train.sif
OUTPUTS=$HOME/runs
HF_CACHE=$HOME/hf_cache

apptainer exec --nv \
  --bind $OUTPUTS:/lerobot/outputs \
  --bind $HF_CACHE:$HOME/.cache/huggingface \
  $SIF \
  python3 -m lerobot.scripts.lerobot_train \
    --dataset.repo_id=catebros/w250-bowl-pickplace \
    --policy.type=act \
    --policy.chunk_size=30 \
    --policy.n_action_steps=30 \
    --batch_size=32 \
    --steps=100000 \
    --save_freq=5000 \
    --log_freq=100 \
    --num_workers=8 \
    --dataset.video_backend=pyav \
    --output_dir=outputs/train/act_w250_bowl_v1 \
    --policy.push_to_hub=true \
    --policy.repo_id=catebros/act-w250-bowl-pickplace
