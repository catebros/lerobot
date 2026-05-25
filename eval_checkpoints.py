"""Compute training loss for each checkpoint to identify the best one."""
import os
import torch
from pathlib import Path
from torch.utils.data import DataLoader
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy

CHECKPOINTS_DIR = Path("outputs/train/act_w250_bowl_v1/checkpoints")
DATASET_REPO_ID = "catebros/w250-bowl-pickplace"
BATCH_SIZE = 32
NUM_BATCHES = 20  # samples ~640 frames per checkpoint, fast enough

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

dataset = LeRobotDataset(DATASET_REPO_ID, video_backend="pyav")
dataloader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=4)

checkpoints = sorted([
    d for d in CHECKPOINTS_DIR.iterdir()
    if d.is_dir() and d.name != "last"
])

print(f"{'Step':>10} | {'Avg Loss':>12}")
print("-" * 26)

for ckpt in checkpoints:
    model_path = ckpt / "pretrained_model"
    try:
        policy = ACTPolicy.from_pretrained(str(model_path))
        policy = policy.to(device)
        policy.eval()

        total_loss = 0.0
        batches = 0
        with torch.no_grad():
            for i, batch in enumerate(dataloader):
                if i >= NUM_BATCHES:
                    break
                batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v
                         for k, v in batch.items()}
                loss, _ = policy.forward(batch)
                total_loss += loss.item()
                batches += 1

        avg_loss = total_loss / batches
        print(f"{ckpt.name:>10} | {avg_loss:>12.6f}")
    except Exception as e:
        print(f"{ckpt.name:>10} | ERROR: {e}")
