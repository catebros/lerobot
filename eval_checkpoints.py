"""Compute training loss for each checkpoint to identify the best one."""
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="torchvision")
import torch
from pathlib import Path
from torch.utils.data import DataLoader

from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.datasets.factory import resolve_delta_timestamps
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors

CHECKPOINTS_DIR = Path("outputs/train/act_w250_bowl_v1/checkpoints")
DATASET_REPO_ID = "catebros/w250-bowl-pickplace"
BATCH_SIZE = 32
NUM_BATCHES = 20

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

checkpoints = sorted([
    d for d in CHECKPOINTS_DIR.iterdir()
    if d.is_dir() and d.name != "last"
])

# Load policy config from first checkpoint to resolve delta_timestamps
first_ckpt = checkpoints[0] / "pretrained_model"
policy_cfg = ACTPolicy.from_pretrained(str(first_ckpt)).config
ds_meta = LeRobotDatasetMetadata(DATASET_REPO_ID)
delta_timestamps = resolve_delta_timestamps(policy_cfg, ds_meta)

dataset = LeRobotDataset(
    DATASET_REPO_ID,
    delta_timestamps=delta_timestamps,
    video_backend="pyav",
)
dataloader = DataLoader(
    dataset,
    batch_size=BATCH_SIZE,
    shuffle=True,
    num_workers=4,
    pin_memory=device.type == "cuda",
    drop_last=False,
)

print(f"{'Step':>10} | {'Avg Loss':>12}")
print("-" * 26)

for ckpt in checkpoints:
    model_path = ckpt / "pretrained_model"
    try:
        policy = ACTPolicy.from_pretrained(str(model_path))
        policy = policy.to(device)
        policy.train()  # VAE encoder only runs in training mode

        preprocessor, _ = make_pre_post_processors(
            policy_cfg=policy.config,
            pretrained_path=str(model_path),
        )

        total_loss = 0.0
        batches = 0
        with torch.no_grad():
            for i, batch in enumerate(dataloader):
                if i >= NUM_BATCHES:
                    break
                batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v
                         for k, v in batch.items()}
                batch = preprocessor(batch)
                loss, _ = policy.forward(batch)
                total_loss += loss.item()
                batches += 1

        avg_loss = total_loss / batches
        print(f"{ckpt.name:>10} | {avg_loss:>12.6f}")
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"{ckpt.name:>10} | ERROR: {e}")
