import sys

import torch
import yaml

if len(sys.argv) < 3:
    print("Usage: python extract_normalization.py <checkpoint.pt> <output.yaml>")
    sys.exit(1)

checkpoint_path = sys.argv[1]
output_path = sys.argv[2]

# Load checkpoint
checkpoint = torch.load(checkpoint_path, map_location='cpu')

# Print available keys to help you find the right ones
print("Available keys in checkpoint:")
for key in checkpoint.keys():
    print(f"  - {key}")

model_dict = checkpoint['model_state_dict']

try:
    # Access the actor_obs_normalizer attributes
    obs_mean = model_dict['actor_obs_normalizer._mean'].cpu().numpy()
    obs_std = model_dict['actor_obs_normalizer._std'].cpu().numpy()
    obs_var = model_dict['actor_obs_normalizer._var'].cpu().numpy()

    # Save to YAML
    stats = {
        'obs_mean': obs_mean.flatten().tolist(),
        'obs_std': obs_std.flatten().tolist(),
        'obs_var': obs_var.flatten().tolist()
    }

    with open(output_path, 'w') as f:
        yaml.dump(stats, f, indent=2)

    print(f"\n✓ Saved normalization stats to {output_path}")
    print(f"  Observation size: {obs_mean.shape[1]}")

except KeyError as e:
    print(f"\nERROR: Key not found: {e}")
    print("\nKeys in model_state_dict:")
    for key in model_dict.keys():
        print(f"  - {key}")
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
