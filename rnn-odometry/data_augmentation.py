import numpy as np
from scipy.interpolate import CubicSpline


def add_noise(data, noise_level=0.05):
    noise = np.random.normal(loc=0, scale=noise_level, size=data.shape)
    return data + noise

def time_warp(data, sigma=0.2, knot=4):
    orig_steps = np.arange(data.shape[0])
    random_warps = np.random.normal(loc=1.0, scale=sigma, size=(knot + 2,))
    warp_steps = np.linspace(0, data.shape[0] - 1, num=knot + 2)
    warp_steps = np.clip(warp_steps, 0, data.shape[0] - 1)  # Ensure warp_steps are within bounds
    warped_steps = np.cumsum(random_warps)
    warped_steps = (warped_steps / warped_steps[-1]) * (data.shape[0] - 1)
    warped_steps = np.clip(warped_steps, 0, data.shape[0] - 1)  # Ensure warped_steps are within bounds

    spline = CubicSpline(warp_steps, warped_steps)
    new_steps = spline(orig_steps)
    new_steps = np.clip(new_steps, 0, data.shape[0] - 1).astype(int)  # Ensure new_steps are within bounds

    return data[new_steps]

def augment_batch(batch):
    augmented = []
    for sequence in batch:
        if np.random.rand() < 0.5:
            sequence = add_noise(sequence)
        if np.random.rand() < 0.5:
            sequence = time_warp(sequence)
        augmented.append(sequence)
    return np.array(augmented)

def time_warp_batch(batch, sigma=0.2, knot=4):
    augmented = []
    for sequence in batch:
        sequence = time_warp(sequence, sigma, knot)
        augmented.append(sequence)
    return np.array(augmented)

def time_warp_batch(batch, targets, sigma=0.2, knot=4):
    augmented_batch = []
    augmented_targets = []
    index = 0
    for sequence, target in zip(batch, targets):
        index += 1
        # Debug print with an index to indicate that the time warp is being applied
        print("Time warp applied to index: ", index)
        warped_sequence = time_warp(sequence, sigma, knot)
        warped_target = time_warp(target, sigma, knot)
        augmented_batch.append(warped_sequence)
        augmented_targets.append(warped_target)
    return np.array(augmented_batch), np.array(augmented_targets)
