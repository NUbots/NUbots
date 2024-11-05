import numpy as np


def add_noise(data, noise_level=0.05):
    noise = np.random.normal(loc=0, scale=noise_level, size=data.shape)
    return data + noise

def time_warp(data, sigma=0.2, knot=4):
    from scipy.interpolate import CubicSpline
    orig_steps = np.arange(data.shape[0])
    random_warps = np.random.normal(loc=1.0, scale=sigma, size=(knot+2,))
    warp_steps = (np.cumsum(random_warps) * (data.shape[0]-1) / (knot+1)) \
                 [:knot+1]
    warper = CubicSpline(np.concatenate(([-1], warp_steps, [data.shape[0]])),
                         np.concatenate(([-1], orig_steps[warp_steps.astype(int)], [data.shape[0]])))
    return warper(orig_steps)

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
    for sequence, target in zip(batch, targets):
        warped_sequence = time_warp(sequence, sigma, knot)
        warped_target = time_warp(target, sigma, knot)
        augmented_batch.append(warped_sequence)
        augmented_targets.append(warped_target)
    return np.array(augmented_batch), np.array(augmented_targets)
