#!/usr/bin/env python3

from fractions import Fraction

import numpy as np


def possible_lines(rows, cols):

    # Convert the coordinates into grid coordinates and offset to get the original
    coords = np.stack(np.meshgrid(range(rows), range(cols), indexing="ij"), axis=-1)
    coords[:, :, 1] = coords[:, :, 1] * 2 + coords[:, :, 0] % 2

    # Flatten into a list so we can get all the combinations
    coords = np.reshape(coords, (-1, 2))

    gradients = {}
    offsets = set()
    for i, a in enumerate(coords):
        for b in coords[i + 1 :]:

            # Calculate what we should step at to find other points
            offset = b - a
            if offset[0] == 0:
                step = np.array([0, 1])
            elif offset[1] == 0:
                step = np.array([1, 0])
            else:
                f = Fraction(*offset)
                step = np.array([f.numerator, f.denominator])

            # Gather all the other points that would be on this line
            line = [(a[0], a[1])]
            p = a + step
            while (0 <= p[0] < rows) and (0 <= p[1] < (cols * 2)):
                if p[1] % 2 == p[0] % 2:  # Odd or even row
                    line.append((p[0], p[1]))
                p = p + step
            p = a - step
            while (0 <= p[0] < rows) and (0 <= p[1] < (cols * 2)):
                if p[1] % 2 == p[0] % 2:  # Odd or even row
                    line.append((p[0], p[1]))
                p = p - step

            # Sort these points so they always appear in the same order
            line.sort()

            # Store in the list ensuring we don't have duplicates
            grad_key = (step[0], step[1])
            start_key = line[0]
            if grad_key not in gradients:
                gradients[grad_key] = {}
            gradients[grad_key][start_key] = line

    # Filter out any line that has less than 3 points, and any gradient that has less than 3 lines
    gradients = {k: [lines[k2] for k2 in sorted(lines.keys()) if len(lines[k2]) >= 2] for k, lines in gradients.items()}
    gradients = {k: lines for k, lines in gradients.items() if len(lines) >= 2}

    # Flatten out the start values and divide the 2nd coordinate to get back to asymmetric coordinates
    return {
        k: [np.array([(p[0] * cols + p[1] // 2) for p in line]) for line in lines] for k, lines in gradients.items()
    }
