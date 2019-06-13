#!/usr/bin/env python3

import numpy as np
import hashlib
import cv2
import io
import warnings
import matplotlib as mpl
import gc
mpl.use('Agg')
import matplotlib.pyplot as plt


def draw(img, px, X, colours=None):

  # hash of the image file for sorting later
  img_hash = hashlib.md5()
  img_hash.update(img)
  img_hash = img_hash.digest()

  # Decode the image
  img = cv2.cvtColor(cv2.imdecode(np.fromstring(img, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

  # Setup the display so everything is all at the correct resolution
  dpi = 80
  height, width, channels = img.shape
  figsize = width / float(dpi), height / float(dpi)
  fig = plt.figure(figsize=figsize)
  ax = fig.add_axes([0, 0, 1, 1])
  ax.axis('off')

  # Image underlay
  ax.imshow(img, interpolation='nearest')

  # We need at least 3 points to make a triangle
  if px.shape[0] >= 3:

    # Stop matplotlib complaining
    with warnings.catch_warnings():
      warnings.simplefilter("ignore")

      # Now for each class, produce a contour plot
      if colours is not None:
        for i, colour in enumerate(colours):
          colour = np.array(colour) / 255
          ax.tricontour(
            px[:, 1],
            px[:, 0],
            X[:, i],
            levels=[0.5, 0.75, 0.9],
            colors=[(*colour, 0.33), (*colour, 0.66), (*colour, 1.0)]
          )
      # Or just produce a heatmap
      else:
        ax.tricontour(
          px[:, 1],
          px[:, 0],
          X,
          levels=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
          cmap=plt.get_cmap('jet'),
        )

  ax.set(xlim=[0, width], ylim=[height, 0], aspect=1)

  # Write the image as a jpg to a BytesIO and return it
  data = io.BytesIO()
  fig.savefig(data, format='jpg', dpi=dpi)
  ax.cla()
  fig.clf()
  plt.close(fig)
  gc.collect()
  data.seek(0)
  return (img_hash, height, width, data.read())
