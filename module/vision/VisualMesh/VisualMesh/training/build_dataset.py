#!/usr/bin/env python3

from tqdm import tqdm
import sys
import re
import os
import math
import tensorflow as tf
import json
from glob import glob


def float_feature(value):
  return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))


def float_list_feature(value):
  return tf.train.Feature(float_list=tf.train.FloatList(value=value))


def int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def bytes_feature(value):
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def make_tfrecord(output_file, input_files):

  writer = tf.python_io.TFRecordWriter(output_file)

  for image_file, mask_file, meta_file in tqdm(input_files):

    with open(meta_file, 'r') as f:
      meta = json.load(f)
    with open(image_file, 'rb') as f:
      image = f.read()
    with open(mask_file, 'rb') as f:
      mask = f.read()

    # Extract meta information
    rot = meta['rotation']
    Roc = [rot[0][0], rot[0][1], rot[0][2],
           rot[1][0], rot[1][1], rot[1][2],
           rot[2][0], rot[2][1], rot[2][2],
    ] # yapf: disable
    height = meta['height']
    projection = meta['lens']['projection']
    lens_centre = meta['lens']['centre']
    fov = meta['lens']['fov']
    focal_length = meta['lens']['focal_length']

    features = {
      'image': bytes_feature(image),
      'mask': bytes_feature(mask),
      'lens/projection': bytes_feature(projection.encode('utf-8')),
      'lens/fov': float_feature(fov),
      'lens/focal_length': float_feature(focal_length),
      'lens/centre': float_list_feature(lens_centre),
      'mesh/orientation': float_list_feature(Roc),
      'mesh/height': float_feature(height),
    }

    example = tf.train.Example(features=tf.train.Features(feature=features))

    writer.write(example.SerializeToString())

  writer.close()


if __name__ == '__main__':

  image_path = sys.argv[1]
  mask_path = sys.argv[2]
  meta_path = sys.argv[3]

  image_files = glob(os.path.join(image_path, 'image*.jpg'))
  mask_files = glob(os.path.join(image_path, 'mask*.png'))
  meta_files = glob(os.path.join(image_path, 'meta*.json'))

  # Extract which numbers are in each of the folders
  image_re = re.compile(r'image([^.]+)\.jpg$')
  mask_re = re.compile(r'mask([^.]+)\.png$')
  meta_re = re.compile(r'meta([^.]+)\.json$')
  image_nums = set([image_re.search(f).group(1) for f in image_files])
  mask_nums = set([mask_re.search(f).group(1) for f in mask_files])
  meta_nums = set([meta_re.search(f).group(1) for f in meta_files])
  common_nums = image_nums & mask_nums & meta_nums

  files = [(
    os.path.join(image_path, 'image{}.jpg'.format(n)),
    os.path.join(mask_path, 'mask{}.png'.format(n)),
    os.path.join(meta_path, 'meta{}.json'.format(n)),
  ) for n in common_nums]

  nf = len(files)

  training = 0.45
  validation = 0.10

  test = (round(nf * (training + validation)), nf)
  validation = (round(nf * training), round(nf * (training + validation)))
  training = (0, round(nf * training))

  print('Making training dataset')
  make_tfrecord('training.tfrecord', files[training[0]:training[1]])
  print('Making validation dataset')
  make_tfrecord('validation.tfrecord', files[validation[0]:validation[1]])
  print('Making test dataset')
  make_tfrecord('test.tfrecord', files[test[0]:test[1]])
