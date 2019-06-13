#!/usr/bin/env python3

import os
import argparse
import yaml

import tensorflow as tf
import training.training as training
import training.testing as testing


class Config():

  def __init__(self, **config):
    for k, v in config.items():
      if type(v) == dict:
        self.__dict__[k] = Config(**v)
      elif type(v) == list:
        self.__dict__[k] = Config.objectify_list(*v)
      else:
        self.__dict__[k] = v

  def objectify_list(*config):
    out = []

    for v in config:
      if type(v) == dict:
        out.append(Config(**v))
      elif type(v) == list:
        out.append(Config.objectify_list(*v))
      else:
        out.append(v)

    return out

  def __iter__(self):
    return self.__dict__.__iter__()

  def __repr__(self):
    return self.__dict__.__repr__()


if __name__ == "__main__":

  # Parse our command line arguments
  command = argparse.ArgumentParser(description='Utility for training a Visual Mesh network')

  command.add_argument('command', choices=['train', 'test'], action='store')
  command.add_argument('config', action='store', help='Path to the configuration file for training')
  command.add_argument('output_path', nargs='?', action='store', help='Output directory to store the logs and models')

  args = command.parse_args()

  # Load our yaml file and convert it to an object
  with open(args.config) as f:
    config = yaml.load(f)
    config = Config(**config)

  output_path = 'output' if args.output_path is None else args.output_path

  # Run the appropriate action
  if args.command == 'train':
    training.train(config, output_path)

  elif args.command == 'test':
    testing.test(config, output_path)
