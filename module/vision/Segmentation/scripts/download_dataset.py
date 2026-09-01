#!/usr/bin/env python3
import argparse
import os
import sys
import zipfile
from urllib.parse import urlencode
from urllib.request import urlretrieve

DATA_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__), '../data'))
DOWNLOAD_LINK = 'https://data.bit-bots.de/TORSO-21'


def download_and_extract_zip(filename, file_url, folder, approx_size):
    print(f'Downloading dataset to {folder}... '
          f'This might take a lot of time and take up to {approx_size} GB of disk space')
    os.makedirs(folder, exist_ok=True)
    urlretrieve(f'{DOWNLOAD_LINK}/{file_url}', filename)
    print('Download finished, extracting data...')
    with zipfile.ZipFile(filename) as f:
        f.extractall(folder)
    os.remove(filename)
    print('Extraction finished.')


def download_file(filename, file_url):
    print(f'Downloading annotations file to {filename}...')
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    urlretrieve(f'{DOWNLOAD_LINK}/{file_url}', filename)
    print('Download finished.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Download the TORSO-21 dataset.')
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument('-a', '--all', action='store_true', help='Download the complete dataset')
    grp.add_argument('-r', '--real', action='store_true', help='Download the images captured in real environments')
    grp.add_argument('-s', '--simulation', action='store_true', help='Download the images generated from simulation')
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument('--test', action='store_true', help='Only download test data')
    grp.add_argument('--train', action='store_true', help='Only download training data')
    parser.add_argument('--annotations', action='store_true', help='Only download annotations')
    args = parser.parse_args()

    if not any(getattr(args, arg) for arg in args.__dict__):
        parser.error('Please specify which data to download. Use --help for further information.')

    if args.all and any((args.test, args.train, args.annotations)):
        parser.error('--all cannot be used with other options.')

    if args.all:
        tmp_file = os.path.join(DATA_FOLDER, 'reality.zip')
        approx_size = 11
        download_and_extract_zip(tmp_file, 'reality.zip', DATA_FOLDER, approx_size)
        tmp_file = os.path.join(DATA_FOLDER, 'simulation.zip')
        approx_size = 180
        download_and_extract_zip(tmp_file, 'simulation.zip', DATA_FOLDER, approx_size)
    elif args.real and args.test and args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'reality', 'test', 'annotations.yaml'), 'reality/test/annotations.yaml')
    elif args.real and args.test:
        tmp_file = os.path.join(DATA_FOLDER, 'reality', 'test.zip')
        folder = os.path.join(DATA_FOLDER, 'reality')
        approx_size = 2
        download_and_extract_zip(tmp_file, 'reality/test.zip', folder, approx_size)
    elif args.real and args.train and args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'reality', 'train', 'annotations.yaml'), 'reality/train/annotations.yaml')
    elif args.real and args.train:
        tmp_file = os.path.join(DATA_FOLDER, 'reality', 'train.zip')
        folder = os.path.join(DATA_FOLDER, 'reality')
        approx_size = 10
        download_and_extract_zip(tmp_file, 'reality/train.zip', folder, approx_size)
    elif args.real and args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'reality', 'train', 'annotations.yaml'), 'reality/train/annotations.yaml')
        download_file(os.path.join(DATA_FOLDER, 'reality', 'test', 'annotations.yaml'), 'reality/test/annotations.yaml')
    elif args.real:
        tmp_file = os.path.join(DATA_FOLDER, 'reality.zip')
        folder = DATA_FOLDER
        approx_size = 11
        download_and_extract_zip(tmp_file, 'reality.zip', folder, approx_size)
    elif args.simulation and args.test and args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'simulation', 'test', 'annotations.yaml'), 'simulation/test/annotations.yaml')
    elif args.simulation and args.test:
        tmp_file = os.path.join(DATA_FOLDER, 'simulation', 'test.zip')
        folder = os.path.join(DATA_FOLDER, 'simulation/test/test.zip')
        approx_size = 28
        download_and_extract_zip(tmp_file, 'simulation/test.zip', folder, approx_size)
    elif args.simulation and args.train and args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'simulation', 'train', 'annotations.yaml'), 'simulation/train/annotations.yaml')
    elif args.simulation and args.train:
        tmp_file = os.path.join(DATA_FOLDER, 'simulation', 'train.zip')
        folder = os.path.join(DATA_FOLDER, 'simulation')
        approx_size = 152
        download_and_extract_zip(tmp_file, 'simulation/train.zip', folder, approx_size)
    elif args.simulation and args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'simulation', 'train', 'annotations.yaml'), 'simulation/train/annotations.yaml')
        download_file(os.path.join(DATA_FOLDER, 'simulation', 'test', 'annotations.yaml'), 'simulation/test/annotations.yaml')
    elif args.simulation:
        tmp_file = os.path.join(DATA_FOLDER, 'simulation.zip')
        folder = DATA_FOLDER
        approx_size = 180
        download_and_extract_zip(tmp_file, 'simulation.zip', folder, approx_size)
    elif args.annotations:
        download_file(os.path.join(DATA_FOLDER, 'reality', 'train', 'annotations.yaml'), 'reality/train/annotations.yaml')
        download_file(os.path.join(DATA_FOLDER, 'reality', 'test', 'annotations.yaml'), 'reality/test/annotations.yaml')
        download_file(os.path.join(DATA_FOLDER, 'simulation', 'train', 'annotations.yaml'), 'simulation/train/annotations.yaml')
        download_file(os.path.join(DATA_FOLDER, 'simulation', 'test', 'annotations.yaml'), 'simulation/test/annotations.yaml')
