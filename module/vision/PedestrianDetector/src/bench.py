#!/usr/bin/env python

from kernels import *
from scipy.misc import imread, imsave
import time
import sys
import enum

debug = False
filename = 'image.png'
output_filename = None
iters = 500
pattern = None
output_channels = 4

if __name__ == '__main__':
    import argparse, sys
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', default=filename)
    parser.add_argument('output', default=output_filename)
    parser.add_argument('--debug', action='store_true', default=debug)
    parser.add_argument('-i','--iterations', type=int, default=iters)
    parser.add_argument('--pattern','-p', type=str, required=True)
    parser.add_argument('--channels','-c', type=int, default = output_channels)
    args = parser.parse_args()
    filename = args.filename
    output_filename = args.output
    debug = args.debug
    iters = args.iterations
    pattern = getattr(Demosaic.Pattern, args.pattern)
    output_channels = args.channels

img = imread(filename)
if len(img.shape) > 2:
    img = np.sum(img, axis=2, dtype=img.dtype)

demosaic = Demosaic(img.dtype, output_channels = output_channels, debug=debug)
demosaic.compile()


cl_src_img = clarray.empty(queue, img.shape, img.dtype)
cl_dst_img = clarray.empty(queue, img.shape+(output_channels,), img.dtype)
dst_img = np.empty(img.shape+(output_channels,), img.dtype)

def upload(wait_for = None):
    event = cl.enqueue_copy(queue, cl_src_img.data, img, wait_for=wait_for)
    return event
def download(wait_for = None):
    event = cl.enqueue_copy(queue, dst_img, cl_dst_img.data, wait_for = wait_for)
    return event

def core_loop(wait_for = None):
    global cl_dst_img
    event, cl_dst_img = demosaic(queue, cl_src_img, pattern, cl_dst_img)
    return event

def full_loop():
    event = upload()
    event = core_loop(wait_for = [event])
    download(wait_for = [event])

print('compiled')
upload().wait()
print('uploaded')

times = np.zeros((iters, 2), np.double)
loop_start = time.time()
for x in range(iters):
    start = time.time()
    core_loop().wait()
    end = time.time()
    times[x, :] = start, end
loop_end = time.time()
loop_total = loop_end - loop_start
loop_avg = (loop_total / iters)*1e3
#import timeit
#iters = 100*15
#print timeit.timeit(lambda: core_loop().wait(), number=iters)*1e3/iters
timings = np.squeeze(np.diff(times, axis=1))*1e3
print(timings)
print('total: %r loop avg: %r best: %r iterations: %d std: %r'%(loop_total, loop_avg, timings.min(), iters, np.std(timings)))

print('download')
download().wait()

if output_filename is not None:
    imsave(output_filename, dst_img)
