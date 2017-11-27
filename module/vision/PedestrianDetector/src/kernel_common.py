import pyopencl as cl
import pyopencl.array as clarray
from time import time
import numpy as np
import os
from numpy import uint32, int32
from kernel_util import *

common_lib_path = 'clcommons/include'
base_path = os.path.dirname(os.path.realpath(__file__))

#ctx = cl.create_some_context()
platform = cl.get_platforms()[0]
devices = [device for device in platform.get_devices() if device.type == cl.device_type.GPU]
device = [devices[0]]
queue_properties = cl.command_queue_properties.PROFILING_ENABLE | cl.command_queue_properties.OUT_OF_ORDER_EXEC_MODE_ENABLE
ctx = cl.Context(devices)
queues = [cl.CommandQueue(ctx, device, properties=queue_properties) for device in devices]
#multicontext
#ctxs = [cl.Context(device) for device in devices]
#queues = [cl.CommandQueue(ctx, device, properties=queue_properties) for ctx, device in zip(ctxs, devices)]
queue = queues[0]
compute_units = max([device.max_compute_units for device in devices])
device_wg_size = min([wavefront_wg_size(device) for device in devices])
default_max_wg_size = max([device.max_work_group_size for device in devices])
default_wg_size = device_wg_size
is_amd_platform = all([is_device_amd(device) for device in devices])
is_gpu_platform = all([device.type == cl.device_type.GPU for device in devices])
is_amd_gpu_platform = all([(is_device_amd(device) and device.type == cl.device_type.GPU) for device in devices])
is_nvidia_platform = all([is_device_nvidia(device) for device in devices])
is_intel_platform = all([(is_device_intel(device) and device.type == cl.device_type.GPU) for device in devices])

def cl_opt_decorate(kop, CL_FLAGS, max_wg_size_used = None, max_wg_size = None):
    if is_amd_gpu_platform:
        CL_FLAGS2 = '-D AMD_GPU_ARCH -D DEVICE_WAVEFRONT_SIZE={wavefront_size} '.format(wavefront_size=device_wg_size)
        if max_wg_size_used is not None and np.prod(max_wg_size_used, dtype=np.uint32) <= device_wg_size:
            CL_FLAGS2 = CL_FLAGS2 + '-D PROMISE_WG_IS_WAVEFRONT '
        CL_FLAGS = CL_FLAGS2 + CL_FLAGS
    elif is_nvidia_platform:
        CL_FLAGS2 = '-D NVIDIA_ARCH -D DEVICE_WAVEFRONT_SIZE={wavefront_size} '.format(wavefront_size=device_wg_size)
        #if max_wg_size_used is not None and np.prod(max_wg_size_used, dtype=np.uint32) <= device_wg_size:
        #    CL_FLAGS2 = CL_FLAGS2 + '-D PROMISE_WG_IS_WAVEFRONT '
        #causes segfault in NvCliCompileBitcode - seems like internal compiler error
        CL_FLAGS = CL_FLAGS2 + CL_FLAGS
    elif is_intel_platform:
        CL_FLAGS2 = '-D NVIDIA_ARCH -D DEVICE_WAVEFRONT_SIZE={wavefront_size} '.format(wavefront_size=device_wg_size)
        if max_wg_size_used is not None and np.prod(max_wg_size_used, dtype=np.uint32) <= device_wg_size:
            CL_FLAGS2 = CL_FLAGS2 + '-D PROMISE_WG_IS_WAVEFRONT '
        CL_FLAGS = CL_FLAGS2 + CL_FLAGS

    if max_wg_size is None:
        max_wg_size = default_max_wg_size
    CL_FLAGS = '-D WG_SIZE_MAX=%d '%(max_wg_size,) + CL_FLAGS
    if is_gpu_platform:
        CL_FLAGS = '-D GPU_ARCH ' + CL_FLAGS

    if kop.debug == 2:
        CL_FLAGS = '-D DEBUG -g -cl-opt-disable '+CL_FLAGS
    elif kop.debug:
        CL_FLAGS = '-D DEBUG '+CL_FLAGS
    return CL_FLAGS
