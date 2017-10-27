import pyopencl as cl
import pyopencl.array as clarray
import numpy as np

def divUp(x, by):
    return ((x + by - 1)//by)

def divUpSafe(x, bpy):
    k = x // bpy
    if k * bpy != x:
        k = k + 1
    return k

def logDown(x, base):
    power = 1
    xbak = x
    x /= base
    i = 0
    while x > 0:
        power *= base
        x /= base
        i+=1
    return i

def logUp(x, base):
    if x == 0 or base == 0:
        return 0
    x2 = 1
    k = 0
    while x2 < x:
        x2 *= base
        k += 1
    return k

def roundUpToMultiple(x, multiple):
    return divUp(x, multiple) * multiple

def prefix_sum(counts, inclusive_or_exclusive = 0, dtype=None):
    counts = np.asarray(counts, dtype=dtype)
    sums = np.cumsum(counts, dtype=dtype)
    if inclusive_or_exclusive == 1:
        _sums = np.empty_like(sums)
        _sums[0] = 0
        _sums[1:] = sums[0:-1]
        sums = _sums
    return sums

def inclusive_prefix_sum(counts, dtype=None):
    return prefix_sum(counts, 0, dtype=dtype)

def exclusive_prefix_sum(counts, dtype=None):
    return prefix_sum(counts, 1, dtype=dtype)

def prefix_product(counts, inclusive_or_exclusive = 0, dtype=None):
    counts = np.asarray(counts, dtype=dtype)
    products = np.cumprod(counts, dtype=dtype)
    if inclusive_or_exclusive == 1:
        _products = np.empty_like(products)
        _products[0] = 1
        _products[1:] = products[0:-1]
        products = _products
    return products
def inclusive_prefix_product(counts, dtype=None):
    return prefix_product(counts, 0, dtype=dtype)

def exclusive_prefix_product(counts, dtype=None):
    return prefix_product(counts, 1, dtype=dtype)

def type_mapper(x):
    if x == np.int32:
        return 'int'
    if x == np.uint32:
        return 'uint'
    if x == np.int8:
        return 'char'
    if x == np.uint8:
        return 'uchar'
    if x == np.int16:
        return 'short'
    if x == np.uint16:
        return 'ushort'
    if x == np.int64:
        return 'long'
    if x == np.uint64:
        return 'ulong'
    if x == np.float32:
        return 'float'
    if x == np.float64:
        return 'double'
    return None

def dtype_of(x):
    if type(x) is type:
        x = np.dtype(x)
    return x

def is_platform_amd(platform):
    return platform.vendor == 'Advanced Micro Devices, Inc.'

def is_platform_nvidia(platform):
    return platform.vendor == 'NVIDIA Corporation'

def is_device_amd(device):
    return device.vendor == 'Advanced Micro Devices, Inc.'

def is_device_nvidia(device):
    return device.vendor == 'NVIDIA Corporation'

def is_device_intel(device):
    return device.vendor == 'GenuineIntel' or device.vendor == 'Intel(R) Corporation'

def wavefront_wg_size(device):
    if is_device_amd(device) and device.type == cl.device_type.GPU:
        return device.wavefront_width_amd
    elif is_device_nvidia(device) and device.type == cl.device_type.GPU:
        return device.warp_size_nv
    else:
        return device.max_work_group_size

def device_workgroups(device):
    if is_device_amd(device):
        return device.simd_per_compute_unit_amd * device.max_compute_units
    else:
        return device.max_compute_units
