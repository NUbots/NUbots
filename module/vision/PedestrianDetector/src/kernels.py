#The MIT License (MIT)
#
#Copyright (c) 2015 Jason Newton <nevion@gmail.com>
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

from kernel_common import *
import enum

class Demosaic(object):
    class Pattern(enum.IntEnum):
        RGGB = 0
        GRBG = 1
        GBRG = 2
        BGGR = 3

    def __init__(self, img_dtype, rgb_pixel_base_dtype = None, output_channels=3, debug=False):
        self.img_dtype = img_dtype
        if rgb_pixel_base_dtype is None:
            rgb_pixel_base_dtype = img_dtype
        self.rgb_pixel_base_dtype = rgb_pixel_base_dtype
        self.output_channels = output_channels
        self.debug = debug

        self.program = None
        self.kernel = None
        self.TILE_ROWS = 5
        self.TILE_COLS = 32

    def compile(self):
        PixelT = type_mapper(self.img_dtype)
        RGBPixelBaseT = type_mapper(self.rgb_pixel_base_dtype)

        KERNEL_FLAGS = '-D PIXELT={PixelT} -D RGBPIXELBASET={RGBPixelBaseT} -D OUTPUT_CHANNELS={output_channels} -D TILE_ROWS={tile_rows} -D TILE_COLS={tile_cols} -D IMAGE_MAD_INDEXING' \
             .format(PixelT=PixelT, RGBPixelBaseT=RGBPixelBaseT, output_channels=self.output_channels, tile_rows=self.TILE_ROWS, tile_cols=self.TILE_COLS)
        CL_SOURCE = None
        with open(os.path.join(base_path, 'kernels.cl'), 'r') as f:
            CL_SOURCE = f.read()
        CL_FLAGS = "-I %s -cl-std=CL1.2 %s" %(common_lib_path, KERNEL_FLAGS)
        CL_FLAGS = cl_opt_decorate(self, CL_FLAGS)
        print('%r compile flags: %s'%(self.__class__.__name__, CL_FLAGS))
        self.program = cl.Program(ctx, CL_SOURCE).build(options=CL_FLAGS)

        self._malvar_he_cutler_demosaic = self.program.malvar_he_cutler_demosaic

    def make_output_buffer(self, queue, image):
        return clarray.empty(queue, image.shape + (self.output_channels,), dtype = self.img_dtype)

    def __call__(self, queue, image, pattern, dst_img = None, wait_for = None):
        tile_dims = self.TILE_COLS, self.TILE_ROWS
        ldims = tile_dims
        rows, cols = int(image.shape[0]), int(image.shape[1])
        if dst_img is None:
            dst_img = self.make_output_buffer(queue, image)
        r_blocks, c_blocks = divUp(rows, tile_dims[1]), divUp(cols, tile_dims[0])
        gdims = (c_blocks * ldims[0], r_blocks * ldims[1])
        event = self._malvar_he_cutler_demosaic(queue,
            gdims, ldims,
            uint32(rows), uint32(cols),
            image.data, uint32(image.strides[0]),
            dst_img.data, uint32(dst_img.strides[0]),
            np.int32(pattern),
            wait_for = wait_for
        )
        return event, dst_img
