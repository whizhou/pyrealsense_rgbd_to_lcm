"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

import bot_core

class image_t(object):
    """
    this was copied from libbot, bot_core lcmtypes package
    Describes an image
    
    This type should be kept compatible with camlcm.image_t, as defined
    in camunits-extra
    """

    __slots__ = ["utime", "width", "height", "row_stride", "pixelformat", "size", "data", "nmetadata", "metadata"]

    __typenames__ = ["int64_t", "int32_t", "int32_t", "int32_t", "int32_t", "int32_t", "byte", "int32_t", "bot_core.image_metadata_t"]

    __dimensions__ = [None, None, None, None, None, None, ["size"], None, ["nmetadata"]]

    PIXEL_FORMAT_UYVY = 1498831189
    """
    these values are to be kept in sync with the CamPixelFormat enumeration
    in the Camunits library.  See camunits/pixels.h in Camunits
    """
    PIXEL_FORMAT_YUYV = 1448695129
    PIXEL_FORMAT_IYU1 = 827677001
    PIXEL_FORMAT_IYU2 = 844454217
    PIXEL_FORMAT_YUV420 = 842093913
    PIXEL_FORMAT_YUV411P = 1345401140
    PIXEL_FORMAT_I420 = 808596553
    PIXEL_FORMAT_NV12 = 842094158
    PIXEL_FORMAT_GRAY = 1497715271
    PIXEL_FORMAT_RGB = 859981650
    PIXEL_FORMAT_BGR = 861030210
    PIXEL_FORMAT_RGBA = 876758866
    PIXEL_FORMAT_BGRA = 877807426
    PIXEL_FORMAT_BAYER_BGGR = 825770306
    PIXEL_FORMAT_BAYER_GBRG = 844650584
    PIXEL_FORMAT_BAYER_GRBG = 861427800
    PIXEL_FORMAT_BAYER_RGGB = 878205016
    PIXEL_FORMAT_BE_BAYER16_BGGR = 826360386
    PIXEL_FORMAT_BE_BAYER16_GBRG = 843137602
    PIXEL_FORMAT_BE_BAYER16_GRBG = 859914818
    PIXEL_FORMAT_BE_BAYER16_RGGB = 876692034
    PIXEL_FORMAT_LE_BAYER16_BGGR = 826360396
    PIXEL_FORMAT_LE_BAYER16_GBRG = 843137612
    PIXEL_FORMAT_LE_BAYER16_GRBG = 859914828
    PIXEL_FORMAT_LE_BAYER16_RGGB = 876692044
    PIXEL_FORMAT_MJPEG = 1196444237
    PIXEL_FORMAT_BE_GRAY16 = 357
    PIXEL_FORMAT_LE_GRAY16 = 909199180
    PIXEL_FORMAT_BE_RGB16 = 358
    PIXEL_FORMAT_LE_RGB16 = 1279412050
    PIXEL_FORMAT_BE_SIGNED_GRAY16 = 359
    PIXEL_FORMAT_BE_SIGNED_RGB16 = 360
    PIXEL_FORMAT_FLOAT_GRAY32 = 842221382
    PIXEL_FORMAT_INVALID = -2
    PIXEL_FORMAT_ANY = -1

    def __init__(self):
        self.utime = 0
        """
        microseconds since the epoch
        LCM Type: int64_t
        """

        self.width = 0
        """ LCM Type: int32_t """
        self.height = 0
        """ LCM Type: int32_t """
        self.row_stride = 0
        """ LCM Type: int32_t """
        self.pixelformat = 0
        """ LCM Type: int32_t """
        self.size = 0
        """ LCM Type: int32_t """
        self.data = b""
        """ LCM Type: byte[size] """
        self.nmetadata = 0
        """
        metadata dictionary
        LCM Type: int32_t
        """

        self.metadata = []
        """ LCM Type: bot_core.image_metadata_t[nmetadata] """

    def encode(self):
        buf = BytesIO()
        buf.write(image_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qiiiii", self.utime, self.width, self.height, self.row_stride, self.pixelformat, self.size))
        buf.write(bytearray(self.data[:self.size]))
        buf.write(struct.pack(">i", self.nmetadata))
        for i0 in range(self.nmetadata):
            assert self.metadata[i0]._get_packed_fingerprint() == bot_core.image_metadata_t._get_packed_fingerprint()
            self.metadata[i0]._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != image_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return image_t._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = image_t()
        self.utime, self.width, self.height, self.row_stride, self.pixelformat, self.size = struct.unpack(">qiiiii", buf.read(28))
        self.data = buf.read(self.size)
        self.nmetadata = struct.unpack(">i", buf.read(4))[0]
        self.metadata = []
        for i0 in range(self.nmetadata):
            self.metadata.append(bot_core.image_metadata_t._decode_one(buf))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if image_t in parents: return 0
        newparents = parents + [image_t]
        tmphash = (0x6fee6cb20472ff6a+ bot_core.image_metadata_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if image_t._packed_fingerprint is None:
            image_t._packed_fingerprint = struct.pack(">Q", image_t._get_hash_recursive([]))
        return image_t._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", image_t._get_packed_fingerprint())[0]

