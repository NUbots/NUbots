INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME OpenCV
                       HEADER opencv2/core.hpp
)

FIND_LIBRARY(
    opencv_calib3d_LIBRARY
    opencv_calib3d
    DOC "The opencv (opencv_calib3d) library"
)
FIND_LIBRARY(
    opencv_imgcodecs_LIBRARY
    opencv_imgcodecs
    DOC "The opencv (opencv_imgcodecs) library"
)
FIND_LIBRARY(
    opencv_stitching_LIBRARY
    opencv_stitching
    DOC "The opencv (opencv_stitching) library"
)
FIND_LIBRARY(
    opencv_core_LIBRARY
    opencv_core
    DOC "The opencv (opencv_core) library"
)
FIND_LIBRARY(
    opencv_imgproc_LIBRARY
    opencv_imgproc
    DOC "The opencv (opencv_imgproc) library"
)
FIND_LIBRARY(
    opencv_superres_LIBRARY
    opencv_superres
    DOC "The opencv (opencv_superres) library"
)
FIND_LIBRARY(
    opencv_dnn_LIBRARY
    opencv_dnn
    DOC "The opencv (opencv_dnn) library"
)
FIND_LIBRARY(
    opencv_ml_LIBRARY
    opencv_ml
    DOC "The opencv (opencv_ml) library"
)
FIND_LIBRARY(
    opencv_videoio_LIBRARY
    opencv_videoio
    DOC "The opencv (opencv_videoio) library"
)
FIND_LIBRARY(
    opencv_features2d_LIBRARY
    opencv_features2d
    DOC "The opencv (opencv_features2d) library"
)
FIND_LIBRARY(
    opencv_objdetect_LIBRARY
    opencv_objdetect
    DOC "The opencv (opencv_objdetect) library"
)
FIND_LIBRARY(
    opencv_video_LIBRARY
    opencv_video
    DOC "The opencv (opencv_video) library"
)
FIND_LIBRARY(
    opencv_flann_LIBRARY
    opencv_flann
    DOC "The opencv (opencv_flann) library"
)
FIND_LIBRARY(
    opencv_photo_LIBRARY
    opencv_photo
    DOC "The opencv (opencv_photo) library"
)
FIND_LIBRARY(
    opencv_videostab_LIBRARY
    opencv_videostab
    DOC "The opencv (opencv_videostab) library"
)
FIND_LIBRARY(
    opencv_highgui_LIBRARY
    opencv_highgui
    DOC "The opencv (opencv_highgui) library"
)
FIND_LIBRARY(
    opencv_shape_LIBRARY
    opencv_shape
    DOC "The opencv (opencv_shape) library"
)

SET(OpenCV_LIBRARIES
    ${opencv_calib3d_LIBRARY}
    ${opencv_imgcodecs_LIBRARY}
    ${opencv_stitching_LIBRARY}
    ${opencv_core_LIBRARY}
    ${opencv_imgproc_LIBRARY}
    ${opencv_superres_LIBRARY}
    ${opencv_dnn_LIBRARY}
    ${opencv_ml_LIBRARY}
    ${opencv_videoio_LIBRARY}
    ${opencv_features2d_LIBRARY}
    ${opencv_objdetect_LIBRARY}
    ${opencv_video_LIBRARY}
    ${opencv_flann_LIBRARY}
    ${opencv_photo_LIBRARY}
    ${opencv_videostab_LIBRARY}
    ${opencv_highgui_LIBRARY}
    ${opencv_shape_LIBRARY}
)

MARK_AS_ADVANCED(OpenCV LIBRARIES
    opencv_calib3d_LIBRARY
    opencv_imgcodecs_LIBRARY
    opencv_stitching_LIBRARY
    opencv_core_LIBRARY
    opencv_imgproc_LIBRARY
    opencv_superres_LIBRARY
    opencv_dnn_LIBRARY
    opencv_ml_LIBRARY
    opencv_videoio_LIBRARY
    opencv_features2d_LIBRARY
    opencv_objdetect_LIBRARY
    opencv_video_LIBRARY
    opencv_flann_LIBRARY
    opencv_photo_LIBRARY
    opencv_videostab_LIBRARY
    opencv_highgui_LIBRARY
    opencv_shape_LIBRARY
)
