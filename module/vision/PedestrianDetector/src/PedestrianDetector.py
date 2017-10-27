#!/usr/bin/env python3

from nuclear import Reactor, on, Trigger, Single, With, Every
from message.input import Image
from message.vision import Obstacle
import numpy as np
import tensorflow as tf
import yaml
import datetime
import time

#from kernels import *

#patterns = {0x42474752: 0, 0x47425247: 1, 0x47524247: 2, 0x52474742: 3}

@Reactor
class PedestrianDetector(object):

    def __init__(self):
        # Constructor for PedestrianDetector
        self.config = yaml.load(open('config/PedestrianDetector.yaml', 'r'))

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.config['model']['checkpoint_file'], 'rb') as fid:
                serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')

        # Setup OpenCL kernel for demosaicing
        #self.demosaic = Demosaic(img.dtype, output_channels = output_channels, debug=False)
        #self.demosaic.compile()

        # TODO: Find a nice way to get this information at this point in time
        #self.img.shape = (1024, 1280)
        #self.channels = 3

        #self.cl_src_img = clarray.empty(queue, img.shape, img.dtype)
        #self.cl_dst_img = clarray.empty(queue, img.shape + (output_channels,), img.dtype)
        #self.dst_img = np.empty(img.shape + (output_channels,), img.dtype)

        #print('PedestrianDetector: Successfully compiled OpenCL kernel.')


    # @on(Configuration('PedestrianDetector.yaml'))
    # def PedestrianDetector_configfuration(self, config):
    #     # Use configuration here from file PedestrianDetector.yaml

    @on(Trigger(Image), Single())
    def run_detection(self, image):
        # Convert image to numpy array
        image_np = np.zeros((image.dimensions[1], image.dimensions[0], 3), dtype = np.uint8)
        img = np.array(image.data).reshape((image.dimensions[1], image.dimensions[0], 1)).astype(np.uint8)
        image_np[:,:,0] = img[:,:,0].copy()
        image_np[:,:,1] = img[:,:,0].copy()
        image_np[:,:,2] = img[:,:,0].copy()

        # Demosaic image
        #demosaic_start = time.time()
        #event = cl.enqueue_copy(queue, self.cl_src_img.data, image_np, wait_for = None)
        #event, self.cl_dst_img = demosaic(queue, self.cl_src_img, patterns[image.format], self.cl_dst_img, wait_for = event)
        #event = cl.enqueue_copy(queue, self.dst_img, self.cl_dst_img.data, wait_for = event)
        #demosaic_end = time.time()
        #print('PedestrianDetector::run_detection: Demosaicing time: {0}'.format(demosaic_end - demosaic_start))

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis = 0)

                # Actual detection.
                detection_start = time.time()
                (boxes, scores, classes, num) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})
                detection_end = time.time()
                print('PedestrianDetector::run_detection: Detection time: {0}'.format(detection_end - detection_start))

                msg = []
                for box in np.squeeze(boxes):
                    obj = Obstacle()
                    obj.visObject.timestamp = datetime.datetime.now()
                    obj.visObject.camera_id = image.camera_id
                    ymin, xmin, ymax, xmax = box.tolist()
                    obj.shape.points = [[int(xmin * image.dimensions[0]), int(ymin * image.dimensions[1])],
                                        [int(xmax * image.dimensions[0]), int(ymin * image.dimensions[1])],
                                        [int(xmax * image.dimensions[0]), int(ymax * image.dimensions[1])],
                                        [int(xmin * image.dimensions[0]), int(ymax * image.dimensions[1])]]
                    obj.team = Obstacle.Team.UNKNOWN_TEAM

                    msg.append(obj)

                self.emit(msg)

