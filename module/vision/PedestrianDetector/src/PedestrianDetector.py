#!/usr/bin/env python3

from nuclear import Reactor, on, Trigger, Single, With, Every
from message.input import Image
from message.vision import Obstacle
import tensorflow as tf
import numpy as np
import yaml
import datetime

@Reactor
class PedestrianDetector(object):

    def __init__(self):
        # Constructor for PedestrianDetector
        self.config = yaml.load(file('config/PedestrianDetector.yaml', 'r'))

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.config['model']['checkpoint_file'], 'rb') as fid:
                serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')

    # @on(Configuration('PedestrianDetector.yaml'))
    # def PedestrianDetector_configfuration(self, config):
    #     # Use configuration here from file PedestrianDetector.yaml

    @on(Trigger(Image))
    def example_callback(self, image):
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

                # Convert image to numpy array
                image_np = np.array(image.data).reshape((image.dimensions[1], image.dimensions[0], 3)).astype(np.uint8)

                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)

                # Actual detection.
                (boxes, scores, classes, num) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})

                msg = []
                for box in boxes:
                    obj = Obstacle()
                    obj.visObject.timestamp = datetime.datetime.now()
                    obj.visObject.camera_id = image.camera_id
                    obj.shape = np.squeeze(box)
                    obj.team = Obstacle.Team.UNKNOWN

                    msg.append(obj)


                self.emit(msg)
