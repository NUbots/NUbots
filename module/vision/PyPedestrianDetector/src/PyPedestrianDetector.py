#!/usr/bin/env python3

from nuclear import Reactor, on, Trigger, Single, With, Every
from message.vision import ReprojectedImage, Obstacle, BakedImage
from message.motion import HeadCommand, WaveRightCommand
from message.input import Sensors
import numpy as np
import tensorflow as tf
import yaml
import datetime
import time
import sys
import collections
from skimage.draw import polygon, set_color
import PIL.Image as Image
import PIL.ImageColor as ImageColor
import PIL.ImageDraw as ImageDraw
import PIL.ImageFont as ImageFont

STANDARD_COLORS = [
    'AliceBlue', 'Chartreuse', 'Aqua', 'Aquamarine', 'Azure', 'Beige', 'Bisque', 'BlanchedAlmond', 'BlueViolet',
    'BurlyWood', 'CadetBlue', 'AntiqueWhite', 'Chocolate', 'Coral', 'CornflowerBlue', 'Cornsilk', 'Crimson', 'Cyan',
    'DarkCyan', 'DarkGoldenRod', 'DarkGrey', 'DarkKhaki', 'DarkOrange', 'DarkOrchid', 'DarkSalmon', 'DarkSeaGreen',
    'DarkTurquoise', 'DarkViolet', 'DeepPink', 'DeepSkyBlue', 'DodgerBlue', 'FireBrick', 'FloralWhite', 'ForestGreen',
    'Fuchsia', 'Gainsboro', 'GhostWhite', 'Gold', 'GoldenRod', 'Salmon', 'Tan', 'HoneyDew', 'HotPink', 'IndianRed',
    'Ivory', 'Khaki', 'Lavender', 'LavenderBlush', 'LawnGreen', 'LemonChiffon', 'LightBlue', 'LightCoral', 'LightCyan',
    'LightGoldenRodYellow', 'LightGray', 'LightGrey', 'LightGreen', 'LightPink', 'LightSalmon', 'LightSeaGreen',
    'LightSkyBlue', 'LightSlateGray', 'LightSlateGrey', 'LightSteelBlue', 'LightYellow', 'Lime', 'LimeGreen', 'Linen',
    'Magenta', 'MediumAquaMarine', 'MediumOrchid', 'MediumPurple', 'MediumSeaGreen', 'MediumSlateBlue',
    'MediumSpringGreen', 'MediumTurquoise', 'MediumVioletRed', 'MintCream', 'MistyRose', 'Moccasin', 'NavajoWhite',
    'OldLace', 'Olive', 'OliveDrab', 'Orange', 'OrangeRed', 'Orchid', 'PaleGoldenRod', 'PaleGreen', 'PaleTurquoise',
    'PaleVioletRed', 'PapayaWhip', 'PeachPuff', 'Peru', 'Pink', 'Plum', 'PowderBlue', 'Purple', 'Red', 'RosyBrown',
    'RoyalBlue', 'SaddleBrown', 'Green', 'SandyBrown', 'SeaGreen', 'SeaShell', 'Sienna', 'Silver', 'SkyBlue',
    'SlateBlue', 'SlateGray', 'SlateGrey', 'Snow', 'SpringGreen', 'SteelBlue', 'GreenYellow', 'Teal', 'Thistle',
    'Tomato', 'Turquoise', 'Violet', 'Wheat', 'White', 'WhiteSmoke', 'Yellow', 'YellowGreen'
]


@Reactor
class PyPedestrianDetector(object):

    def __init__(self):
        # Constructor for PyPedestrianDetector
        self.config = yaml.load(open('config/PyPedestrianDetector.yaml', 'r'))
        self.dump_images = self.config['dump']['images']
        self.dump_stats = self.config['dump']['stats']
        self.stats_file = self.config['dump']['stats_file']

        # Load labels map
        self.category_index = self.load_labelmap(self.config['model']['label_file'])

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.config['model']['graph_file'], 'rb') as fid:
                serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        self.session = tf.Session(graph=self.detection_graph)

        self.avg_fp_ms = 0
        self.avg_count = 0

    def __del__(self):
        self.session.close()

    # @on(Configuration('PyPedestrianDetector.yaml'))
    # def PedestrianDetector_configfuration(self, config):
    #     # Use configuration here from file PyPedestrianDetector.yaml

    @on(Trigger(ReprojectedImage), With(Sensors), Single())
    def run_detection(self, image, sensors):
        if image.camera_id > 1:
            # Convert image to numpy array
            input_img = np.array(image.data).reshape((image.dimensions[1], image.dimensions[0],
                                                      3)).astype(np.uint8).copy()

            with self.detection_graph.as_default():
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(input_img, axis=0)

                # Actual detection.
                detection_start = time.time()
                (boxes, scores, classes, num) = self.session.run([
                    self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections
                ],
                                                                 feed_dict={self.image_tensor: image_np_expanded})
                detection_end = time.time()

                # Visualization of the results of a detection.
                self.visualize_boxes_and_labels_on_image_array(
                    input_img,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    self.category_index,
                    use_normalized_coordinates=True,
                    line_thickness=8
                )

                img = BakedImage()
                img.format = image.format
                img.dimensions = image.dimensions.copy()
                img.data = input_img.flatten().tolist().copy()
                img.camera_id = image.camera_id
                img.serial_number = "PyPedestrianDetector"
                img.timestamp = datetime.datetime.now()

                if (self.dump_images):
                    with open('pedestrian_detector-{}.ppm'.format(self.avg_count), 'wb') as f:
                        f.write('P6\n{} {}\n255\n'.format(image.dimensions[0], image.dimensions[1]).encode())
                        f.write(bytes(img.data))

                self.avg_fp_ms += detection_end - detection_start
                self.avg_count += 1

                if self.dump_stats:
                    with open(self.stats_file, 'a') as f:
                        f.write(
                            '{},{},{}\n'.format(
                                np.squeeze(boxes).shape, detection_end - detection_start,
                                self.avg_fp_ms / self.avg_count
                            )
                        )

                if ((self.avg_count - 1) % 10) == 0:
                    print(
                        'PyPedestrianDetector::run_detection: Detection time: {0:.4f} s (avg: {1:.4f} s)'.format(
                            detection_end - detection_start, self.avg_fp_ms / self.avg_count
                        )
                    )

                index = -1
                s_classes = np.squeeze(classes)
                max_width_score = 0.0
                max_score = 0.0
                for i, s in enumerate(np.squeeze(scores)):
                    if self.category_index[int(s_classes[i])] == 'person':
                        ymin, xmin, ymax, xmax = np.squeeze(boxes)[i]
                        if s * (xmax - xmin) > max_width_score:
                            max_score = s
                            index = i
                            max_width_score = s * (xmax - xmin)

                if max_score >= 0.5 and i > -1:
                    # Figure out camera parameters
                    imageCenter = (image.dimensions - 1) * 0.5
                    camFocalLengthPixels = imageCenter[0] / np.tan(image.FOV * np.pi / 360.0)

                    # Approximate pedestrian head position as vertically in the middle and about 10% from the top
                    # of the box
                    ymin, xmin, ymax, xmax = np.squeeze(boxes)[index]
                    width = (xmax - xmin) * image.dimensions[0]
                    height = (ymax - ymin) * image.dimensions[1]
                    x = (xmin * image.dimensions[0]) + width * 0.5
                    y = (ymin * image.dimensions[1]) + height * 0.1

                    # Form unit vector from camera to head position
                    screen = (imageCenter[0] - x, imageCenter[1] - y)
                    vec = (camFocalLengthPixels, screen[0], screen[1])
                    vec /= np.linalg.norm(vec)

                    # Get camera to torso transform
                    Htc = sensors.forwardKinematics[19]
                    body_vec = Htc.dot((vec[0], vec[1], vec[2], 0))
                    screen_angular = (np.arctan2(body_vec[1], body_vec[0]), np.arctan2(body_vec[2], body_vec[0]))

                    # Now create our head movement command
                    head_command = HeadCommand()
                    head_command.yaw = screen_angular[0]
                    head_command.pitch = screen_angular[1]
                    head_command.robotSpace = True
                    self.emit(head_command)

                    if width > 0.25 * image.dimensions[0]:
                        cmd = WaveRightCommand()
                        cmd.pre_delay = 0
                        cmd.post_delay = 10
                        self.emit(cmd)

                self.emit(img)

    def load_labelmap(self, path):
        category_index = {}

        with open(path, 'r') as label_file:
            labels = yaml.load(label_file)

            for label in labels['labels']:
                if int(label['id']) < 1:
                    raise ValueError('Label map ids should be >= 1.')

                category_index[int(label['id'])] = label['display_name']

        return category_index

    def visualize_boxes_and_labels_on_image_array(
            self,
            image,
            boxes,
            classes,
            scores,
            category_index,
            instance_masks=None,
            keypoints=None,
            use_normalized_coordinates=False,
            max_boxes_to_draw=20,
            min_score_thresh=.5,
            agnostic_mode=False,
            line_thickness=4
    ):
        """Overlay labeled boxes on an image with formatted scores and label names.
        This function groups boxes that correspond to the same location
        and creates a display string for each detection and overlays these
        on the image. Note that this function modifies the image in place, and returns
        that same image.
        Args:
          image: uint8 numpy array with shape (img_height, img_width, 3)
          boxes: a numpy array of shape [N, 4]
          classes: a numpy array of shape [N]. Note that class indices are 1-based,
            and match the keys in the label map.
          scores: a numpy array of shape [N] or None.  If scores=None, then
            this function assumes that the boxes to be plotted are groundtruth
            boxes and plot all boxes as black with no classes or scores.
          category_index: a dict containing category dictionaries (each holding
            category index `id` and category name `name`) keyed by category indices.
          instance_masks: a numpy array of shape [N, image_height, image_width], can
            be None
          keypoints: a numpy array of shape [N, num_keypoints, 2], can
            be None
          use_normalized_coordinates: whether boxes is to be interpreted as
            normalized coordinates or not.
          max_boxes_to_draw: maximum number of boxes to visualize.  If None, draw
            all boxes.
          min_score_thresh: minimum score threshold for a box to be visualized
          agnostic_mode: boolean (default: False) controlling whether to evaluate in
            class-agnostic mode or not.  This mode will display scores but ignore
            classes.
          line_thickness: integer (default: 4) controlling line width of the boxes.
        Returns:
          uint8 numpy array with shape (img_height, img_width, 3) with overlaid boxes.
        """
        # Create a display string (and color) for every box location, group any boxes
        # that correspond to the same location.
        box_to_display_str_map = collections.defaultdict(list)
        box_to_color_map = collections.defaultdict(str)
        box_to_instance_masks_map = {}
        box_to_keypoints_map = collections.defaultdict(list)

        if not max_boxes_to_draw:
            max_boxes_to_draw = boxes.shape[0]

        for i in range(min(max_boxes_to_draw, boxes.shape[0])):
            if scores is None or scores[i] > min_score_thresh:
                box = tuple(boxes[i].tolist())

                if instance_masks is not None:
                    box_to_instance_masks_map[box] = instance_masks[i]

                if keypoints is not None:
                    box_to_keypoints_map[box].extend(keypoints[i])

                if scores is None:
                    box_to_color_map[box] = 'black'

                else:
                    if not agnostic_mode:
                        if classes[i] in category_index.keys():
                            class_name = category_index[int(classes[i])]

                        else:
                            class_name = 'N/A'

                        display_str = '{}: {}%'.format(class_name, int(100 * scores[i]))

                    else:
                        display_str = 'score: {}%'.format(int(100 * scores[i]))

                box_to_display_str_map[box].append(display_str)

                if agnostic_mode:
                    box_to_color_map[box] = 'DarkOrange'

                else:
                    box_to_color_map[box] = STANDARD_COLORS[classes[i] % len(STANDARD_COLORS)]

        # Draw all boxes onto image.
        for box, color in box_to_color_map.items():
            ymin, xmin, ymax, xmax = box

            if instance_masks is not None:
                self.draw_mask_on_image_array(image, box_to_instance_masks_map[box], color=color)

            self.draw_bounding_box_on_image_array(
                image,
                ymin,
                xmin,
                ymax,
                xmax,
                color=color,
                thickness=line_thickness,
                display_str_list=box_to_display_str_map[box],
                use_normalized_coordinates=use_normalized_coordinates
            )

            if keypoints is not None:
                self.draw_keypoints_on_image_array(
                    image,
                    box_to_keypoints_map[box],
                    color=color,
                    radius=line_thickness / 2,
                    use_normalized_coordinates=use_normalized_coordinates
                )

        return image

    def draw_bounding_box_on_image_array(
            self,
            image,
            ymin,
            xmin,
            ymax,
            xmax,
            color='red',
            thickness=4,
            display_str_list=(),
            use_normalized_coordinates=True
    ):
        """Adds a bounding box to an image (numpy array).
        Args:
          image: a numpy array with shape [height, width, 3].
          ymin: ymin of bounding box in normalized coordinates (same below).
          xmin: xmin of bounding box.
          ymax: ymax of bounding box.
          xmax: xmax of bounding box.
          color: color to draw bounding box. Default is red.
          thickness: line thickness. Default value is 4.
          display_str_list: list of strings to display in box
                            (each to be shown on its own line).
          use_normalized_coordinates: If True (default), treat coordinates
            ymin, xmin, ymax, xmax as relative to the image.  Otherwise treat
            coordinates as absolute.
        """
        image_pil = Image.fromarray(np.uint8(image)).convert('RGB')
        self.draw_bounding_box_on_image(
            image_pil, ymin, xmin, ymax, xmax, color, thickness, display_str_list, use_normalized_coordinates
        )
        np.copyto(image, np.array(image_pil))

    def draw_bounding_box_on_image(
            self,
            image,
            ymin,
            xmin,
            ymax,
            xmax,
            color='red',
            thickness=4,
            display_str_list=(),
            use_normalized_coordinates=True
    ):
        """Adds a bounding box to an image.
        Each string in display_str_list is displayed on a separate line above the
        bounding box in black text on a rectangle filled with the input 'color'.
        If the top of the bounding box extends to the edge of the image, the strings
        are displayed below the bounding box.
        Args:
          image: a PIL.Image object.
          ymin: ymin of bounding box.
          xmin: xmin of bounding box.
          ymax: ymax of bounding box.
          xmax: xmax of bounding box.
          color: color to draw bounding box. Default is red.
          thickness: line thickness. Default value is 4.
          display_str_list: list of strings to display in box
                            (each to be shown on its own line).
          use_normalized_coordinates: If True (default), treat coordinates
            ymin, xmin, ymax, xmax as relative to the image.  Otherwise treat
            coordinates as absolute.
        """
        draw = ImageDraw.Draw(image)
        im_width, im_height = image.size
        if use_normalized_coordinates:
            (left, right, top, bottom) = (xmin * im_width, xmax * im_width, ymin * im_height, ymax * im_height)

        else:
            (left, right, top, bottom) = (xmin, xmax, ymin, ymax)

        draw.line([(left, top), (left, bottom), (right, bottom), (right, top), (left, top)],
                  width=thickness,
                  fill=color)

        try:
            font = ImageFont.truetype('arial.ttf', 24)

        except IOError:
            font = ImageFont.load_default()

        # If the total height of the display strings added to the top of the bounding
        # box exceeds the top of the image, stack the strings below the bounding box
        # instead of above.
        display_str_heights = [font.getsize(ds)[1] for ds in display_str_list]
        # Each display_str has a top and bottom margin of 0.05x.
        total_display_str_height = (1 + 2 * 0.05) * sum(display_str_heights)

        if top > total_display_str_height:
            text_bottom = top

        else:
            text_bottom = bottom + total_display_str_height

        # Reverse list and print from bottom to top.
        for display_str in display_str_list[::-1]:
            text_width, text_height = font.getsize(display_str)
            margin = np.ceil(0.05 * text_height)
            draw.rectangle([(left, text_bottom - text_height - 2 * margin), (left + text_width, text_bottom)],
                           fill=color)
            draw.text((left + margin, text_bottom - text_height - margin), display_str, fill='black', font=font)
            text_bottom -= text_height - 2 * margin

    def draw_bounding_boxes_on_image_array(self, image, boxes, color='red', thickness=4, display_str_list_list=()):
        """Draws bounding boxes on image (numpy array).
        Args:
          image: a numpy array object.
          boxes: a 2 dimensional numpy array of [N, 4]: (ymin, xmin, ymax, xmax).
                 The coordinates are in normalized format between [0, 1].
          color: color to draw bounding box. Default is red.
          thickness: line thickness. Default value is 4.
          display_str_list_list: list of list of strings.
                                 a list of strings for each bounding box.
                                 The reason to pass a list of strings for a
                                 bounding box is that it might contain
                                 multiple labels.
        Raises:
          ValueError: if boxes is not a [N, 4] array
        """
        image_pil = Image.fromarray(image)
        self.draw_bounding_boxes_on_image(image_pil, boxes, color, thickness, display_str_list_list)
        np.copyto(image, np.array(image_pil))

    def draw_bounding_boxes_on_image(self, image, boxes, color='red', thickness=4, display_str_list_list=()):
        """Draws bounding boxes on image.
        Args:
          image: a PIL.Image object.
          boxes: a 2 dimensional numpy array of [N, 4]: (ymin, xmin, ymax, xmax).
                 The coordinates are in normalized format between [0, 1].
          color: color to draw bounding box. Default is red.
          thickness: line thickness. Default value is 4.
          display_str_list_list: list of list of strings.
                                 a list of strings for each bounding box.
                                 The reason to pass a list of strings for a
                                 bounding box is that it might contain
                                 multiple labels.
        Raises:
          ValueError: if boxes is not a [N, 4] array
        """
        boxes_shape = boxes.shape

        if not boxes_shape:
            return

        if len(boxes_shape) != 2 or boxes_shape[1] != 4:
            raise ValueError('Input must be of size [N, 4]')

        for i in range(boxes_shape[0]):
            display_str_list = ()

            if display_str_list_list:
                display_str_list = display_str_list_list[i]

            self.draw_bounding_box_on_image(
                image, boxes[i, 0], boxes[i, 1], boxes[i, 2], boxes[i, 3], color, thickness, display_str_list
            )

    def draw_keypoints_on_image_array(self, image, keypoints, color='red', radius=2, use_normalized_coordinates=True):
        """Draws keypoints on an image (numpy array).
        Args:
          image: a numpy array with shape [height, width, 3].
          keypoints: a numpy array with shape [num_keypoints, 2].
          color: color to draw the keypoints with. Default is red.
          radius: keypoint radius. Default value is 2.
          use_normalized_coordinates: if True (default), treat keypoint values as
            relative to the image.  Otherwise treat them as absolute.
        """
        image_pil = Image.fromarray(np.uint8(image)).convert('RGB')
        self.draw_keypoints_on_image(image_pil, keypoints, color, radius, use_normalized_coordinates)
        np.copyto(image, np.array(image_pil))

    def draw_keypoints_on_image(self, image, keypoints, color='red', radius=2, use_normalized_coordinates=True):
        """Draws keypoints on an image.
        Args:
          image: a PIL.Image object.
          keypoints: a numpy array with shape [num_keypoints, 2].
          color: color to draw the keypoints with. Default is red.
          radius: keypoint radius. Default value is 2.
          use_normalized_coordinates: if True (default), treat keypoint values as
            relative to the image.  Otherwise treat them as absolute.
        """
        draw = ImageDraw.Draw(image)
        im_width, im_height = image.size
        keypoints_x = [k[1] for k in keypoints]
        keypoints_y = [k[0] for k in keypoints]

        if use_normalized_coordinates:
            keypoints_x = tuple([im_width * x for x in keypoints_x])
            keypoints_y = tuple([im_height * y for y in keypoints_y])

        for keypoint_x, keypoint_y in zip(keypoints_x, keypoints_y):
            draw.ellipse([(keypoint_x - radius, keypoint_y - radius), (keypoint_x + radius, keypoint_y + radius)],
                         outline=color,
                         fill=color)

    def draw_mask_on_image_array(self, image, mask, color='red', alpha=0.7):
        """Draws mask on an image.
        Args:
          image: uint8 numpy array with shape (img_height, img_height, 3)
          mask: a uint8 numpy array of shape (img_height, img_height) with
            values between either 0 or 1.
          color: color to draw the keypoints with. Default is red.
          alpha: transparency value between 0 and 1. (default: 0.7)
        Raises:
          ValueError: On incorrect data type for image or masks.
        """
        if image.dtype != np.uint8:
            raise ValueError('`image` not of type np.uint8')

        if mask.dtype != np.uint8:
            raise ValueError('`mask` not of type np.uint8')

        if np.any(np.logical_and(mask != 1, mask != 0)):
            raise ValueError('`mask` elements should be in [0, 1]')

        rgb = ImageColor.getrgb(color)
        pil_image = Image.fromarray(image)

        solid_color = np.expand_dims(np.ones_like(mask), axis=2) * np.reshape(list(rgb), [1, 1, 3])
        pil_solid_color = Image.fromarray(np.uint8(solid_color)).convert('RGBA')
        pil_mask = Image.fromarray(np.uint8(255.0 * alpha * mask)).convert('L')
        pil_image = Image.composite(pil_solid_color, pil_image, pil_mask)
        np.copyto(image, np.array(pil_image.convert('RGB')))
