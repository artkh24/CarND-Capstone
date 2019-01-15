from styx_msgs.msg import TrafficLight
import numpy as np
import os
import tensorflow as tf
from utils import label_map_util
import cv2
import yaml
import rospy
from random import randint

#uncomment when want to save classified images and setting is_save_image=True  but also in this case need to install matplotlib
#from utils import visualization_utils as vis_util

class TLClassifier(object):
    def __init__(self):
        self.counter=0
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.safe_load(config_string)
        self.im_width = self.config['camera_info']['image_width']
        self.im_height = self.config['camera_info']['image_height']
        
        MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
        GRAPH='frozen_inference_graph.pb'
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_FROZEN_GRAPH =os.path.dirname(os.path.realpath(__file__))+'/'+ MODEL_NAME + '/'+ GRAPH
        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.dirname(os.path.realpath(__file__))+'/data/mscoco_label_map.pbtxt'
        
        # Load a frozen Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
       
        # Loading label map
        self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
    
    def run_inference_for_single_image(self,image):
      with self.detection_graph.as_default():
        with tf.Session() as sess:
          # Get handles to input and output tensors
          ops = tf.get_default_graph().get_operations()
          all_tensor_names = {output.name for op in ops for output in op.outputs}
          tensor_dict = {}
          for key in [
              'num_detections', 'detection_boxes', 'detection_scores',
              'detection_classes', 'detection_masks'
          ]:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
              tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                  tensor_name)
          if 'detection_masks' in tensor_dict:
            # The following processing is only for single image
            detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
            detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
            # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
            real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
            detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
            detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                detection_masks, detection_boxes, image.shape[0], image.shape[1])
            detection_masks_reframed = tf.cast(
                tf.greater(detection_masks_reframed, 0.5), tf.uint8)
            # Follow the convention by adding back the batch dimension
            tensor_dict['detection_masks'] = tf.expand_dims(
                detection_masks_reframed, 0)
          image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

          # Run inference
          output_dict = sess.run(tensor_dict,
                                 feed_dict={image_tensor: np.expand_dims(image, 0)})

          # all outputs are float32 numpy arrays, so convert types as appropriate
          output_dict['num_detections'] = int(output_dict['num_detections'][0])
          output_dict['detection_classes'] = output_dict[
              'detection_classes'][0].astype(np.uint8)
          output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
          output_dict['detection_scores'] = output_dict['detection_scores'][0]
          if 'detection_masks' in output_dict:
            output_dict['detection_masks'] = output_dict['detection_masks'][0]
      return output_dict 
  
    def load_image_into_numpy_array(self,image):
      return np.array(image).reshape(
          (self.im_height, self.im_width, 3)).astype(np.uint8)
    
    def brighter_region(self,ymin,ymax,maxLoc):
        height=ymax-ymin
        dist=height/3.0
        y=maxLoc[1]
        if y <=dist:
            return TrafficLight.RED
        elif y<=2*dist:
            return TrafficLight.YELLOW
        else:
            return TrafficLight.GREEN
    
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.counter+=1
        if self.counter<5:
            return TrafficLight.UNKNOWN
        self.counter=0
        image_np =self.load_image_into_numpy_array(image)
        output_dict = self.run_inference_for_single_image(image_np)
        boxes = output_dict['detection_boxes']
        classes = output_dict['detection_classes']
        scores = output_dict['detection_scores']
        
        #Traffic light class is 10 in labels
        TRAFFIC_LIGHT_CLASS=10
        traffic_lights=np.argwhere(classes==TRAFFIC_LIGHT_CLASS)
        if len(traffic_lights)==0:
            return TrafficLight.UNKNOWN
        traffic_scores=np.take(scores,traffic_lights)
        traffic_boxes=np.take(boxes,traffic_lights,axis=0)
        max_score_index=np.argmax(traffic_scores)
        max_score=traffic_scores[max_score_index]
        if max_score < 0.5:
            return TrafficLight.UNKNOWN
        box=traffic_boxes[max_score_index][0]
        
        height = image_np.shape[0]
        width = image_np.shape[1]
        
        # denormalizing box values
        ymin = int(box[0] * height)
        xmin = int(box[1] * width)
        ymax = int(box[2] * height)
        xmax = int(box[3] * width)
        pad = int((xmax-xmin)* 0.2)
        trl_box = image_np[ymin+pad:ymax-pad, xmin+pad:xmax-pad]
        gray_trl = cv2.cvtColor(trl_box, cv2.COLOR_RGB2GRAY)
        gray_trl = cv2.GaussianBlur(gray_trl, (35, 35), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray_trl)
        
        #save image
        is_save_image=False
        if is_save_image:
            cv2.circle(trl_box, maxLoc, 16, (0, 0, 255), 2)
            dir = 'data/light_images'
            if not os.path.exists(dir):
                os.makedirs(dir)
            rand=randint(0,100000000)
            res=self.brighter_region(ymin+pad,ymax-pad,maxLoc)
            f_name = "tl_{}_{}.jpg".format(rand,res)
            f_name1 = "tl_{}_{}_1.jpg".format(rand,res)
            pathname = '{}/{}'.format(dir, f_name)
            pathname1 = '{}/{}'.format(dir, f_name1)
            cv2.imwrite(pathname, trl_box)
            vis_util.visualize_boxes_and_labels_on_image_array(
                  image_np,
                  output_dict['detection_boxes'],
                  output_dict['detection_classes'],
                  output_dict['detection_scores'],
                  self.category_index,
                  instance_masks=output_dict.get('detection_masks'),
                  use_normalized_coordinates=True,
                  line_thickness=8)
            cv2.imwrite(pathname1, image_np)
        return self.brighter_region(ymin+pad,ymax-pad,maxLoc)
        