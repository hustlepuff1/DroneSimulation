#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes

# Add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # Add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # Relative path

# Import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        # Read parameters from ROS parameter server
        self.conf_thres = rospy.get_param("~confidence_threshold", 0.75)
        self.iou_thres = rospy.get_param("~iou_threshold", 0.45)
        self.agnostic_nms = rospy.get_param("~agnostic_nms", True)
        self.max_det = rospy.get_param("~maximum_detections", 1000)
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness", 3)
        self.view_image = rospy.get_param("~view_image", True)
        weights = rospy.get_param("~weights", "/home/h/catkin_ws/src/yolov5_ros/src/landing.pt")
        
        # Initialize model
        self.device = select_device(rospy.get_param("~device", ""))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn", True), data=rospy.get_param("~data", ""))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h", 480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (self.pt or self.jit or self.onnx or self.engine) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # Batch size
        cudnn.benchmark = True  # Set True to speed up constant image size inference
        self.model.warmup()  # Warmup

        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic", "/iris/usb_cam/image_raw"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "detections"), BoundingBoxes, queue_size=10
        )

        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image", False)
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic", "annotated_image"), Image, queue_size=10
            )

        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        im, im0 = self.preprocess(im)

        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        # added
        image_height = data.height
        image_width = data.width




        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        if len(det):
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            for *xyxy, conf, cls in reversed(det):
                bounding_box = BoundingBox()
                c = int(cls)
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf
                bounding_box.xmin = int(xyxy[0])
                bounding_box.ymin = int(xyxy[1])
                bounding_box.xmax = int(xyxy[2])
                bounding_box.ymax = int(xyxy[3])

                # Calculate center point - added
                center_x = (bounding_box.xmin + bounding_box.xmax) / 2
                center_y = (bounding_box.ymin + bounding_box.ymax) / 2

                # Print center point - added
                center_x = center_x - image_width / 2
                center_y = center_y - image_height / 2
                rospy.loginfo(f"Object: {bounding_box.Class}, Center X: {center_x}, Center Y: {center_y}")




                bounding_boxes.bounding_boxes.append(bounding_box)

                if self.publish_image or self.view_image:
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))

            im0 = annotator.result()

        self.pred_pub.publish(bounding_boxes)

        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond

        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))

    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)
        return img, img0

if __name__ == "__main__":
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.spin()
