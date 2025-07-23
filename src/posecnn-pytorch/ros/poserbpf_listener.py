import time
import rospy
import tf
import message_filters
import cv2
import numpy as np
import torch
import torch.nn as nn
import threading
import sys
import scipy.io
import random
import datetime
import tf.transformations as tra
import matplotlib.pyplot as plt
import posecnn_cuda

# from queue import queue
from random import shuffle
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance_matrix as scipy_distance_matrix
from rospy_tutorials.srv import AddTwoInts
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROS_Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import JointState
from transforms3d.quaternions import mat2quat, quat2mat, qmult
from scipy.optimize import minimize
from geometry_msgs.msg import PoseStamped, PoseArray

from fcn.config import cfg, cfg_from_file, get_output_dir
from video_recorder import *
from utils.cython_bbox import bbox_overlaps
from utils.nms import *
from posecnn_pytorch.msg import DetectionList, Detection, BBox

lock = threading.Lock()
lock_tf = threading.Lock()

def ros_qt_to_rt(rot, trans):
    qt = np.zeros((4,), dtype=np.float32)
    qt[0] = rot[3]
    qt[1] = rot[0]
    qt[2] = rot[1]
    qt[3] = rot[2]
    obj_T = np.eye(4)
    obj_T[:3, :3] = quat2mat(qt)
    obj_T[:3, 3] = trans
    return obj_T


def get_relative_pose_from_tf(listener, source_frame, target_frame):
    first_time = True
    while True:
        try:
            stamp = rospy.Time.now()
            init_trans, init_rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except Exception as e:
            if first_time:
                print(str(e))
                first_time = False
            continue
    return ros_qt_to_rt(init_rot, init_trans), stamp


def rotation_matrix_from_vectors(A, B):
    A = A / np.linalg.norm(A)
    B = B / np.linalg.norm(B)
    v = np.cross(A, B)
    ssc = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + ssc + np.matmul(ssc, ssc) * (1 - np.dot(A, B)) / (np.linalg.norm(v))**2
    return R


class ImageListener:

    def __init__(self, pose_rbpf, gen_data):

        print(' *** Initializing PoseRBPF ROS Node ... ')

        # variables
        self.cv_bridge = CvBridge()
        self.count = 0
        self.objects = []
        self.frame_names = []
        self.frame_lost = []
        self.queue_size = 10
        self.gen_data = gen_data

        self.pose_rbpf = pose_rbpf
        self.dataset = pose_rbpf.dataset
        self.camera_type = cfg.TEST.ROS_CAMERA
        self.suffix = '_%02d' % (cfg.instance_id)
        self.prefix = '%02d_' % (cfg.instance_id)

        self.init_failure_steps = 0
        self.input_rgb = None
        self.input_depth = None
        self.input_seg = None
        self.input_rois = None
        self.input_stamp = None
        self.input_frame_id = None
        self.input_joint_states = None
        self.input_robot_joint_states = None
        self.main_thread_free = True
        self.kf_time_stamp = None

        # thread for publish poses
        self.tf_thread = None
        self.stop_event = None

        # initialize a node
        rospy.init_node('poserbpf_image_listener' + self.suffix)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        rospy.sleep(3.0)
        self.pose_pub = rospy.Publisher('poserbpf_image' + self.suffix, ROS_Image, queue_size=1)
        self.pose_pub_1 = rospy.Publisher('poserbpf_image_render' + self.suffix, ROS_Image, queue_size=1)
        self.detection_pub = rospy.Publisher('poserbpf/%02d/info' % cfg.instance_id, DetectionList, queue_size=1)

        # subscriber for camera information
        # self.base_frame = 'measured/base_link'
        if cfg.TEST.ROS_CAMERA == 'D415':
            self.base_frame = 'measured/camera_color_optical_frame'        
            rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image, queue_size=1)
            depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, queue_size=1)
            msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
            self.target_frame = self.base_frame
            self.camera_frame = 'measured/camera_color_optical_frame'
            self.forward_kinematics = True

            '''
            self.T_delta = np.array([[0.99911077, 0.04145749, -0.00767817, 0.003222],  # -0.003222, -0.013222 (left plus and right minus)
                                     [-0.04163608, 0.99882554, -0.02477858, 0.01589],  # -0.00289, 0.01089 (close plus and far minus)
                                     [0.0066419, 0.02507623, 0.99966348, 0.003118],
                                     [0., 0., 0., 1.]], dtype=np.float32)
            '''
            self.T_delta = np.eye(4, dtype=np.float32)
            
        elif cfg.TEST.ROS_CAMERA  == 'Fetch':
            self.base_frame = 'base_link'
            rgb_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image, queue_size=10)
            depth_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw', Image, queue_size=10)
            msg = rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo)
            self.camera_frame = 'head_camera_rgb_optical_frame'
            self.target_frame = self.base_frame
            self.forward_kinematics = True

        elif cfg.TEST.ROS_CAMERA == 'Azure':
            self.base_frame = 'measured/base_link'        
            rgb_sub = message_filters.Subscriber('/k4a/rgb/image_raw', Image, queue_size=1)
            depth_sub = message_filters.Subscriber('/k4a/depth_to_rgb/image_raw', Image, queue_size=1)
            msg = rospy.wait_for_message('/k4a/rgb/camera_info', CameraInfo)
            self.target_frame = self.base_frame
            self.camera_frame = 'rgb_camera_link'
            self.forward_kinematics = False
        elif cfg.TEST.ROS_CAMERA == 'ISAAC_SIM':
            rgb_sub = message_filters.Subscriber('/sim/left_color_camera/image', Image, queue_size=1)
            depth_sub = message_filters.Subscriber('/sim/left_depth_camera/image', Image, queue_size=1)
            msg = rospy.wait_for_message('/sim/left_color_camera/camera_info', CameraInfo)
            self.target_frame = self.base_frame
            self.forward_kinematics = True
        else:
            rgb_sub = message_filters.Subscriber('/%s/rgb/image_color' % (cfg.TEST.ROS_CAMERA), Image, queue_size=1)
            depth_sub = message_filters.Subscriber('/%s/depth_registered/image' % (cfg.TEST.ROS_CAMERA), Image, queue_size=1)
            msg = rospy.wait_for_message('/%s/rgb/camera_info' % (cfg.TEST.ROS_CAMERA), CameraInfo)
            self.forward_kinematics = False

        # camera to base transformation
        self.Tbc_now = np.eye(4, dtype=np.float32)
        self.Tbc_prev = np.eye(4, dtype=np.float32)
        self.camera_distance = 0
        self.pose_rbpf.forward_kinematics = self.forward_kinematics

        # service for reset poserbpf
        s = rospy.Service('reset_poserbpf' + self.suffix, AddTwoInts, self.reset_poserbpf)
        self.reset = False
        self.grasp_mode = False
        self.grasp_cls = -1

        K = np.array(msg.K).reshape(3, 3)
        self.intrinsic_matrix = K
        print('Intrinsics matrix : ')
        print(self.intrinsic_matrix)

        # set up ros service
        print(' PoseRBPF ROS Node is Initialized ! *** ')
        self.is_keyframe = False

        # subscriber for posecnn label
        label_sub = message_filters.Subscriber('/posecnn_label' + self.suffix, ROS_Image, queue_size=1)
        queue_size = 10
        slop_seconds = 0.2
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, label_sub], queue_size, slop_seconds)
        ts.registerCallback(self.callback)

        self.Tbr_kf = np.eye(4, dtype=np.float32)  # keyframe which is used for refine the object pose
        self.Tbr_kf_list = []
        self.Tco_kf_list = []
        self.record = False
        self.Tbr_save = np.eye(4, dtype=np.float32)
        self.image_disp = None

        # create pose publisher for each known object class
        self.pubs = []
        for i in range(1, len(cfg.TEST.CLASSES)):
            ind = cfg.TEST.CLASSES[i]
            if self.dataset._classes_all[ind][3] == '_':
                cls = self.prefix + self.dataset._classes_all[ind][4:]
            else:
                cls = self.prefix + self.dataset._classes_all[ind]
            self.pubs.append(rospy.Publisher('/objects/prior_pose/' + cls, PoseStamped, queue_size=1))

        # data saving directory
        self.scene = 0
        self.step = 0
        dataset_dir = './data_self/'
        now = datetime.datetime.now()
        seq_name = "{:%m%dT%H%M%S}/".format(now)
        self.save_dir = dataset_dir + seq_name
        if self.gen_data and not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # start pose thread
        self.start_publishing_tf()


    def start_publishing_tf(self):
        self.stop_event = threading.Event()
        self.tf_thread = threading.Thread(target=self.tf_thread_func)
        self.tf_thread.start()


    def stop_publishing_tf(self):
        if self.tf_thread is None:
            return False
        self.stop_event.set()
        self.tf_thread.join()
        return True


    def make_fake_detection(self, name):

        # create pose msg
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.target_frame
        msg.pose.orientation.x = 1
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 0
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0

        detection = Detection()
        detection.name = name
        detection.score = -1
        detection.roi.x1 = 0
        detection.roi.y1 = 0
        detection.roi.x2 = 0
        detection.roi.y2 = 0
        detection.pose = msg
        return detection


    def align_z_axis(self, pose):
        a = np.array([0, 0, 1])
        R = pose[:3, :3]
        A = np.matmul(R, a)
        B = np.array([0, 0, 1])
        R_delta = rotation_matrix_from_vectors(A, B)
        pose_new = pose.copy()
        pose_new[:3, :3] = np.matmul(R_delta, R)
        return pose_new


    # publish poses
    def tf_thread_func(self):
        rate = rospy.Rate(30.)
        while not self.stop_event.is_set() and not rospy.is_shutdown():

            # publish pose
            with lock_tf:
                detections = DetectionList()
                for i in range(self.pose_rbpf.num_rbpfs):

                    name = 'poserbpf/' + self.pose_rbpf.rbpfs[i].name
                    if self.grasp_mode and self.pose_rbpf.rbpfs[i].cls_id != self.grasp_cls:
                        self.br.sendTransform([0, 0, 0], [1, 0, 0, 0], rospy.Time.now(), name, self.base_frame)
                        detection = self.make_fake_detection(name)
                        detections.detections.append(detection)
                        continue

                    # if self.reset or (self.grasp_mode and not self.pose_rbpf.rbpfs[i].graspable) or not self.pose_rbpf.rbpfs[i].status:
                    if self.reset or not self.pose_rbpf.rbpfs[i].graspable or not self.pose_rbpf.rbpfs[i].status:
                        self.br.sendTransform([0, 0, 0], [1, 0, 0, 0], rospy.Time.now(), name, self.base_frame)
                        detection = self.make_fake_detection(name)
                        detections.detections.append(detection)
                        continue

                    if self.grasp_mode:
                        print('***************publish****************', name)

                    Tbo = self.pose_rbpf.rbpfs[i].T_in_base
                    if cfg.TEST.ALIGN_Z_AXIS:
                        Tbo = self.align_z_axis(Tbo)

                    # publish tf
                    t_bo = Tbo[:3, 3]
                    q_bo = mat2quat(Tbo[:3, :3])
                    self.br.sendTransform(t_bo, [q_bo[1], q_bo[2], q_bo[3], q_bo[0]], rospy.Time.now(), name, self.target_frame)

                    # create pose msg
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = self.target_frame
                    msg.pose.orientation.x = q_bo[1]
                    msg.pose.orientation.y = q_bo[2]
                    msg.pose.orientation.z = q_bo[3]
                    msg.pose.orientation.w = q_bo[0]
                    msg.pose.position.x = t_bo[0]
                    msg.pose.position.y = t_bo[1]
                    msg.pose.position.z = t_bo[2]
                    cls = self.pose_rbpf.rbpfs[i].cls_id
                    pub = self.pubs[cls-1]
                    pub.publish(msg)

                    detection = Detection()
                    detection.name = name
                    detection.score = self.pose_rbpf.rbpfs[i].roi[6]
                    detection.roi.x1 = int(self.pose_rbpf.rbpfs[i].roi[2])
                    detection.roi.y1 = int(self.pose_rbpf.rbpfs[i].roi[3])
                    detection.roi.x2 = int(self.pose_rbpf.rbpfs[i].roi[4])
                    detection.roi.y2 = int(self.pose_rbpf.rbpfs[i].roi[5])
                    detection.pose = msg
                    detections.detections.append(detection)

                self.detection_pub.publish(detections)
            rate.sleep()
            # self.stop_event.wait(timeout=0.1)


    # callback function to get images
    def callback(self, rgb, depth, label):

        self.Tbc_now, self.Tbc_stamp = get_relative_pose_from_tf(self.listener, self.camera_frame, self.base_frame)

        # decode image
        if depth is not None:
            if depth.encoding == '32FC1':
                depth_cv = self.cv_bridge.imgmsg_to_cv2(depth)
            elif depth.encoding == '16UC1':
                depth = self.cv_bridge.imgmsg_to_cv2(depth)
                depth_cv = depth.copy().astype(np.float32)
                depth_cv /= 1000.0
            else:
                rospy.logerr_throttle(1, 'Unsupported depth type. Expected 16UC1 or 32FC1, got {}'.format(depth.encoding))
                return
        else:
            depth_cv = None

        with lock:
            self.input_depth = depth_cv
            # rgb image used for posecnn detection
            self.input_rgb = self.cv_bridge.imgmsg_to_cv2(rgb, 'rgb8')
            # segmentation information from posecnn
            self.input_seg = self.cv_bridge.imgmsg_to_cv2(label, 'mono8')
            # other information
            self.input_stamp = rgb.header.stamp
            self.input_frame_id = rgb.header.frame_id


    def reset_poserbpf(self, req):
        self.reset = True
        self.pose_rbpf.reset = True
        self.req = req
        return 0


    def check_object_size(self, input_rgb, input_depth, input_seg, rois):
        rois_est = rois.copy()
        im_label = input_seg.copy()
        width = im_label.shape[1]
        height = im_label.shape[0]
        for i in range(rois_est.shape[0]):
            roi = rois_est[i, :]
            cls = int(roi[1])
            roi_w = roi[4] - roi[2]
            roi_h = roi[5] - roi[3]
            roi_size = min(roi_w, roi_h)
            cls_id = cfg.TEST.CLASSES.index(cfg.TRAIN.CLASSES[cls])
            n_init_samples = cfg.PF.N_PROCESS
            uv_init = np.zeros((2, ), dtype=np.float32)
            uv_init[0] = (roi[4] + roi[2]) / 2
            uv_init[1] = (roi[5] + roi[3]) / 2

            # codebook of the class
            codebook = self.pose_rbpf.codebooks[cls_id]
            render_dist = codebook['distance']
            intrinsic_matrix = codebook['intrinsic_matrix']
            cfg.PF.FU = intrinsic_matrix[0, 0]
            cfg.PF.FV = intrinsic_matrix[1, 1]
            z_init = (128 - 40) * render_dist / roi_size * self.intrinsic_matrix[0, 0] / cfg.PF.FU
            z_init = z_init[0, 0]

            # sample around the center of bounding box
            uv_h = np.array([uv_init[0], uv_init[1], 1])
            uv_h = np.repeat(np.expand_dims(uv_h, axis=0), n_init_samples, axis=0)
            bound = roi_w * 0.1
            uv_h[:, 0] += np.random.uniform(-bound, bound, (n_init_samples, ))
            bound = roi_h * 0.1
            uv_h[:, 1] += np.random.uniform(-bound, bound, (n_init_samples, ))
            uv_h[:, 0] = np.clip(uv_h[:, 0], 0, input_rgb.shape[1])
            uv_h[:, 1] = np.clip(uv_h[:, 1], 0, input_rgb.shape[0])

            # sampling with depth        
            uv_h_int = uv_h.astype(int)
            uv_h_int[:, 0] = np.clip(uv_h_int[:, 0], 0, input_rgb.shape[1] - 1)
            uv_h_int[:, 1] = np.clip(uv_h_int[:, 1], 0, input_rgb.shape[0] - 1)
            z = input_depth[uv_h_int[:, 1], uv_h_int[:, 0]]
            z[np.isnan(z)] = 0

            print(self.dataset._classes_all[cfg.TRAIN.CLASSES[int(roi[1])]], z_init, np.mean(z[z > 0]))
            if z_init - np.mean(z[z > 0]) > 0.25:
                cls_name = self.dataset._classes_all[cfg.TRAIN.CLASSES[int(roi[1])]] + '_small'
                for j in range(len(self.dataset._classes_all)):
                    if cls_name == self.dataset._classes_all[j]:
                        cls_test = cfg.TEST.CLASSES.index(j)
                        rois_est[i, 1] = cls_test
                        # change labels
                        x1 = max(int(roi[2]), 0)
                        y1 = max(int(roi[3]), 0)
                        x2 = min(int(roi[4]), width-1)
                        y2 = min(int(roi[5]), height-1)
                        labels = np.zeros((height, width), dtype=np.float32)
                        labels[y1:y2, x1:x2] = im_label[y1:y2, x1:x2]
                        labels[labels == cls] = cls_test
                        im_label[y1:y2, x1:x2] = labels[y1:y2, x1:x2]
                        print(i, 'small object ' + cls_name)
                        break
        return rois_est, im_label


    def process_data(self):
        # callback data
        with lock:
            input_stamp = self.input_stamp
            input_rgb = self.input_rgb.copy()
            input_depth = self.input_depth.copy()
            input_seg = self.input_seg.copy()
            input_Tbc = self.Tbc_now.copy()
            input_Tbc_stamp = self.Tbc_stamp

        if self.reset:
            if self.req.a == 0:
                print('=========================reset===========================')
                self.pose_rbpf.rbpfs = []
                self.pose_rbpf.num_objects_per_class = np.zeros((len(cfg.TEST.CLASSES), 10), dtype=np.int32)
                self.scene += 1
                self.step = 0
                self.gen_data = True
            elif self.req.a == 1:
                self.gen_data = False
            elif self.req.a == 2:
                self.grasp_mode = True
                self.grasp_cls = self.req.b
                detections = DetectionList()
                for i in range(self.pose_rbpf.num_rbpfs):
                    self.pose_rbpf.rbpfs[i].need_filter = True
                    self.pose_rbpf.rbpfs[i].status = False
                    name = 'poserbpf/' + self.pose_rbpf.rbpfs[i].name
                    self.br.sendTransform([0, 0, 0], [1, 0, 0, 0], rospy.Time.now(), name, self.target_frame)
                    detection = self.make_fake_detection(name)
                    detections.detections.append(detection)
                self.detection_pub.publish(detections)

            elif self.req.a == 3:
                self.grasp_mode = False
                self.grasp_cls = -1
            self.reset = False
            self.pose_rbpf.reset = False

        # detection information of the target object
        rois_est = np.zeros((0, 7), dtype=np.float32)
        # TODO look for multiple object instances
        max_objects = 5
        for i in range(len(cfg.TEST.CLASSES)):

            for object_id in range(max_objects):
                suffix_frame = '_%02d_roi' % (object_id)

                # check posecnn frame
                ind = cfg.TEST.CLASSES[i]
                if self.dataset._classes_all[ind][3] == '_':
                    source_frame = 'posecnn/' + self.prefix + self.dataset._classes_all[ind][4:] + suffix_frame
                else:
                    source_frame = 'posecnn/' + self.prefix + self.dataset._classes_all[ind] + suffix_frame

                try:
                    # print('look for posecnn detection ' + source_frame)
                    trans, rot = self.listener.lookupTransform(self.target_frame, source_frame, rospy.Time(0))
                    n = trans[0]
                    secs = trans[1]
                    now = rospy.Time.now()
                    if abs(now.secs - secs) > 1.0:
                        print('posecnn pose for %s time out %f %f' % (source_frame, now.secs, secs))
                        continue
                    roi = np.zeros((1, 7), dtype=np.float32)
                    roi[0, 0] = 0
                    roi[0, 1] = i
                    if self.grasp_mode and roi[0, 1] != self.grasp_cls:
                        continue
                    roi[0, 2] = rot[0] * n
                    roi[0, 3] = rot[1] * n
                    roi[0, 4] = rot[2] * n
                    roi[0, 5] = rot[3] * n
                    roi[0, 6] = trans[2]
                    rois_est = np.concatenate((rois_est, roi), axis=0)
                    print('find posecnn detection ' + source_frame)
                except:
                    continue

        if rois_est.shape[0] > 0:
            # non-maximum suppression within class
            index = nms(rois_est, 0.2)
            rois_est = rois_est[index, :]

            # only keep the roi close to the image center in grasp mode
            if self.grasp_mode and rois_est.shape[0] > 1:
                cx = (rois_est[:, 2] + rois_est[:, 4]) / 2.0
                cy = (rois_est[:, 3] + rois_est[:, 5]) / 2.0
                width = input_rgb.shape[1]
                height = input_rgb.shape[0]
                distance = np.multiply(cx - width / 2.0, cx - width / 2.0) + np.multiply(cy - height / 2.0, cy - height / 2.0)
                index = np.argmin(distance)
                rois_est = rois_est[index, :].reshape((1, 7))

        # compute camera distance
        d = np.linalg.norm(input_Tbc[:3, 3] - self.pose_rbpf.Tbc[:3, 3])
        self.pose_rbpf.Tbc = input_Tbc
        # moving camera, need filtering
        if d > 0.005:
            for i in range(self.pose_rbpf.num_rbpfs):
                # self.pose_rbpf.rbpfs[i].need_filter = True
                T_bo = self.pose_rbpf.rbpfs[i].T_in_base
                RT = np.matmul(np.linalg.inv(input_Tbc), T_bo)
                self.pose_rbpf.rbpfs[i].pose[:4] = mat2quat(RT[:3, :3])
                self.pose_rbpf.rbpfs[i].pose[4:] = RT[:3, 3]
                roi = self.pose_rbpf.rbpfs[i].roi
                self.pose_rbpf.rbpfs[i].roi[2:6] = self.pose_rbpf.compute_box(self.dataset, self.intrinsic_matrix, int(roi[1]), self.pose_rbpf.rbpfs[i].pose)

            # if self.image_disp is not None:
            if cfg.TEST.VISUALIZE and self.image_disp is not None:
                print('image time stamp', input_stamp.to_sec())
                print('Tbc time stamp', input_Tbc_stamp.to_sec())

                fig = plt.figure()
                ax = fig.add_subplot(2, 2, 1)
                plt.imshow(self.image_disp)
                ax.set_title('previous result')
                image_tensor, pcloud_tensor = self.pose_rbpf.render_image_all(self.intrinsic_matrix, self.grasp_mode, self.grasp_cls)
                im = (0.4 * input_rgb.astype(np.float32) / 255.0 + 0.6 * image_tensor.cpu().numpy()) * 255
                im = np.clip(im, 0, 255)
                im = im.astype(np.uint8)
                ax = fig.add_subplot(2, 2, 2)
                plt.imshow(im)
                ax.set_title('current result')
                ax = fig.add_subplot(2, 2, 4)
                plt.imshow(input_seg)
                # draw roi
                for i in range(rois_est.shape[0]):
                    roi = rois_est[i, 2:6]
                    cls = int(rois_est[i, 1])
                    color = np.array(self.dataset._class_colors_test[cls])/255.0
                    x1 = roi[0]
                    y1 = roi[1]
                    x2 = roi[2]
                    y2 = roi[3]
                    plt.gca().add_patch(plt.Rectangle((x1, y1), x2-x1, y2-y1, fill=False, edgecolor=color, linewidth=3, clip_on=False))
                ax.set_title('current label')
                plt.show()

        # differentiate objects with different sizes
        if cfg.TEST.CHECK_SIZE:
            rois_est, input_seg = self.check_object_size(input_rgb, input_depth, input_seg, rois_est)

        # call pose estimation function
        save, image_disp, image_disp_1 = self.process_image_multi_obj(input_rgb, input_depth, input_seg, rois_est)

        if self.grasp_mode:
            print('****************************Grasping Mode (%s) ****************************' % (self.dataset._classes_all[cfg.TEST.CLASSES[self.grasp_cls]]))
        else:
            print('****************************Tracking Mode**********************************')

        # visualization
        # '''
        if image_disp is not None:
            pose_msg = self.cv_bridge.cv2_to_imgmsg(image_disp)
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.input_frame_id
            pose_msg.encoding = 'rgb8'
            self.pose_pub.publish(pose_msg)
            self.image_disp = image_disp
            
            pose_msg = self.cv_bridge.cv2_to_imgmsg(image_disp_1)
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.input_frame_id
            pose_msg.encoding = 'rgb8'
            self.pose_pub_1.publish(pose_msg)
            
        # '''

        # save data
        if save and self.gen_data:
            self.save_data(input_rgb, input_depth, input_seg)


    # function for pose etimation and tracking
    def process_image_multi_obj(self, rgb, depth, im_label, rois):

        image_rgb = rgb.astype(np.float32) / 255.0
        image_bgr = image_rgb[:, :, (2, 1, 0)]
        image_bgr = torch.from_numpy(image_bgr).cuda()
        im_label = torch.from_numpy(im_label).cuda()

        # backproject depth
        depth = torch.from_numpy(depth).cuda()
        fx = self.intrinsic_matrix[0, 0]
        fy = self.intrinsic_matrix[1, 1]
        px = self.intrinsic_matrix[0, 2]
        py = self.intrinsic_matrix[1, 2]
        # dpoints = backproject(depth, self.intrinsic_matrix)
        im_pcloud = posecnn_cuda.backproject_forward(fx, fy, px, py, depth)[0]

        # collect rois from rbpfs
        num_rbpfs = self.pose_rbpf.num_rbpfs
        rois_rbpf = np.zeros((num_rbpfs, 7), dtype=np.float32)
        for i in range(num_rbpfs):
            rois_rbpf[i, :] = self.pose_rbpf.rbpfs[i].roi
            self.pose_rbpf.rbpfs[i].roi_assign = None

        # data association based on bounding box overlap
        num_rois = rois.shape[0]
        assigned_rois = np.zeros((num_rois, ), dtype=np.int32)
        if num_rbpfs > 0 and num_rois > 0:
            # overlaps: (rois x gt_boxes) (batch_id, x1, y1, x2, y2)
            overlaps = bbox_overlaps(np.ascontiguousarray(rois_rbpf[:, (1, 2, 3, 4, 5)], dtype=np.float),
                np.ascontiguousarray(rois[:, (1, 2, 3, 4, 5)], dtype=np.float))

            # assign rois to rbpfs
            assignment = overlaps.argmax(axis=1)
            max_overlaps = overlaps.max(axis=1)
            unassigned = []
            for i in range(num_rbpfs):
                if max_overlaps[i] > 0.2:
                    self.pose_rbpf.rbpfs[i].roi_assign = rois[assignment[i]]
                    assigned_rois[assignment[i]] = 1
                else:
                    unassigned.append(i)

            # check if there are un-assigned rois
            index = np.where(assigned_rois == 0)[0]

            # if there is un-assigned rbpfs
            if len(unassigned) > 0 and len(index) > 0:
                for i in range(len(unassigned)):
                    for j in range(len(index)):
                        if assigned_rois[index[j]] == 0 and self.pose_rbpf.rbpfs[unassigned[i]].roi[1] == rois[index[j], 1]:
                            self.pose_rbpf.rbpfs[unassigned[i]].roi_assign = rois[index[j]]
                            assigned_rois[index[j]] = 1

        elif num_rbpfs == 0 and num_rois == 0:
            return False, None, None

        # initialize new object
        if num_rois > 0:
            good_initial = True
        else:
            good_initial = False

        start_time = rospy.Time.now()
        for i in range(num_rois):
            if assigned_rois[i]:
                continue

            print('Initializing detection {} ... '.format(i))
            roi = rois[i].copy()
            print(roi)
            self.pose_rbpf.estimation_poserbpf(roi, self.intrinsic_matrix, image_bgr, depth, im_pcloud, im_label, self.grasp_mode, self.grasp_cls)

            # pose evaluation
            image_tensor, pcloud_tensor = self.pose_rbpf.render_image_all(self.intrinsic_matrix, self.grasp_mode, self.grasp_cls)
            cls = cfg.TEST.CLASSES[int(roi[1])]
            sim, depth_error, vis_ratio = self.pose_rbpf.evaluate_6d_pose(self.pose_rbpf.rbpfs[-1].roi, self.pose_rbpf.rbpfs[-1].pose, cls, \
                image_bgr, image_tensor, pcloud_tensor, depth, self.intrinsic_matrix, im_label)
            print('Initialization : Object: {}, Sim obs: {}, Depth Err: {:.3}, Vis Ratio: {:.2}'.format(i, sim, depth_error, vis_ratio))

            if sim < cfg.PF.THRESHOLD_SIM or torch.isnan(depth_error) or depth_error > cfg.PF.THRESHOLD_DEPTH or vis_ratio < cfg.PF.THRESHOLD_RATIO:
                print('===================is NOT initialized!=================')
                self.pose_rbpf.num_objects_per_class[self.pose_rbpf.rbpfs[-1].cls_id, self.pose_rbpf.rbpfs[-1].object_id] = 0
                with lock_tf:
                    del self.pose_rbpf.rbpfs[-1]
                good_initial = False
            else:
                print('===================is initialized!======================')
                self.pose_rbpf.rbpfs[-1].roi_assign = roi
                if self.grasp_mode:
                    if not (sim < cfg.PF.THRESHOLD_SIM_GRASPING or depth_error > cfg.PF.THRESHOLD_DEPTH_GRASPING or vis_ratio < cfg.PF.THRESHOLD_RATIO_GRASPING):
                        self.pose_rbpf.rbpfs[-1].graspable = True
                        self.pose_rbpf.rbpfs[-1].status = True
                        self.pose_rbpf.rbpfs[-1].need_filter = False
        print('initialization time %.6f' % (rospy.Time.now() - start_time).to_sec())

        # filter all the objects
        print('Filtering objects')
        save, image_tensor = self.pose_rbpf.filtering_poserbpf(self.intrinsic_matrix, image_bgr, depth, im_pcloud, im_label, self.grasp_mode, self.grasp_cls)
        print('*********full time %.6f' % (rospy.Time.now() - start_time).to_sec())

        # non-maximum suppression within class
        num = self.pose_rbpf.num_rbpfs
        status = np.zeros((num, ), dtype=np.int32)
        rois = np.zeros((num, 7), dtype=np.float32)
        for i in range(num):
            rois[i, :6] = self.pose_rbpf.rbpfs[i].roi[:6]
            rois[i, 6] = self.pose_rbpf.rbpfs[i].num_frame
        keep = nms(rois, 0.5)
        status[keep] = 1

        # remove untracked objects
        for i in range(num):
            if status[i] == 0 or self.pose_rbpf.rbpfs[i].num_lost >= cfg.TEST.NUM_LOST:
                print('###############remove rbpf#################')
                self.pose_rbpf.num_objects_per_class[self.pose_rbpf.rbpfs[i].cls_id, self.pose_rbpf.rbpfs[i].object_id] = 0
                status[i] = 0
                save = False
        with lock_tf:
            self.pose_rbpf.rbpfs = [self.pose_rbpf.rbpfs[i] for i in range(num) if status[i] > 0]

        if self.pose_rbpf.num_rbpfs == 0:
            save = False

        # image to publish for visualization
        # '''
        if image_tensor is not None:
            image_disp = (0.4 * image_bgr[:, :, (2, 1, 0)] + 0.6 * image_tensor) * 255
            image_disp_1 = image_tensor * 255
        else:
            image_disp = 0.4 * image_bgr[:, :, (2, 1, 0)] * 255
            image_disp_1 = image_disp
        image_disp = torch.clamp(image_disp, 0, 255).byte().cpu().numpy()
        image_disp_1 = torch.clamp(image_disp_1, 0, 255).byte().cpu().numpy()        
        # '''
        # image_disp = None
        
        return save & good_initial, image_disp, image_disp_1


    # save data
    def save_data(self, rgb, depth, label):
        factor_depth = 1000.0
        classes_all = self.dataset._classes_all

        cls_indexes = []
        cls_names = []
        num = self.pose_rbpf.num_rbpfs
        poses = np.zeros((3, 4, num), dtype=np.float32)
        for i in range(num):
            cls_index = cfg.TEST.CLASSES[self.pose_rbpf.rbpfs[i].cls_id]
            cls_indexes.append(cls_index)
            cls_names.append(classes_all[cls_index])
            poses[:3, :3, i] = quat2mat(self.pose_rbpf.rbpfs[i].pose[:4])
            poses[:3, 3, i] = self.pose_rbpf.rbpfs[i].pose[4:]

        if not os.path.exists(self.save_dir + 'scene_{:02}/'.format(self.scene)):
            os.makedirs(self.save_dir+ 'scene_{:02}/'.format(self.scene))

        meta = {'cls_names': cls_names, 'cls_indexes': cls_indexes, 'poses': poses,
                'intrinsic_matrix': self.intrinsic_matrix, 'factor_depth': factor_depth}
        filename = self.save_dir + 'scene_{:02}/'.format(self.scene) + '{:08}_meta.mat'.format(self.step)
        scipy.io.savemat(filename, meta, do_compression=True)
        print('save data to {}'.format(filename))

        # convert rgb to bgr8
        rgb_save = rgb[:, :, (2, 1, 0)]
        image_tensor, _ = self.pose_rbpf.render_image_all(self.intrinsic_matrix)
        rgb_render = image_tensor.cpu().numpy()
        rgb_render = np.clip(rgb_render, 0, 1) * 255
        rgb_render_save = 0.4 * rgb_save.astype(np.float32) + 0.6 * rgb_render[:, :, (2, 1, 0)].astype(np.float32)
        rgb_render_save = rgb_render_save.astype(np.uint8)

        # convert depth to unit16
        depth_save = np.array(depth * factor_depth, dtype=np.uint16)

        # segmentation labels
        label_save = self.dataset.labels_to_image(label)
        label_save = label_save[:, :, (2, 1, 0)]

        save_name_rgb = self.save_dir + 'scene_{:02}/'.format(self.scene) + '{:08}_color.jpg'.format(self.step)
        save_name_rgb_render = self.save_dir + 'scene_{:02}/'.format(self.scene) + '{:08}_color_render.jpg'.format(self.step)
        save_name_depth = self.save_dir + 'scene_{:02}/'.format(self.scene) + '{:08}_depth.png'.format(self.step)
        save_name_label = self.save_dir + 'scene_{:02}/'.format(self.scene) + '{:08}_label.jpg'.format(self.step)
        cv2.imwrite(save_name_rgb, rgb_save)
        cv2.imwrite(save_name_rgb_render, rgb_render_save)
        cv2.imwrite(save_name_depth, depth_save)
        cv2.imwrite(save_name_label, label_save)
        self.step += 1


    # globally search objects without detection
    def process_data_global_search(self):
        # callback data
        with lock:
            input_stamp = self.input_stamp
            input_rgb = self.input_rgb.copy()
            input_depth = self.input_depth.copy()
            input_seg = self.input_seg.copy()
            input_Tbc = self.Tbc_now.copy()
            input_Tbc_stamp = self.Tbc_stamp

        if self.reset:
            if self.req.a == 0:
                print('=========================reset===========================')
                self.pose_rbpf.rbpfs = []
                self.pose_rbpf.num_objects_per_class = np.zeros((len(cfg.TEST.CLASSES), 10), dtype=np.int32)
                self.scene += 1
                self.step = 0
                self.gen_data = True
            elif self.req.a == 1:
                self.gen_data = False
            elif self.req.a == 2:
                self.grasp_mode = True
                self.grasp_cls = self.req.b
                detections = DetectionList()
                for i in range(self.pose_rbpf.num_rbpfs):
                    self.pose_rbpf.rbpfs[i].need_filter = True
                    self.pose_rbpf.rbpfs[i].status = False
                    name = 'poserbpf/' + self.pose_rbpf.rbpfs[i].name
                    self.br.sendTransform([0, 0, 0], [1, 0, 0, 0], rospy.Time.now(), name, self.target_frame)
                    detection = self.make_fake_detection(name)
                    detections.detections.append(detection)
                self.detection_pub.publish(detections)

            elif self.req.a == 3:
                self.grasp_mode = False
                self.grasp_cls = -1
            self.reset = False
            self.pose_rbpf.reset = False

        # compute camera distance
        d = np.linalg.norm(input_Tbc[:3, 3] - self.pose_rbpf.Tbc[:3, 3])
        self.pose_rbpf.Tbc = input_Tbc
        # moving camera, need filtering
        if d > 0.005:
            for i in range(self.pose_rbpf.num_rbpfs):
                # self.pose_rbpf.rbpfs[i].need_filter = True
                T_bo = self.pose_rbpf.rbpfs[i].T_in_base
                RT = np.matmul(np.linalg.inv(input_Tbc), T_bo)
                self.pose_rbpf.rbpfs[i].pose[:4] = mat2quat(RT[:3, :3])
                self.pose_rbpf.rbpfs[i].pose[4:] = RT[:3, 3]
                roi = self.pose_rbpf.rbpfs[i].roi
                self.pose_rbpf.rbpfs[i].roi[2:6] = self.pose_rbpf.compute_box(self.dataset, 
                    self.intrinsic_matrix, int(roi[1]), self.pose_rbpf.rbpfs[i].pose)

            # if self.image_disp is not None:
            if cfg.TEST.VISUALIZE and self.image_disp is not None:
                print('image time stamp', input_stamp.to_sec())
                print('Tbc time stamp', input_Tbc_stamp.to_sec())

                fig = plt.figure()
                ax = fig.add_subplot(1, 2, 1)
                plt.imshow(self.image_disp)
                ax.set_title('previous result')
                image_tensor, pcloud_tensor = self.pose_rbpf.render_image_all(self.intrinsic_matrix, self.grasp_mode, self.grasp_cls)
                im = (0.4 * input_rgb.astype(np.float32) / 255.0 + 0.6 * image_tensor.cpu().numpy()) * 255
                im = np.clip(im, 0, 255)
                im = im.astype(np.uint8)
                ax = fig.add_subplot(1, 2, 2)
                plt.imshow(im)
                ax.set_title('current result')
                plt.show()

        # call pose estimation function
        image_rgb = input_rgb.astype(np.float32) / 255.0
        image_bgr = image_rgb[:, :, (2, 1, 0)]
        image_bgr = torch.from_numpy(image_bgr).cuda()
        im_label = torch.from_numpy(input_seg).cuda()

        # backproject depth
        depth = torch.from_numpy(input_depth).cuda()
        fx = self.intrinsic_matrix[0, 0]
        fy = self.intrinsic_matrix[1, 1]
        px = self.intrinsic_matrix[0, 2]
        py = self.intrinsic_matrix[1, 2]
        im_pcloud = posecnn_cuda.backproject_forward(fx, fy, px, py, depth)[0]

        # collect rois from rbpfs
        num_rbpfs = self.pose_rbpf.num_rbpfs
        rois_rbpf = np.zeros((num_rbpfs, 7), dtype=np.float32)
        for i in range(num_rbpfs):
            rois_rbpf[i, :] = self.pose_rbpf.rbpfs[i].roi
            self.pose_rbpf.rbpfs[i].roi_assign = None

        # initialize object globally
        start_time = rospy.Time.now()
        good_initial = True
        if num_rbpfs == 0:

            print('Initializing object globally')
            self.pose_rbpf.estimation_poserbpf_globally(self.intrinsic_matrix, image_bgr, depth, im_pcloud, im_label, self.grasp_mode, self.grasp_cls)

            # pose evaluation
            image_tensor, pcloud_tensor = self.pose_rbpf.render_image_all(self.intrinsic_matrix, self.grasp_mode, self.grasp_cls)
            cls = cfg.TEST.CLASSES[int(self.pose_rbpf.rbpfs[-1].roi[1])]
            sim, depth_error, vis_ratio = self.pose_rbpf.evaluate_6d_pose(self.pose_rbpf.rbpfs[-1].roi, self.pose_rbpf.rbpfs[-1].pose, cls, \
                image_bgr, image_tensor, pcloud_tensor, depth, self.intrinsic_matrix, im_label)
            print('Initialization : Sim obs: {}, Depth Err: {:.3}, Vis Ratio: {:.2}'.format(sim, depth_error, vis_ratio))

            if sim < cfg.PF.THRESHOLD_SIM or torch.isnan(depth_error) or depth_error > cfg.PF.THRESHOLD_DEPTH or vis_ratio < cfg.PF.THRESHOLD_RATIO:
                print('===================is NOT initialized!=================')
                self.pose_rbpf.num_objects_per_class[self.pose_rbpf.rbpfs[-1].cls_id, self.pose_rbpf.rbpfs[-1].object_id] = 0
                with lock_tf:
                    del self.pose_rbpf.rbpfs[-1]
                good_initial = False
            else:
                print('===================is initialized!======================')
                self.pose_rbpf.rbpfs[-1].roi_assign = self.pose_rbpf.rbpfs[-1].roi.copy()
                if self.grasp_mode:
                    if not (sim < cfg.PF.THRESHOLD_SIM_GRASPING or depth_error > cfg.PF.THRESHOLD_DEPTH_GRASPING 
                            or vis_ratio < cfg.PF.THRESHOLD_RATIO_GRASPING):
                        self.pose_rbpf.rbpfs[-1].graspable = True
                        self.pose_rbpf.rbpfs[-1].status = True
                        self.pose_rbpf.rbpfs[-1].need_filter = False
        print('initialization time %.6f' % (rospy.Time.now() - start_time).to_sec())

        # filter all the objects
        print('Filtering objects')
        save, image_tensor = self.pose_rbpf.filtering_poserbpf(self.intrinsic_matrix, image_bgr, depth, im_pcloud, im_label, self.grasp_mode, self.grasp_cls)
        print('*********full time %.6f' % (rospy.Time.now() - start_time).to_sec())

        # non-maximum suppression within class
        num = self.pose_rbpf.num_rbpfs
        status = np.zeros((num, ), dtype=np.int32)
        rois = np.zeros((num, 7), dtype=np.float32)
        for i in range(num):
            rois[i, :6] = self.pose_rbpf.rbpfs[i].roi[:6]
            rois[i, 6] = self.pose_rbpf.rbpfs[i].num_frame
        keep = nms(rois, 0.5)
        status[keep] = 1

        # remove untracked objects
        for i in range(num):
            if status[i] == 0 or self.pose_rbpf.rbpfs[i].num_lost >= cfg.TEST.NUM_LOST:
                print('###############remove rbpf#################')
                self.pose_rbpf.num_objects_per_class[self.pose_rbpf.rbpfs[i].cls_id, self.pose_rbpf.rbpfs[i].object_id] = 0
                status[i] = 0
                save = False
        with lock_tf:
            self.pose_rbpf.rbpfs = [self.pose_rbpf.rbpfs[i] for i in range(num) if status[i] > 0]

        if self.pose_rbpf.num_rbpfs == 0:
            save = False

        # image to publish for visualization
        # '''
        if image_tensor is not None:
            image_disp = (0.4 * image_bgr[:, :, (2, 1, 0)] + 0.6 * image_tensor) * 255
        else:
            image_disp = 0.4 * image_bgr[:, :, (2, 1, 0)] * 255
        image_disp = torch.clamp(image_disp, 0, 255).byte().cpu().numpy()
        # '''
        # image_disp = None
        save = save & good_initial

        if self.grasp_mode:
            print('****************************Grasping Mode (%s) ****************************' % (self.dataset._classes_all[cfg.TEST.CLASSES[self.grasp_cls]]))
        else:
            print('****************************Tracking Mode**********************************')

        # visualization
        # '''
        if image_disp is not None:
            pose_msg = self.cv_bridge.cv2_to_imgmsg(image_disp)
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.input_frame_id
            pose_msg.encoding = 'rgb8'
            self.pose_pub.publish(pose_msg)
            self.image_disp = image_disp
        # '''

        # save data
        if save and self.gen_data:
            self.save_data(input_rgb, input_depth, input_seg)
