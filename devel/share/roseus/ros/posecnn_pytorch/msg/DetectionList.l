;; Auto-generated. Do not edit!


(when (boundp 'posecnn_pytorch::DetectionList)
  (if (not (find-package "POSECNN_PYTORCH"))
    (make-package "POSECNN_PYTORCH"))
  (shadow 'DetectionList (find-package "POSECNN_PYTORCH")))
(unless (find-package "POSECNN_PYTORCH::DETECTIONLIST")
  (make-package "POSECNN_PYTORCH::DETECTIONLIST"))

(in-package "ROS")
;;//! \htmlinclude DetectionList.msg.html


(defclass posecnn_pytorch::DetectionList
  :super ros::object
  :slots (_detections ))

(defmethod posecnn_pytorch::DetectionList
  (:init
   (&key
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _detections __detections)
   self)
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; posecnn_pytorch/Detection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; posecnn_pytorch/Detection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; posecnn_pytorch/Detection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance posecnn_pytorch::Detection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get posecnn_pytorch::DetectionList :md5sum-) "85de709e8e4b1999046a6c2a4f7beaec")
(setf (get posecnn_pytorch::DetectionList :datatype-) "posecnn_pytorch/DetectionList")
(setf (get posecnn_pytorch::DetectionList :definition-)
      "posecnn_pytorch/Detection[] detections

================================================================================
MSG: posecnn_pytorch/Detection
string name
float32 score
posecnn_pytorch/BBox roi
geometry_msgs/PoseStamped pose

================================================================================
MSG: posecnn_pytorch/BBox
int32 x1
int32 y1
int32 x2
int32 y2

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :posecnn_pytorch/DetectionList "85de709e8e4b1999046a6c2a4f7beaec")


