;; Auto-generated. Do not edit!


(when (boundp 'posecnn_pytorch::Detection)
  (if (not (find-package "POSECNN_PYTORCH"))
    (make-package "POSECNN_PYTORCH"))
  (shadow 'Detection (find-package "POSECNN_PYTORCH")))
(unless (find-package "POSECNN_PYTORCH::DETECTION")
  (make-package "POSECNN_PYTORCH::DETECTION"))

(in-package "ROS")
;;//! \htmlinclude Detection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass posecnn_pytorch::Detection
  :super ros::object
  :slots (_name _score _roi _pose ))

(defmethod posecnn_pytorch::Detection
  (:init
   (&key
    ((:name __name) "")
    ((:score __score) 0.0)
    ((:roi __roi) (instance posecnn_pytorch::BBox :init))
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _score (float __score))
   (setq _roi __roi)
   (setq _pose __pose)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:roi
   (&rest __roi)
   (if (keywordp (car __roi))
       (send* _roi __roi)
     (progn
       (if __roi (setq _roi (car __roi)))
       _roi)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float32 _score
    4
    ;; posecnn_pytorch/BBox _roi
    (send _roi :serialization-length)
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _score
       (sys::poke _score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; posecnn_pytorch/BBox _roi
       (send _roi :serialize s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _score
     (setq _score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; posecnn_pytorch/BBox _roi
     (send _roi :deserialize buf ptr-) (incf ptr- (send _roi :serialization-length))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get posecnn_pytorch::Detection :md5sum-) "4be425bb4306d8ff3be362c5aaed7540")
(setf (get posecnn_pytorch::Detection :datatype-) "posecnn_pytorch/Detection")
(setf (get posecnn_pytorch::Detection :definition-)
      "string name
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



(provide :posecnn_pytorch/Detection "4be425bb4306d8ff3be362c5aaed7540")


