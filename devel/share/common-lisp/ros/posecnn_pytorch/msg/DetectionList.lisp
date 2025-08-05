; Auto-generated. Do not edit!


(cl:in-package posecnn_pytorch-msg)


;//! \htmlinclude DetectionList.msg.html

(cl:defclass <DetectionList> (roslisp-msg-protocol:ros-message)
  ((detections
    :reader detections
    :initarg :detections
    :type (cl:vector posecnn_pytorch-msg:Detection)
   :initform (cl:make-array 0 :element-type 'posecnn_pytorch-msg:Detection :initial-element (cl:make-instance 'posecnn_pytorch-msg:Detection))))
)

(cl:defclass DetectionList (<DetectionList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectionList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectionList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name posecnn_pytorch-msg:<DetectionList> is deprecated: use posecnn_pytorch-msg:DetectionList instead.")))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <DetectionList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader posecnn_pytorch-msg:detections-val is deprecated.  Use posecnn_pytorch-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectionList>) ostream)
  "Serializes a message object of type '<DetectionList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectionList>) istream)
  "Deserializes a message object of type '<DetectionList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'posecnn_pytorch-msg:Detection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectionList>)))
  "Returns string type for a message object of type '<DetectionList>"
  "posecnn_pytorch/DetectionList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectionList)))
  "Returns string type for a message object of type 'DetectionList"
  "posecnn_pytorch/DetectionList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectionList>)))
  "Returns md5sum for a message object of type '<DetectionList>"
  "85de709e8e4b1999046a6c2a4f7beaec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectionList)))
  "Returns md5sum for a message object of type 'DetectionList"
  "85de709e8e4b1999046a6c2a4f7beaec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectionList>)))
  "Returns full string definition for message of type '<DetectionList>"
  (cl:format cl:nil "posecnn_pytorch/Detection[] detections~%~%================================================================================~%MSG: posecnn_pytorch/Detection~%string name~%float32 score~%posecnn_pytorch/BBox roi~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: posecnn_pytorch/BBox~%int32 x1~%int32 y1~%int32 x2~%int32 y2~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectionList)))
  "Returns full string definition for message of type 'DetectionList"
  (cl:format cl:nil "posecnn_pytorch/Detection[] detections~%~%================================================================================~%MSG: posecnn_pytorch/Detection~%string name~%float32 score~%posecnn_pytorch/BBox roi~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: posecnn_pytorch/BBox~%int32 x1~%int32 y1~%int32 x2~%int32 y2~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectionList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectionList>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectionList
    (cl:cons ':detections (detections msg))
))
