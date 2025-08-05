
(cl:in-package :asdf)

(defsystem "posecnn_pytorch-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BBox" :depends-on ("_package_BBox"))
    (:file "_package_BBox" :depends-on ("_package"))
    (:file "Detection" :depends-on ("_package_Detection"))
    (:file "_package_Detection" :depends-on ("_package"))
    (:file "DetectionList" :depends-on ("_package_DetectionList"))
    (:file "_package_DetectionList" :depends-on ("_package"))
  ))