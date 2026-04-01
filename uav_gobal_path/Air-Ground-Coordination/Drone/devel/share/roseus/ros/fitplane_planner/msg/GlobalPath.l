;; Auto-generated. Do not edit!


(when (boundp 'fitplane_planner::GlobalPath)
  (if (not (find-package "FITPLANE_PLANNER"))
    (make-package "FITPLANE_PLANNER"))
  (shadow 'GlobalPath (find-package "FITPLANE_PLANNER")))
(unless (find-package "FITPLANE_PLANNER::GLOBALPATH")
  (make-package "FITPLANE_PLANNER::GLOBALPATH"))

(in-package "ROS")
;;//! \htmlinclude GlobalPath.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fitplane_planner::GlobalPath
  :super ros::object
  :slots (_header _map_version _poses _corridor_half_width _path_confidence ))

(defmethod fitplane_planner::GlobalPath
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:map_version __map_version) 0)
    ((:poses __poses) ())
    ((:corridor_half_width __corridor_half_width) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:path_confidence __path_confidence) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _map_version (round __map_version))
   (setq _poses __poses)
   (setq _corridor_half_width __corridor_half_width)
   (setq _path_confidence __path_confidence)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:map_version
   (&optional __map_version)
   (if __map_version (setq _map_version __map_version)) _map_version)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:corridor_half_width
   (&optional __corridor_half_width)
   (if __corridor_half_width (setq _corridor_half_width __corridor_half_width)) _corridor_half_width)
  (:path_confidence
   (&optional __path_confidence)
   (if __path_confidence (setq _path_confidence __path_confidence)) _path_confidence)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _map_version
    4
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ;; float32[] _corridor_half_width
    (* 4    (length _corridor_half_width)) 4
    ;; float32[] _path_confidence
    (* 4    (length _path_confidence)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _map_version
       (write-long _map_version s)
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;; float32[] _corridor_half_width
     (write-long (length _corridor_half_width) s)
     (dotimes (i (length _corridor_half_width))
       (sys::poke (elt _corridor_half_width i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _path_confidence
     (write-long (length _path_confidence) s)
     (dotimes (i (length _path_confidence))
       (sys::poke (elt _path_confidence i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _map_version
     (setq _map_version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _corridor_half_width
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _corridor_half_width (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _corridor_half_width i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _path_confidence
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_confidence (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _path_confidence i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get fitplane_planner::GlobalPath :md5sum-) "9916ef7263e04949af022154c3512e69")
(setf (get fitplane_planner::GlobalPath :datatype-) "fitplane_planner/GlobalPath")
(setf (get fitplane_planner::GlobalPath :definition-)
      "# Global reference path for UGV (UAV → UGV)
std_msgs/Header header
uint32 map_version                # 地图/路径版本（UAV 侧每次接收新图自增）
geometry_msgs/Pose[] poses        # 路径控制点（世界系）
float32[] corridor_half_width     # 对应poses的走廊半宽 w_i，单位m
float32[] path_confidence         # 对应poses的段置信度 c_i ∈ [0,1]

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



(provide :fitplane_planner/GlobalPath "9916ef7263e04949af022154c3512e69")


