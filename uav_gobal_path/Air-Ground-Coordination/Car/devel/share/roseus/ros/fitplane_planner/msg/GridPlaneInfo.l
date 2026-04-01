;; Auto-generated. Do not edit!


(when (boundp 'fitplane_planner::GridPlaneInfo)
  (if (not (find-package "FITPLANE_PLANNER"))
    (make-package "FITPLANE_PLANNER"))
  (shadow 'GridPlaneInfo (find-package "FITPLANE_PLANNER")))
(unless (find-package "FITPLANE_PLANNER::GRIDPLANEINFO")
  (make-package "FITPLANE_PLANNER::GRIDPLANEINFO"))

(in-package "ROS")
;;//! \htmlinclude GridPlaneInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fitplane_planner::GridPlaneInfo
  :super ros::object
  :slots (_header _resolution _width _height _origin _traversability _plane_angle _plane_height _occupancy ))

(defmethod fitplane_planner::GridPlaneInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:resolution __resolution) 0.0)
    ((:width __width) 0)
    ((:height __height) 0)
    ((:origin __origin) (instance geometry_msgs::Point :init))
    ((:traversability __traversability) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:plane_angle __plane_angle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:plane_height __plane_height) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:occupancy __occupancy) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _resolution (float __resolution))
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _origin __origin)
   (setq _traversability __traversability)
   (setq _plane_angle __plane_angle)
   (setq _plane_height __plane_height)
   (setq _occupancy __occupancy)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:traversability
   (&optional __traversability)
   (if __traversability (setq _traversability __traversability)) _traversability)
  (:plane_angle
   (&optional __plane_angle)
   (if __plane_angle (setq _plane_angle __plane_angle)) _plane_angle)
  (:plane_height
   (&optional __plane_height)
   (if __plane_height (setq _plane_height __plane_height)) _plane_height)
  (:occupancy
   (&optional __occupancy)
   (if __occupancy (setq _occupancy __occupancy)) _occupancy)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _resolution
    4
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ;; geometry_msgs/Point _origin
    (send _origin :serialization-length)
    ;; float32[] _traversability
    (* 4    (length _traversability)) 4
    ;; float32[] _plane_angle
    (* 4    (length _plane_angle)) 4
    ;; float32[] _plane_height
    (* 4    (length _plane_height)) 4
    ;; int32[] _occupancy
    (* 4    (length _occupancy)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;; geometry_msgs/Point _origin
       (send _origin :serialize s)
     ;; float32[] _traversability
     (write-long (length _traversability) s)
     (dotimes (i (length _traversability))
       (sys::poke (elt _traversability i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _plane_angle
     (write-long (length _plane_angle) s)
     (dotimes (i (length _plane_angle))
       (sys::poke (elt _plane_angle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _plane_height
     (write-long (length _plane_height) s)
     (dotimes (i (length _plane_height))
       (sys::poke (elt _plane_height i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _occupancy
     (write-long (length _occupancy) s)
     (dotimes (i (length _occupancy))
       (write-long (elt _occupancy i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; float32[] _traversability
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traversability (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _traversability i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _plane_angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _plane_angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _plane_angle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _plane_height
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _plane_height (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _plane_height i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _occupancy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _occupancy (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _occupancy i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get fitplane_planner::GridPlaneInfo :md5sum-) "16e811bb180035be79c09c3399b1506a")
(setf (get fitplane_planner::GridPlaneInfo :datatype-) "fitplane_planner/GridPlaneInfo")
(setf (get fitplane_planner::GridPlaneInfo :definition-)
      "Header header
float32 resolution
uint32 width
uint32 height
geometry_msgs/Point origin
float32[] traversability  # 每个栅格的可通行性
float32[] plane_angle    # 每个栅格的平面角度
float32[] plane_height   # 每个栅格的高度
int32[] occupancy       # 栅格占用状态：-1未知，0空闲，100占用 
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :fitplane_planner/GridPlaneInfo "16e811bb180035be79c09c3399b1506a")


