; Auto-generated. Do not edit!


(cl:in-package fitplane_planner-msg)


;//! \htmlinclude GlobalPath.msg.html

(cl:defclass <GlobalPath> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (map_version
    :reader map_version
    :initarg :map_version
    :type cl:integer
    :initform 0)
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (corridor_half_width
    :reader corridor_half_width
    :initarg :corridor_half_width
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (path_confidence
    :reader path_confidence
    :initarg :path_confidence
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GlobalPath (<GlobalPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GlobalPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GlobalPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fitplane_planner-msg:<GlobalPath> is deprecated: use fitplane_planner-msg:GlobalPath instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GlobalPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane_planner-msg:header-val is deprecated.  Use fitplane_planner-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'map_version-val :lambda-list '(m))
(cl:defmethod map_version-val ((m <GlobalPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane_planner-msg:map_version-val is deprecated.  Use fitplane_planner-msg:map_version instead.")
  (map_version m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <GlobalPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane_planner-msg:poses-val is deprecated.  Use fitplane_planner-msg:poses instead.")
  (poses m))

(cl:ensure-generic-function 'corridor_half_width-val :lambda-list '(m))
(cl:defmethod corridor_half_width-val ((m <GlobalPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane_planner-msg:corridor_half_width-val is deprecated.  Use fitplane_planner-msg:corridor_half_width instead.")
  (corridor_half_width m))

(cl:ensure-generic-function 'path_confidence-val :lambda-list '(m))
(cl:defmethod path_confidence-val ((m <GlobalPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane_planner-msg:path_confidence-val is deprecated.  Use fitplane_planner-msg:path_confidence instead.")
  (path_confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GlobalPath>) ostream)
  "Serializes a message object of type '<GlobalPath>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'map_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'map_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'map_version)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corridor_half_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'corridor_half_width))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'path_confidence))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GlobalPath>) istream)
  "Deserializes a message object of type '<GlobalPath>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'map_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'map_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'map_version)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corridor_half_width) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corridor_half_width)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_confidence) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_confidence)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GlobalPath>)))
  "Returns string type for a message object of type '<GlobalPath>"
  "fitplane_planner/GlobalPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GlobalPath)))
  "Returns string type for a message object of type 'GlobalPath"
  "fitplane_planner/GlobalPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GlobalPath>)))
  "Returns md5sum for a message object of type '<GlobalPath>"
  "9916ef7263e04949af022154c3512e69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GlobalPath)))
  "Returns md5sum for a message object of type 'GlobalPath"
  "9916ef7263e04949af022154c3512e69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GlobalPath>)))
  "Returns full string definition for message of type '<GlobalPath>"
  (cl:format cl:nil "# Global reference path for UGV (UAV → UGV)~%std_msgs/Header header~%uint32 map_version                # 地图/路径版本（UAV 侧每次接收新图自增）~%geometry_msgs/Pose[] poses        # 路径控制点（世界系）~%float32[] corridor_half_width     # 对应poses的走廊半宽 w_i，单位m~%float32[] path_confidence         # 对应poses的段置信度 c_i ∈ [0,1]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GlobalPath)))
  "Returns full string definition for message of type 'GlobalPath"
  (cl:format cl:nil "# Global reference path for UGV (UAV → UGV)~%std_msgs/Header header~%uint32 map_version                # 地图/路径版本（UAV 侧每次接收新图自增）~%geometry_msgs/Pose[] poses        # 路径控制点（世界系）~%float32[] corridor_half_width     # 对应poses的走廊半宽 w_i，单位m~%float32[] path_confidence         # 对应poses的段置信度 c_i ∈ [0,1]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GlobalPath>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corridor_half_width) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_confidence) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GlobalPath>))
  "Converts a ROS message object to a list"
  (cl:list 'GlobalPath
    (cl:cons ':header (header msg))
    (cl:cons ':map_version (map_version msg))
    (cl:cons ':poses (poses msg))
    (cl:cons ':corridor_half_width (corridor_half_width msg))
    (cl:cons ':path_confidence (path_confidence msg))
))
