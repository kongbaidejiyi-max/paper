; Auto-generated. Do not edit!


(cl:in-package fitplane-msg)


;//! \htmlinclude GridPlaneInfo.msg.html

(cl:defclass <GridPlaneInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (traversability
    :reader traversability
    :initarg :traversability
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (plane_angle
    :reader plane_angle
    :initarg :plane_angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (plane_height
    :reader plane_height
    :initarg :plane_height
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (occupancy
    :reader occupancy
    :initarg :occupancy
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass GridPlaneInfo (<GridPlaneInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GridPlaneInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GridPlaneInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fitplane-msg:<GridPlaneInfo> is deprecated: use fitplane-msg:GridPlaneInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:header-val is deprecated.  Use fitplane-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:resolution-val is deprecated.  Use fitplane-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:width-val is deprecated.  Use fitplane-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:height-val is deprecated.  Use fitplane-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:origin-val is deprecated.  Use fitplane-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'traversability-val :lambda-list '(m))
(cl:defmethod traversability-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:traversability-val is deprecated.  Use fitplane-msg:traversability instead.")
  (traversability m))

(cl:ensure-generic-function 'plane_angle-val :lambda-list '(m))
(cl:defmethod plane_angle-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:plane_angle-val is deprecated.  Use fitplane-msg:plane_angle instead.")
  (plane_angle m))

(cl:ensure-generic-function 'plane_height-val :lambda-list '(m))
(cl:defmethod plane_height-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:plane_height-val is deprecated.  Use fitplane-msg:plane_height instead.")
  (plane_height m))

(cl:ensure-generic-function 'occupancy-val :lambda-list '(m))
(cl:defmethod occupancy-val ((m <GridPlaneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:occupancy-val is deprecated.  Use fitplane-msg:occupancy instead.")
  (occupancy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GridPlaneInfo>) ostream)
  "Serializes a message object of type '<GridPlaneInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traversability))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'traversability))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'plane_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'plane_angle))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'plane_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'plane_height))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'occupancy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'occupancy))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GridPlaneInfo>) istream)
  "Deserializes a message object of type '<GridPlaneInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traversability) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traversability)))
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
  (cl:setf (cl:slot-value msg 'plane_angle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'plane_angle)))
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
  (cl:setf (cl:slot-value msg 'plane_height) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'plane_height)))
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
  (cl:setf (cl:slot-value msg 'occupancy) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'occupancy)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GridPlaneInfo>)))
  "Returns string type for a message object of type '<GridPlaneInfo>"
  "fitplane/GridPlaneInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GridPlaneInfo)))
  "Returns string type for a message object of type 'GridPlaneInfo"
  "fitplane/GridPlaneInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GridPlaneInfo>)))
  "Returns md5sum for a message object of type '<GridPlaneInfo>"
  "16e811bb180035be79c09c3399b1506a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GridPlaneInfo)))
  "Returns md5sum for a message object of type 'GridPlaneInfo"
  "16e811bb180035be79c09c3399b1506a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GridPlaneInfo>)))
  "Returns full string definition for message of type '<GridPlaneInfo>"
  (cl:format cl:nil "Header header~%float32 resolution~%uint32 width~%uint32 height~%geometry_msgs/Point origin~%float32[] traversability  # 每个栅格的可通行性~%float32[] plane_angle    # 每个栅格的平面角度~%float32[] plane_height   # 每个栅格的高度~%int32[] occupancy       # 栅格占用状态：-1未知，0空闲，100占用 ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GridPlaneInfo)))
  "Returns full string definition for message of type 'GridPlaneInfo"
  (cl:format cl:nil "Header header~%float32 resolution~%uint32 width~%uint32 height~%geometry_msgs/Point origin~%float32[] traversability  # 每个栅格的可通行性~%float32[] plane_angle    # 每个栅格的平面角度~%float32[] plane_height   # 每个栅格的高度~%int32[] occupancy       # 栅格占用状态：-1未知，0空闲，100占用 ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GridPlaneInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traversability) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'plane_angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'plane_height) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'occupancy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GridPlaneInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'GridPlaneInfo
    (cl:cons ':header (header msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':traversability (traversability msg))
    (cl:cons ':plane_angle (plane_angle msg))
    (cl:cons ':plane_height (plane_height msg))
    (cl:cons ':occupancy (occupancy msg))
))
