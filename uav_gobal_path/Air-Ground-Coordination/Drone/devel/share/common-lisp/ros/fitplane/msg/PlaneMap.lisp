; Auto-generated. Do not edit!


(cl:in-package fitplane-msg)


;//! \htmlinclude PlaneMap.msg.html

(cl:defclass <PlaneMap> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0)
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (origin_x
    :reader origin_x
    :initarg :origin_x
    :type cl:float
    :initform 0.0)
   (origin_y
    :reader origin_y
    :initarg :origin_y
    :type cl:float
    :initform 0.0)
   (PlaneGridMap
    :reader PlaneGridMap
    :initarg :PlaneGridMap
    :type (cl:vector fitplane-msg:Plane)
   :initform (cl:make-array 0 :element-type 'fitplane-msg:Plane :initial-element (cl:make-instance 'fitplane-msg:Plane))))
)

(cl:defclass PlaneMap (<PlaneMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaneMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaneMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fitplane-msg:<PlaneMap> is deprecated: use fitplane-msg:PlaneMap instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <PlaneMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:width-val is deprecated.  Use fitplane-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <PlaneMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:height-val is deprecated.  Use fitplane-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <PlaneMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:resolution-val is deprecated.  Use fitplane-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'origin_x-val :lambda-list '(m))
(cl:defmethod origin_x-val ((m <PlaneMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:origin_x-val is deprecated.  Use fitplane-msg:origin_x instead.")
  (origin_x m))

(cl:ensure-generic-function 'origin_y-val :lambda-list '(m))
(cl:defmethod origin_y-val ((m <PlaneMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:origin_y-val is deprecated.  Use fitplane-msg:origin_y instead.")
  (origin_y m))

(cl:ensure-generic-function 'PlaneGridMap-val :lambda-list '(m))
(cl:defmethod PlaneGridMap-val ((m <PlaneMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:PlaneGridMap-val is deprecated.  Use fitplane-msg:PlaneGridMap instead.")
  (PlaneGridMap m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaneMap>) ostream)
  "Serializes a message object of type '<PlaneMap>"
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'origin_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'origin_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PlaneGridMap))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'PlaneGridMap))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaneMap>) istream)
  "Deserializes a message object of type '<PlaneMap>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'origin_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'origin_y) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PlaneGridMap) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PlaneGridMap)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fitplane-msg:Plane))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaneMap>)))
  "Returns string type for a message object of type '<PlaneMap>"
  "fitplane/PlaneMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaneMap)))
  "Returns string type for a message object of type 'PlaneMap"
  "fitplane/PlaneMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaneMap>)))
  "Returns md5sum for a message object of type '<PlaneMap>"
  "0eee82ac8b4aa09e68a738e9785183cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaneMap)))
  "Returns md5sum for a message object of type 'PlaneMap"
  "0eee82ac8b4aa09e68a738e9785183cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaneMap>)))
  "Returns full string definition for message of type '<PlaneMap>"
  (cl:format cl:nil "int16 width~%int16 height~%float32 resolution~%float32 origin_x~%float32 origin_y~%Plane[] PlaneGridMap~%================================================================================~%MSG: fitplane/Plane~%float32 PlaneCellHeight~%float32 PlaneCellAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaneMap)))
  "Returns full string definition for message of type 'PlaneMap"
  (cl:format cl:nil "int16 width~%int16 height~%float32 resolution~%float32 origin_x~%float32 origin_y~%Plane[] PlaneGridMap~%================================================================================~%MSG: fitplane/Plane~%float32 PlaneCellHeight~%float32 PlaneCellAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaneMap>))
  (cl:+ 0
     2
     2
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PlaneGridMap) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaneMap>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaneMap
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':origin_x (origin_x msg))
    (cl:cons ':origin_y (origin_y msg))
    (cl:cons ':PlaneGridMap (PlaneGridMap msg))
))
