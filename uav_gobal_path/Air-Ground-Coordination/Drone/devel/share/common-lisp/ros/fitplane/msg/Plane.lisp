; Auto-generated. Do not edit!


(cl:in-package fitplane-msg)


;//! \htmlinclude Plane.msg.html

(cl:defclass <Plane> (roslisp-msg-protocol:ros-message)
  ((PlaneCellHeight
    :reader PlaneCellHeight
    :initarg :PlaneCellHeight
    :type cl:float
    :initform 0.0)
   (PlaneCellAngle
    :reader PlaneCellAngle
    :initarg :PlaneCellAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Plane (<Plane>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plane>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plane)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fitplane-msg:<Plane> is deprecated: use fitplane-msg:Plane instead.")))

(cl:ensure-generic-function 'PlaneCellHeight-val :lambda-list '(m))
(cl:defmethod PlaneCellHeight-val ((m <Plane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:PlaneCellHeight-val is deprecated.  Use fitplane-msg:PlaneCellHeight instead.")
  (PlaneCellHeight m))

(cl:ensure-generic-function 'PlaneCellAngle-val :lambda-list '(m))
(cl:defmethod PlaneCellAngle-val ((m <Plane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fitplane-msg:PlaneCellAngle-val is deprecated.  Use fitplane-msg:PlaneCellAngle instead.")
  (PlaneCellAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plane>) ostream)
  "Serializes a message object of type '<Plane>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PlaneCellHeight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PlaneCellAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plane>) istream)
  "Deserializes a message object of type '<Plane>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PlaneCellHeight) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PlaneCellAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plane>)))
  "Returns string type for a message object of type '<Plane>"
  "fitplane/Plane")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plane)))
  "Returns string type for a message object of type 'Plane"
  "fitplane/Plane")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plane>)))
  "Returns md5sum for a message object of type '<Plane>"
  "92467c111abb460b6d16e58287e7864b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plane)))
  "Returns md5sum for a message object of type 'Plane"
  "92467c111abb460b6d16e58287e7864b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plane>)))
  "Returns full string definition for message of type '<Plane>"
  (cl:format cl:nil "float32 PlaneCellHeight~%float32 PlaneCellAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plane)))
  "Returns full string definition for message of type 'Plane"
  (cl:format cl:nil "float32 PlaneCellHeight~%float32 PlaneCellAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plane>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plane>))
  "Converts a ROS message object to a list"
  (cl:list 'Plane
    (cl:cons ':PlaneCellHeight (PlaneCellHeight msg))
    (cl:cons ':PlaneCellAngle (PlaneCellAngle msg))
))
