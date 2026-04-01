;; Auto-generated. Do not edit!


(when (boundp 'fitplane::PlaneMap)
  (if (not (find-package "FITPLANE"))
    (make-package "FITPLANE"))
  (shadow 'PlaneMap (find-package "FITPLANE")))
(unless (find-package "FITPLANE::PLANEMAP")
  (make-package "FITPLANE::PLANEMAP"))

(in-package "ROS")
;;//! \htmlinclude PlaneMap.msg.html


(defclass fitplane::PlaneMap
  :super ros::object
  :slots (_width _height _resolution _origin_x _origin_y _PlaneGridMap ))

(defmethod fitplane::PlaneMap
  (:init
   (&key
    ((:width __width) 0)
    ((:height __height) 0)
    ((:resolution __resolution) 0.0)
    ((:origin_x __origin_x) 0.0)
    ((:origin_y __origin_y) 0.0)
    ((:PlaneGridMap __PlaneGridMap) ())
    )
   (send-super :init)
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _resolution (float __resolution))
   (setq _origin_x (float __origin_x))
   (setq _origin_y (float __origin_y))
   (setq _PlaneGridMap __PlaneGridMap)
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:origin_x
   (&optional __origin_x)
   (if __origin_x (setq _origin_x __origin_x)) _origin_x)
  (:origin_y
   (&optional __origin_y)
   (if __origin_y (setq _origin_y __origin_y)) _origin_y)
  (:PlaneGridMap
   (&rest __PlaneGridMap)
   (if (keywordp (car __PlaneGridMap))
       (send* _PlaneGridMap __PlaneGridMap)
     (progn
       (if __PlaneGridMap (setq _PlaneGridMap (car __PlaneGridMap)))
       _PlaneGridMap)))
  (:serialization-length
   ()
   (+
    ;; int16 _width
    2
    ;; int16 _height
    2
    ;; float32 _resolution
    4
    ;; float32 _origin_x
    4
    ;; float32 _origin_y
    4
    ;; fitplane/Plane[] _PlaneGridMap
    (apply #'+ (send-all _PlaneGridMap :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _width
       (write-word _width s)
     ;; int16 _height
       (write-word _height s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _origin_x
       (sys::poke _origin_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _origin_y
       (sys::poke _origin_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; fitplane/Plane[] _PlaneGridMap
     (write-long (length _PlaneGridMap) s)
     (dolist (elem _PlaneGridMap)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _width
     (setq _width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _height
     (setq _height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _origin_x
     (setq _origin_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _origin_y
     (setq _origin_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; fitplane/Plane[] _PlaneGridMap
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _PlaneGridMap (let (r) (dotimes (i n) (push (instance fitplane::Plane :init) r)) r))
     (dolist (elem- _PlaneGridMap)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fitplane::PlaneMap :md5sum-) "0eee82ac8b4aa09e68a738e9785183cb")
(setf (get fitplane::PlaneMap :datatype-) "fitplane/PlaneMap")
(setf (get fitplane::PlaneMap :definition-)
      "int16 width
int16 height
float32 resolution
float32 origin_x
float32 origin_y
Plane[] PlaneGridMap
================================================================================
MSG: fitplane/Plane
float32 PlaneCellHeight
float32 PlaneCellAngle

")



(provide :fitplane/PlaneMap "0eee82ac8b4aa09e68a738e9785183cb")


