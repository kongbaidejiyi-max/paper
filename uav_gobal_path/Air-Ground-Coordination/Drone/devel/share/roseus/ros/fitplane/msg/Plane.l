;; Auto-generated. Do not edit!


(when (boundp 'fitplane::Plane)
  (if (not (find-package "FITPLANE"))
    (make-package "FITPLANE"))
  (shadow 'Plane (find-package "FITPLANE")))
(unless (find-package "FITPLANE::PLANE")
  (make-package "FITPLANE::PLANE"))

(in-package "ROS")
;;//! \htmlinclude Plane.msg.html


(defclass fitplane::Plane
  :super ros::object
  :slots (_PlaneCellHeight _PlaneCellAngle ))

(defmethod fitplane::Plane
  (:init
   (&key
    ((:PlaneCellHeight __PlaneCellHeight) 0.0)
    ((:PlaneCellAngle __PlaneCellAngle) 0.0)
    )
   (send-super :init)
   (setq _PlaneCellHeight (float __PlaneCellHeight))
   (setq _PlaneCellAngle (float __PlaneCellAngle))
   self)
  (:PlaneCellHeight
   (&optional __PlaneCellHeight)
   (if __PlaneCellHeight (setq _PlaneCellHeight __PlaneCellHeight)) _PlaneCellHeight)
  (:PlaneCellAngle
   (&optional __PlaneCellAngle)
   (if __PlaneCellAngle (setq _PlaneCellAngle __PlaneCellAngle)) _PlaneCellAngle)
  (:serialization-length
   ()
   (+
    ;; float32 _PlaneCellHeight
    4
    ;; float32 _PlaneCellAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _PlaneCellHeight
       (sys::poke _PlaneCellHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _PlaneCellAngle
       (sys::poke _PlaneCellAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _PlaneCellHeight
     (setq _PlaneCellHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _PlaneCellAngle
     (setq _PlaneCellAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fitplane::Plane :md5sum-) "92467c111abb460b6d16e58287e7864b")
(setf (get fitplane::Plane :datatype-) "fitplane/Plane")
(setf (get fitplane::Plane :definition-)
      "float32 PlaneCellHeight
float32 PlaneCellAngle

")



(provide :fitplane/Plane "92467c111abb460b6d16e58287e7864b")


