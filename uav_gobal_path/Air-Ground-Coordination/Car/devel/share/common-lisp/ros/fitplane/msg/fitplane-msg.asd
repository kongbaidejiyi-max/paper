
(cl:in-package :asdf)

(defsystem "fitplane-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GridPlaneInfo" :depends-on ("_package_GridPlaneInfo"))
    (:file "_package_GridPlaneInfo" :depends-on ("_package"))
    (:file "Plane" :depends-on ("_package_Plane"))
    (:file "_package_Plane" :depends-on ("_package"))
    (:file "PlaneMap" :depends-on ("_package_PlaneMap"))
    (:file "_package_PlaneMap" :depends-on ("_package"))
  ))