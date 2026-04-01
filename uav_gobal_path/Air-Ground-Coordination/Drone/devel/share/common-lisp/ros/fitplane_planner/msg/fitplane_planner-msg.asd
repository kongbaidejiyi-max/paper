
(cl:in-package :asdf)

(defsystem "fitplane_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GlobalPath" :depends-on ("_package_GlobalPath"))
    (:file "_package_GlobalPath" :depends-on ("_package"))
    (:file "GridPlaneInfo" :depends-on ("_package_GridPlaneInfo"))
    (:file "_package_GridPlaneInfo" :depends-on ("_package"))
  ))