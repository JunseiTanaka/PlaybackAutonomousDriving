
(cl:in-package :asdf)

(defsystem "ros_whill-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :ros_whill-msg
)
  :components ((:file "_package")
    (:file "SetSpeedProfile" :depends-on ("_package_SetSpeedProfile"))
    (:file "_package_SetSpeedProfile" :depends-on ("_package"))
  ))