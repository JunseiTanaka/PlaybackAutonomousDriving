
(cl:in-package :asdf)

(defsystem "ros_whill-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SpeedPack" :depends-on ("_package_SpeedPack"))
    (:file "_package_SpeedPack" :depends-on ("_package"))
    (:file "SpeedPack" :depends-on ("_package_SpeedPack"))
    (:file "_package_SpeedPack" :depends-on ("_package"))
  ))