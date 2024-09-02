;; Auto-generated. Do not edit!


(when (boundp 'ros_whill::SetSpeedProfile)
  (if (not (find-package "ROS_WHILL"))
    (make-package "ROS_WHILL"))
  (shadow 'SetSpeedProfile (find-package "ROS_WHILL")))
(unless (find-package "ROS_WHILL::SETSPEEDPROFILE")
  (make-package "ROS_WHILL::SETSPEEDPROFILE"))
(unless (find-package "ROS_WHILL::SETSPEEDPROFILEREQUEST")
  (make-package "ROS_WHILL::SETSPEEDPROFILEREQUEST"))
(unless (find-package "ROS_WHILL::SETSPEEDPROFILERESPONSE")
  (make-package "ROS_WHILL::SETSPEEDPROFILERESPONSE"))

(in-package "ROS")





(defclass ros_whill::SetSpeedProfileRequest
  :super ros::object
  :slots (_forward _backward _turn ))

(defmethod ros_whill::SetSpeedProfileRequest
  (:init
   (&key
    ((:forward __forward) (instance ros_whill::SpeedPack :init))
    ((:backward __backward) (instance ros_whill::SpeedPack :init))
    ((:turn __turn) (instance ros_whill::SpeedPack :init))
    )
   (send-super :init)
   (setq _forward __forward)
   (setq _backward __backward)
   (setq _turn __turn)
   self)
  (:forward
   (&rest __forward)
   (if (keywordp (car __forward))
       (send* _forward __forward)
     (progn
       (if __forward (setq _forward (car __forward)))
       _forward)))
  (:backward
   (&rest __backward)
   (if (keywordp (car __backward))
       (send* _backward __backward)
     (progn
       (if __backward (setq _backward (car __backward)))
       _backward)))
  (:turn
   (&rest __turn)
   (if (keywordp (car __turn))
       (send* _turn __turn)
     (progn
       (if __turn (setq _turn (car __turn)))
       _turn)))
  (:serialization-length
   ()
   (+
    ;; ros_whill/SpeedPack _forward
    (send _forward :serialization-length)
    ;; ros_whill/SpeedPack _backward
    (send _backward :serialization-length)
    ;; ros_whill/SpeedPack _turn
    (send _turn :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ros_whill/SpeedPack _forward
       (send _forward :serialize s)
     ;; ros_whill/SpeedPack _backward
       (send _backward :serialize s)
     ;; ros_whill/SpeedPack _turn
       (send _turn :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ros_whill/SpeedPack _forward
     (send _forward :deserialize buf ptr-) (incf ptr- (send _forward :serialization-length))
   ;; ros_whill/SpeedPack _backward
     (send _backward :deserialize buf ptr-) (incf ptr- (send _backward :serialization-length))
   ;; ros_whill/SpeedPack _turn
     (send _turn :deserialize buf ptr-) (incf ptr- (send _turn :serialization-length))
   ;;
   self)
  )

(defclass ros_whill::SetSpeedProfileResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod ros_whill::SetSpeedProfileResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ros_whill::SetSpeedProfile
  :super ros::object
  :slots ())

(setf (get ros_whill::SetSpeedProfile :md5sum-) "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(setf (get ros_whill::SetSpeedProfile :datatype-) "ros_whill/SetSpeedProfile")
(setf (get ros_whill::SetSpeedProfile :request) ros_whill::SetSpeedProfileRequest)
(setf (get ros_whill::SetSpeedProfile :response) ros_whill::SetSpeedProfileResponse)

(defmethod ros_whill::SetSpeedProfileRequest
  (:response () (instance ros_whill::SetSpeedProfileResponse :init)))

(setf (get ros_whill::SetSpeedProfileRequest :md5sum-) "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(setf (get ros_whill::SetSpeedProfileRequest :datatype-) "ros_whill/SetSpeedProfileRequest")
(setf (get ros_whill::SetSpeedProfileRequest :definition-)
      "ros_whill/SpeedPack forward      # m/s(Speed) and m/ss(Acceleration/Deceleration)
ros_whill/SpeedPack backward     # m/s(Speed) and m/ss(Acceleration/Deceleration)
ros_whill/SpeedPack turn         # rad/s(Speed) and rad/ss(Acceleration/Deceleration)


================================================================================
MSG: ros_whill/SpeedPack
float32 speed  # m/s   or rad/s
float32 acc    # m/ss  or rad/ss
---
bool success
string status_message

")

(setf (get ros_whill::SetSpeedProfileResponse :md5sum-) "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(setf (get ros_whill::SetSpeedProfileResponse :datatype-) "ros_whill/SetSpeedProfileResponse")
(setf (get ros_whill::SetSpeedProfileResponse :definition-)
      "ros_whill/SpeedPack forward      # m/s(Speed) and m/ss(Acceleration/Deceleration)
ros_whill/SpeedPack backward     # m/s(Speed) and m/ss(Acceleration/Deceleration)
ros_whill/SpeedPack turn         # rad/s(Speed) and rad/ss(Acceleration/Deceleration)


================================================================================
MSG: ros_whill/SpeedPack
float32 speed  # m/s   or rad/s
float32 acc    # m/ss  or rad/ss
---
bool success
string status_message

")



(provide :ros_whill/SetSpeedProfile "7edfb7c10ea7c5bdda72a3f1fbcf0705")


