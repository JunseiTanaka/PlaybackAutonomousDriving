;; Auto-generated. Do not edit!


(when (boundp 'ros_whill::SpeedPack)
  (if (not (find-package "ROS_WHILL"))
    (make-package "ROS_WHILL"))
  (shadow 'SpeedPack (find-package "ROS_WHILL")))
(unless (find-package "ROS_WHILL::SPEEDPACK")
  (make-package "ROS_WHILL::SPEEDPACK"))

(in-package "ROS")
;;//! \htmlinclude SpeedPack.msg.html


(defclass ros_whill::SpeedPack
  :super ros::object
  :slots (_speed _acc _dec ))

(defmethod ros_whill::SpeedPack
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:acc __acc) 0.0)
    ((:dec __dec) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _acc (float __acc))
   (setq _dec (float __dec))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:dec
   (&optional __dec)
   (if __dec (setq _dec __dec)) _dec)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; float32 _acc
    4
    ;; float32 _dec
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dec
       (sys::poke _dec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acc
     (setq _acc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dec
     (setq _dec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_whill::SpeedPack :md5sum-) "0a1fd5b06ddb3a692b56f449ac43917c")
(setf (get ros_whill::SpeedPack :datatype-) "ros_whill/SpeedPack")
(setf (get ros_whill::SpeedPack :definition-)
      "float32 speed  # m/s   or rad/s
float32 acc    # m/ss  or rad/ss
float32 dec    # m/ss  or rad/ss
")



(provide :ros_whill/SpeedPack "0a1fd5b06ddb3a692b56f449ac43917c")


