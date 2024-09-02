; Auto-generated. Do not edit!


(cl:in-package ros_whill-msg)


;//! \htmlinclude SpeedPack.msg.html

(cl:defclass <SpeedPack> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (acc
    :reader acc
    :initarg :acc
    :type cl:float
    :initform 0.0)
   (dec
    :reader dec
    :initarg :dec
    :type cl:float
    :initform 0.0))
)

(cl:defclass SpeedPack (<SpeedPack>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeedPack>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeedPack)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_whill-msg:<SpeedPack> is deprecated: use ros_whill-msg:SpeedPack instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <SpeedPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-msg:speed-val is deprecated.  Use ros_whill-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <SpeedPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-msg:acc-val is deprecated.  Use ros_whill-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'dec-val :lambda-list '(m))
(cl:defmethod dec-val ((m <SpeedPack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-msg:dec-val is deprecated.  Use ros_whill-msg:dec instead.")
  (dec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeedPack>) ostream)
  "Serializes a message object of type '<SpeedPack>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeedPack>) istream)
  "Deserializes a message object of type '<SpeedPack>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeedPack>)))
  "Returns string type for a message object of type '<SpeedPack>"
  "ros_whill/SpeedPack")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeedPack)))
  "Returns string type for a message object of type 'SpeedPack"
  "ros_whill/SpeedPack")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeedPack>)))
  "Returns md5sum for a message object of type '<SpeedPack>"
  "0a1fd5b06ddb3a692b56f449ac43917c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeedPack)))
  "Returns md5sum for a message object of type 'SpeedPack"
  "0a1fd5b06ddb3a692b56f449ac43917c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeedPack>)))
  "Returns full string definition for message of type '<SpeedPack>"
  (cl:format cl:nil "float32 speed  # m/s   or rad/s~%float32 acc    # m/ss  or rad/ss~%float32 dec    # m/ss  or rad/ss~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeedPack)))
  "Returns full string definition for message of type 'SpeedPack"
  (cl:format cl:nil "float32 speed  # m/s   or rad/s~%float32 acc    # m/ss  or rad/ss~%float32 dec    # m/ss  or rad/ss~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeedPack>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeedPack>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeedPack
    (cl:cons ':speed (speed msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':dec (dec msg))
))
