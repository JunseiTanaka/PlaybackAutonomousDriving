; Auto-generated. Do not edit!


(cl:in-package ros_whill-srv)


;//! \htmlinclude SetSpeedProfile-request.msg.html

(cl:defclass <SetSpeedProfile-request> (roslisp-msg-protocol:ros-message)
  ((forward
    :reader forward
    :initarg :forward
    :type ros_whill-msg:SpeedPack
    :initform (cl:make-instance 'ros_whill-msg:SpeedPack))
   (backward
    :reader backward
    :initarg :backward
    :type ros_whill-msg:SpeedPack
    :initform (cl:make-instance 'ros_whill-msg:SpeedPack))
   (turn
    :reader turn
    :initarg :turn
    :type ros_whill-msg:SpeedPack
    :initform (cl:make-instance 'ros_whill-msg:SpeedPack)))
)

(cl:defclass SetSpeedProfile-request (<SetSpeedProfile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeedProfile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeedProfile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_whill-srv:<SetSpeedProfile-request> is deprecated: use ros_whill-srv:SetSpeedProfile-request instead.")))

(cl:ensure-generic-function 'forward-val :lambda-list '(m))
(cl:defmethod forward-val ((m <SetSpeedProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-srv:forward-val is deprecated.  Use ros_whill-srv:forward instead.")
  (forward m))

(cl:ensure-generic-function 'backward-val :lambda-list '(m))
(cl:defmethod backward-val ((m <SetSpeedProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-srv:backward-val is deprecated.  Use ros_whill-srv:backward instead.")
  (backward m))

(cl:ensure-generic-function 'turn-val :lambda-list '(m))
(cl:defmethod turn-val ((m <SetSpeedProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-srv:turn-val is deprecated.  Use ros_whill-srv:turn instead.")
  (turn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeedProfile-request>) ostream)
  "Serializes a message object of type '<SetSpeedProfile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'forward) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'backward) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'turn) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeedProfile-request>) istream)
  "Deserializes a message object of type '<SetSpeedProfile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'forward) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'backward) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'turn) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeedProfile-request>)))
  "Returns string type for a service object of type '<SetSpeedProfile-request>"
  "ros_whill/SetSpeedProfileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedProfile-request)))
  "Returns string type for a service object of type 'SetSpeedProfile-request"
  "ros_whill/SetSpeedProfileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeedProfile-request>)))
  "Returns md5sum for a message object of type '<SetSpeedProfile-request>"
  "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeedProfile-request)))
  "Returns md5sum for a message object of type 'SetSpeedProfile-request"
  "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeedProfile-request>)))
  "Returns full string definition for message of type '<SetSpeedProfile-request>"
  (cl:format cl:nil "ros_whill/SpeedPack forward      # m/s(Speed) and m/ss(Acceleration/Deceleration)~%ros_whill/SpeedPack backward     # m/s(Speed) and m/ss(Acceleration/Deceleration)~%ros_whill/SpeedPack turn         # rad/s(Speed) and rad/ss(Acceleration/Deceleration)~%~%~%================================================================================~%MSG: ros_whill/SpeedPack~%float32 speed  # m/s   or rad/s~%float32 acc    # m/ss  or rad/ss~%float32 dec    # m/ss  or rad/ss~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeedProfile-request)))
  "Returns full string definition for message of type 'SetSpeedProfile-request"
  (cl:format cl:nil "ros_whill/SpeedPack forward      # m/s(Speed) and m/ss(Acceleration/Deceleration)~%ros_whill/SpeedPack backward     # m/s(Speed) and m/ss(Acceleration/Deceleration)~%ros_whill/SpeedPack turn         # rad/s(Speed) and rad/ss(Acceleration/Deceleration)~%~%~%================================================================================~%MSG: ros_whill/SpeedPack~%float32 speed  # m/s   or rad/s~%float32 acc    # m/ss  or rad/ss~%float32 dec    # m/ss  or rad/ss~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeedProfile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'forward))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'backward))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'turn))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeedProfile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeedProfile-request
    (cl:cons ':forward (forward msg))
    (cl:cons ':backward (backward msg))
    (cl:cons ':turn (turn msg))
))
;//! \htmlinclude SetSpeedProfile-response.msg.html

(cl:defclass <SetSpeedProfile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status_message
    :reader status_message
    :initarg :status_message
    :type cl:string
    :initform ""))
)

(cl:defclass SetSpeedProfile-response (<SetSpeedProfile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeedProfile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeedProfile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_whill-srv:<SetSpeedProfile-response> is deprecated: use ros_whill-srv:SetSpeedProfile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetSpeedProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-srv:success-val is deprecated.  Use ros_whill-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetSpeedProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_whill-srv:status_message-val is deprecated.  Use ros_whill-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeedProfile-response>) ostream)
  "Serializes a message object of type '<SetSpeedProfile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeedProfile-response>) istream)
  "Deserializes a message object of type '<SetSpeedProfile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeedProfile-response>)))
  "Returns string type for a service object of type '<SetSpeedProfile-response>"
  "ros_whill/SetSpeedProfileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedProfile-response)))
  "Returns string type for a service object of type 'SetSpeedProfile-response"
  "ros_whill/SetSpeedProfileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeedProfile-response>)))
  "Returns md5sum for a message object of type '<SetSpeedProfile-response>"
  "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeedProfile-response)))
  "Returns md5sum for a message object of type 'SetSpeedProfile-response"
  "7edfb7c10ea7c5bdda72a3f1fbcf0705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeedProfile-response>)))
  "Returns full string definition for message of type '<SetSpeedProfile-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeedProfile-response)))
  "Returns full string definition for message of type 'SetSpeedProfile-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeedProfile-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeedProfile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeedProfile-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSpeedProfile)))
  'SetSpeedProfile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSpeedProfile)))
  'SetSpeedProfile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedProfile)))
  "Returns string type for a service object of type '<SetSpeedProfile>"
  "ros_whill/SetSpeedProfile")