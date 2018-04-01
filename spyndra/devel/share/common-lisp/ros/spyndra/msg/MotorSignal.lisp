; Auto-generated. Do not edit!


(cl:in-package spyndra-msg)


;//! \htmlinclude MotorSignal.msg.html

(cl:defclass <MotorSignal> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (motor_id
    :reader motor_id
    :initarg :motor_id
    :type cl:integer
    :initform 0)
   (signal
    :reader signal
    :initarg :signal
    :type cl:integer
    :initform 0)
   (load
    :reader load
    :initarg :load
    :type cl:integer
    :initform 0)
   (action_error
    :reader action_error
    :initarg :action_error
    :type cl:integer
    :initform 0))
)

(cl:defclass MotorSignal (<MotorSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spyndra-msg:<MotorSignal> is deprecated: use spyndra-msg:MotorSignal instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <MotorSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:speed-val is deprecated.  Use spyndra-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'motor_id-val :lambda-list '(m))
(cl:defmethod motor_id-val ((m <MotorSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:motor_id-val is deprecated.  Use spyndra-msg:motor_id instead.")
  (motor_id m))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <MotorSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:signal-val is deprecated.  Use spyndra-msg:signal instead.")
  (signal m))

(cl:ensure-generic-function 'load-val :lambda-list '(m))
(cl:defmethod load-val ((m <MotorSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:load-val is deprecated.  Use spyndra-msg:load instead.")
  (load m))

(cl:ensure-generic-function 'action_error-val :lambda-list '(m))
(cl:defmethod action_error-val ((m <MotorSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:action_error-val is deprecated.  Use spyndra-msg:action_error instead.")
  (action_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSignal>) ostream)
  "Serializes a message object of type '<MotorSignal>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'signal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'load)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'action_error)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSignal>) istream)
  "Deserializes a message object of type '<MotorSignal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'signal) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'load) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_error) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSignal>)))
  "Returns string type for a message object of type '<MotorSignal>"
  "spyndra/MotorSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSignal)))
  "Returns string type for a message object of type 'MotorSignal"
  "spyndra/MotorSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSignal>)))
  "Returns md5sum for a message object of type '<MotorSignal>"
  "baf7bfab6e1a220674cb24b5f4878d91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSignal)))
  "Returns md5sum for a message object of type 'MotorSignal"
  "baf7bfab6e1a220674cb24b5f4878d91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSignal>)))
  "Returns full string definition for message of type '<MotorSignal>"
  (cl:format cl:nil "int32 speed~%int32 motor_id~%int32 signal~%int32 load~%int32 action_error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSignal)))
  "Returns full string definition for message of type 'MotorSignal"
  (cl:format cl:nil "int32 speed~%int32 motor_id~%int32 signal~%int32 load~%int32 action_error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSignal>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSignal
    (cl:cons ':speed (speed msg))
    (cl:cons ':motor_id (motor_id msg))
    (cl:cons ':signal (signal msg))
    (cl:cons ':load (load msg))
    (cl:cons ':action_error (action_error msg))
))
