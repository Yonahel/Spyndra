; Auto-generated. Do not edit!


(cl:in-package spyndra-msg)


;//! \htmlinclude BeaconPos.msg.html

(cl:defclass <BeaconPos> (roslisp-msg-protocol:ros-message)
  ((timestamp_ms
    :reader timestamp_ms
    :initarg :timestamp_ms
    :type cl:integer
    :initform 0)
   (address
    :reader address
    :initarg :address
    :type cl:fixnum
    :initform 0)
   (x_m
    :reader x_m
    :initarg :x_m
    :type cl:float
    :initform 0.0)
   (y_m
    :reader y_m
    :initarg :y_m
    :type cl:float
    :initform 0.0)
   (z_m
    :reader z_m
    :initarg :z_m
    :type cl:float
    :initform 0.0))
)

(cl:defclass BeaconPos (<BeaconPos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BeaconPos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BeaconPos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spyndra-msg:<BeaconPos> is deprecated: use spyndra-msg:BeaconPos instead.")))

(cl:ensure-generic-function 'timestamp_ms-val :lambda-list '(m))
(cl:defmethod timestamp_ms-val ((m <BeaconPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:timestamp_ms-val is deprecated.  Use spyndra-msg:timestamp_ms instead.")
  (timestamp_ms m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <BeaconPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:address-val is deprecated.  Use spyndra-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'x_m-val :lambda-list '(m))
(cl:defmethod x_m-val ((m <BeaconPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:x_m-val is deprecated.  Use spyndra-msg:x_m instead.")
  (x_m m))

(cl:ensure-generic-function 'y_m-val :lambda-list '(m))
(cl:defmethod y_m-val ((m <BeaconPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:y_m-val is deprecated.  Use spyndra-msg:y_m instead.")
  (y_m m))

(cl:ensure-generic-function 'z_m-val :lambda-list '(m))
(cl:defmethod z_m-val ((m <BeaconPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spyndra-msg:z_m-val is deprecated.  Use spyndra-msg:z_m instead.")
  (z_m m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BeaconPos>) ostream)
  "Serializes a message object of type '<BeaconPos>"
  (cl:let* ((signed (cl:slot-value msg 'timestamp_ms)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BeaconPos>) istream)
  "Deserializes a message object of type '<BeaconPos>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp_ms) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_m) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_m) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_m) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BeaconPos>)))
  "Returns string type for a message object of type '<BeaconPos>"
  "spyndra/BeaconPos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BeaconPos)))
  "Returns string type for a message object of type 'BeaconPos"
  "spyndra/BeaconPos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BeaconPos>)))
  "Returns md5sum for a message object of type '<BeaconPos>"
  "c67a5a42d752c8877f71d8ea372e1063")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BeaconPos)))
  "Returns md5sum for a message object of type 'BeaconPos"
  "c67a5a42d752c8877f71d8ea372e1063")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BeaconPos>)))
  "Returns full string definition for message of type '<BeaconPos>"
  (cl:format cl:nil "int64 timestamp_ms~%uint8 address~%float64 x_m~%float64 y_m~%float64 z_m~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BeaconPos)))
  "Returns full string definition for message of type 'BeaconPos"
  (cl:format cl:nil "int64 timestamp_ms~%uint8 address~%float64 x_m~%float64 y_m~%float64 z_m~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BeaconPos>))
  (cl:+ 0
     8
     1
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BeaconPos>))
  "Converts a ROS message object to a list"
  (cl:list 'BeaconPos
    (cl:cons ':timestamp_ms (timestamp_ms msg))
    (cl:cons ':address (address msg))
    (cl:cons ':x_m (x_m msg))
    (cl:cons ':y_m (y_m msg))
    (cl:cons ':z_m (z_m msg))
))
