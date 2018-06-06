; Auto-generated. Do not edit!


(cl:in-package ariaClientDriver-msg)


;//! \htmlinclude AriaCommandData.msg.html

(cl:defclass <AriaCommandData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (TransRatio
    :reader TransRatio
    :initarg :TransRatio
    :type cl:float
    :initform 0.0)
   (RotRatio
    :reader RotRatio
    :initarg :RotRatio
    :type cl:float
    :initform 0.0)
   (LatRatio
    :reader LatRatio
    :initarg :LatRatio
    :type cl:float
    :initform 0.0)
   (MaxVel
    :reader MaxVel
    :initarg :MaxVel
    :type cl:float
    :initform 0.0))
)

(cl:defclass AriaCommandData (<AriaCommandData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AriaCommandData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AriaCommandData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ariaClientDriver-msg:<AriaCommandData> is deprecated: use ariaClientDriver-msg:AriaCommandData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AriaCommandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ariaClientDriver-msg:header-val is deprecated.  Use ariaClientDriver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'TransRatio-val :lambda-list '(m))
(cl:defmethod TransRatio-val ((m <AriaCommandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ariaClientDriver-msg:TransRatio-val is deprecated.  Use ariaClientDriver-msg:TransRatio instead.")
  (TransRatio m))

(cl:ensure-generic-function 'RotRatio-val :lambda-list '(m))
(cl:defmethod RotRatio-val ((m <AriaCommandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ariaClientDriver-msg:RotRatio-val is deprecated.  Use ariaClientDriver-msg:RotRatio instead.")
  (RotRatio m))

(cl:ensure-generic-function 'LatRatio-val :lambda-list '(m))
(cl:defmethod LatRatio-val ((m <AriaCommandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ariaClientDriver-msg:LatRatio-val is deprecated.  Use ariaClientDriver-msg:LatRatio instead.")
  (LatRatio m))

(cl:ensure-generic-function 'MaxVel-val :lambda-list '(m))
(cl:defmethod MaxVel-val ((m <AriaCommandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ariaClientDriver-msg:MaxVel-val is deprecated.  Use ariaClientDriver-msg:MaxVel instead.")
  (MaxVel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AriaCommandData>) ostream)
  "Serializes a message object of type '<AriaCommandData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'TransRatio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RotRatio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LatRatio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'MaxVel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AriaCommandData>) istream)
  "Deserializes a message object of type '<AriaCommandData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'TransRatio) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RotRatio) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LatRatio) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'MaxVel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AriaCommandData>)))
  "Returns string type for a message object of type '<AriaCommandData>"
  "ariaClientDriver/AriaCommandData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AriaCommandData)))
  "Returns string type for a message object of type 'AriaCommandData"
  "ariaClientDriver/AriaCommandData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AriaCommandData>)))
  "Returns md5sum for a message object of type '<AriaCommandData>"
  "21b14ee76100ef35375a686e47103895")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AriaCommandData)))
  "Returns md5sum for a message object of type 'AriaCommandData"
  "21b14ee76100ef35375a686e47103895")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AriaCommandData>)))
  "Returns full string definition for message of type '<AriaCommandData>"
  (cl:format cl:nil "# this packs the sequence the robot identity an the time stamp~%Header header~%~%# Navigation data of aria robots~%float64  TransRatio~%float64  RotRatio~%float64  LatRatio~%float64  MaxVel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AriaCommandData)))
  "Returns full string definition for message of type 'AriaCommandData"
  (cl:format cl:nil "# this packs the sequence the robot identity an the time stamp~%Header header~%~%# Navigation data of aria robots~%float64  TransRatio~%float64  RotRatio~%float64  LatRatio~%float64  MaxVel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AriaCommandData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AriaCommandData>))
  "Converts a ROS message object to a list"
  (cl:list 'AriaCommandData
    (cl:cons ':header (header msg))
    (cl:cons ':TransRatio (TransRatio msg))
    (cl:cons ':RotRatio (RotRatio msg))
    (cl:cons ':LatRatio (LatRatio msg))
    (cl:cons ':MaxVel (MaxVel msg))
))
