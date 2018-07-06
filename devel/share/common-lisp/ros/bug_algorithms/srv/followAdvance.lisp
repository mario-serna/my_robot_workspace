; Auto-generated. Do not edit!


(cl:in-package bug_algorithms-srv)


;//! \htmlinclude followAdvance-request.msg.html

(cl:defclass <followAdvance-request> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:integer
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0)
   (reverse
    :reader reverse
    :initarg :reverse
    :type cl:boolean
    :initform cl:nil)
   (choose
    :reader choose
    :initarg :choose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass followAdvance-request (<followAdvance-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <followAdvance-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'followAdvance-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-srv:<followAdvance-request> is deprecated: use bug_algorithms-srv:followAdvance-request instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <followAdvance-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:algorithm-val is deprecated.  Use bug_algorithms-srv:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <followAdvance-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:state-val is deprecated.  Use bug_algorithms-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'reverse-val :lambda-list '(m))
(cl:defmethod reverse-val ((m <followAdvance-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:reverse-val is deprecated.  Use bug_algorithms-srv:reverse instead.")
  (reverse m))

(cl:ensure-generic-function 'choose-val :lambda-list '(m))
(cl:defmethod choose-val ((m <followAdvance-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:choose-val is deprecated.  Use bug_algorithms-srv:choose instead.")
  (choose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <followAdvance-request>) ostream)
  "Serializes a message object of type '<followAdvance-request>"
  (cl:let* ((signed (cl:slot-value msg 'algorithm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reverse) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'choose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <followAdvance-request>) istream)
  "Deserializes a message object of type '<followAdvance-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'algorithm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'reverse) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'choose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<followAdvance-request>)))
  "Returns string type for a service object of type '<followAdvance-request>"
  "bug_algorithms/followAdvanceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'followAdvance-request)))
  "Returns string type for a service object of type 'followAdvance-request"
  "bug_algorithms/followAdvanceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<followAdvance-request>)))
  "Returns md5sum for a message object of type '<followAdvance-request>"
  "0cdc9d6762a3b9cb2474f738c3771c80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'followAdvance-request)))
  "Returns md5sum for a message object of type 'followAdvance-request"
  "0cdc9d6762a3b9cb2474f738c3771c80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<followAdvance-request>)))
  "Returns full string definition for message of type '<followAdvance-request>"
  (cl:format cl:nil "int32 algorithm~%int32 state~%bool reverse~%bool choose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'followAdvance-request)))
  "Returns full string definition for message of type 'followAdvance-request"
  (cl:format cl:nil "int32 algorithm~%int32 state~%bool reverse~%bool choose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <followAdvance-request>))
  (cl:+ 0
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <followAdvance-request>))
  "Converts a ROS message object to a list"
  (cl:list 'followAdvance-request
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':state (state msg))
    (cl:cons ':reverse (reverse msg))
    (cl:cons ':choose (choose msg))
))
;//! \htmlinclude followAdvance-response.msg.html

(cl:defclass <followAdvance-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass followAdvance-response (<followAdvance-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <followAdvance-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'followAdvance-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-srv:<followAdvance-response> is deprecated: use bug_algorithms-srv:followAdvance-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <followAdvance-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:success-val is deprecated.  Use bug_algorithms-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <followAdvance-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:message-val is deprecated.  Use bug_algorithms-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <followAdvance-response>) ostream)
  "Serializes a message object of type '<followAdvance-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <followAdvance-response>) istream)
  "Deserializes a message object of type '<followAdvance-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<followAdvance-response>)))
  "Returns string type for a service object of type '<followAdvance-response>"
  "bug_algorithms/followAdvanceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'followAdvance-response)))
  "Returns string type for a service object of type 'followAdvance-response"
  "bug_algorithms/followAdvanceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<followAdvance-response>)))
  "Returns md5sum for a message object of type '<followAdvance-response>"
  "0cdc9d6762a3b9cb2474f738c3771c80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'followAdvance-response)))
  "Returns md5sum for a message object of type 'followAdvance-response"
  "0cdc9d6762a3b9cb2474f738c3771c80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<followAdvance-response>)))
  "Returns full string definition for message of type '<followAdvance-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'followAdvance-response)))
  "Returns full string definition for message of type 'followAdvance-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <followAdvance-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <followAdvance-response>))
  "Converts a ROS message object to a list"
  (cl:list 'followAdvance-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'followAdvance)))
  'followAdvance-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'followAdvance)))
  'followAdvance-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'followAdvance)))
  "Returns string type for a service object of type '<followAdvance>"
  "bug_algorithms/followAdvance")