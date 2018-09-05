; Auto-generated. Do not edit!


(cl:in-package bug_algorithms-srv)


;//! \htmlinclude bugSwitch-request.msg.html

(cl:defclass <bugSwitch-request> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:integer
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0))
)

(cl:defclass bugSwitch-request (<bugSwitch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bugSwitch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bugSwitch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-srv:<bugSwitch-request> is deprecated: use bug_algorithms-srv:bugSwitch-request instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <bugSwitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:algorithm-val is deprecated.  Use bug_algorithms-srv:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <bugSwitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:state-val is deprecated.  Use bug_algorithms-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bugSwitch-request>) ostream)
  "Serializes a message object of type '<bugSwitch-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bugSwitch-request>) istream)
  "Deserializes a message object of type '<bugSwitch-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bugSwitch-request>)))
  "Returns string type for a service object of type '<bugSwitch-request>"
  "bug_algorithms/bugSwitchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bugSwitch-request)))
  "Returns string type for a service object of type 'bugSwitch-request"
  "bug_algorithms/bugSwitchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bugSwitch-request>)))
  "Returns md5sum for a message object of type '<bugSwitch-request>"
  "06a69ed785453bce53ed290be7818f28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bugSwitch-request)))
  "Returns md5sum for a message object of type 'bugSwitch-request"
  "06a69ed785453bce53ed290be7818f28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bugSwitch-request>)))
  "Returns full string definition for message of type '<bugSwitch-request>"
  (cl:format cl:nil "int32 algorithm~%int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bugSwitch-request)))
  "Returns full string definition for message of type 'bugSwitch-request"
  (cl:format cl:nil "int32 algorithm~%int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bugSwitch-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bugSwitch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'bugSwitch-request
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':state (state msg))
))
;//! \htmlinclude bugSwitch-response.msg.html

(cl:defclass <bugSwitch-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass bugSwitch-response (<bugSwitch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bugSwitch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bugSwitch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-srv:<bugSwitch-response> is deprecated: use bug_algorithms-srv:bugSwitch-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <bugSwitch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:success-val is deprecated.  Use bug_algorithms-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <bugSwitch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:message-val is deprecated.  Use bug_algorithms-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bugSwitch-response>) ostream)
  "Serializes a message object of type '<bugSwitch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bugSwitch-response>) istream)
  "Deserializes a message object of type '<bugSwitch-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bugSwitch-response>)))
  "Returns string type for a service object of type '<bugSwitch-response>"
  "bug_algorithms/bugSwitchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bugSwitch-response)))
  "Returns string type for a service object of type 'bugSwitch-response"
  "bug_algorithms/bugSwitchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bugSwitch-response>)))
  "Returns md5sum for a message object of type '<bugSwitch-response>"
  "06a69ed785453bce53ed290be7818f28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bugSwitch-response)))
  "Returns md5sum for a message object of type 'bugSwitch-response"
  "06a69ed785453bce53ed290be7818f28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bugSwitch-response>)))
  "Returns full string definition for message of type '<bugSwitch-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bugSwitch-response)))
  "Returns full string definition for message of type 'bugSwitch-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bugSwitch-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bugSwitch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'bugSwitch-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'bugSwitch)))
  'bugSwitch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'bugSwitch)))
  'bugSwitch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bugSwitch)))
  "Returns string type for a service object of type '<bugSwitch>"
  "bug_algorithms/bugSwitch")