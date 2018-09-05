; Auto-generated. Do not edit!


(cl:in-package bug_algorithms-msg)


;//! \htmlinclude nodeState.msg.html

(cl:defclass <nodeState> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:fixnum
    :initform 0)
   (node_state
    :reader node_state
    :initarg :node_state
    :type cl:fixnum
    :initform 0)
   (node_state_desc
    :reader node_state_desc
    :initarg :node_state_desc
    :type cl:string
    :initform "")
   (node_state_time
    :reader node_state_time
    :initarg :node_state_time
    :type cl:float
    :initform 0.0)
   (bug_state
    :reader bug_state
    :initarg :bug_state
    :type cl:fixnum
    :initform 0)
   (bug_state_desc
    :reader bug_state_desc
    :initarg :bug_state_desc
    :type cl:string
    :initform "")
   (bug_state_time
    :reader bug_state_time
    :initarg :bug_state_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass nodeState (<nodeState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nodeState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nodeState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-msg:<nodeState> is deprecated: use bug_algorithms-msg:nodeState instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:algorithm-val is deprecated.  Use bug_algorithms-msg:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'node_state-val :lambda-list '(m))
(cl:defmethod node_state-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:node_state-val is deprecated.  Use bug_algorithms-msg:node_state instead.")
  (node_state m))

(cl:ensure-generic-function 'node_state_desc-val :lambda-list '(m))
(cl:defmethod node_state_desc-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:node_state_desc-val is deprecated.  Use bug_algorithms-msg:node_state_desc instead.")
  (node_state_desc m))

(cl:ensure-generic-function 'node_state_time-val :lambda-list '(m))
(cl:defmethod node_state_time-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:node_state_time-val is deprecated.  Use bug_algorithms-msg:node_state_time instead.")
  (node_state_time m))

(cl:ensure-generic-function 'bug_state-val :lambda-list '(m))
(cl:defmethod bug_state-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:bug_state-val is deprecated.  Use bug_algorithms-msg:bug_state instead.")
  (bug_state m))

(cl:ensure-generic-function 'bug_state_desc-val :lambda-list '(m))
(cl:defmethod bug_state_desc-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:bug_state_desc-val is deprecated.  Use bug_algorithms-msg:bug_state_desc instead.")
  (bug_state_desc m))

(cl:ensure-generic-function 'bug_state_time-val :lambda-list '(m))
(cl:defmethod bug_state_time-val ((m <nodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:bug_state_time-val is deprecated.  Use bug_algorithms-msg:bug_state_time instead.")
  (bug_state_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nodeState>) ostream)
  "Serializes a message object of type '<nodeState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'algorithm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node_state_desc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node_state_desc))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'node_state_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bug_state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bug_state_desc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bug_state_desc))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bug_state_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nodeState>) istream)
  "Deserializes a message object of type '<nodeState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'algorithm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node_state_desc) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'node_state_desc) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'node_state_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bug_state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bug_state_desc) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bug_state_desc) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bug_state_time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nodeState>)))
  "Returns string type for a message object of type '<nodeState>"
  "bug_algorithms/nodeState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nodeState)))
  "Returns string type for a message object of type 'nodeState"
  "bug_algorithms/nodeState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nodeState>)))
  "Returns md5sum for a message object of type '<nodeState>"
  "eacf4f1a4f8ef654fd25492c527c277f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nodeState)))
  "Returns md5sum for a message object of type 'nodeState"
  "eacf4f1a4f8ef654fd25492c527c277f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nodeState>)))
  "Returns full string definition for message of type '<nodeState>"
  (cl:format cl:nil "uint8 algorithm~%uint8 node_state~%string node_state_desc~%float32 node_state_time~%uint8 bug_state~%string bug_state_desc~%float32 bug_state_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nodeState)))
  "Returns full string definition for message of type 'nodeState"
  (cl:format cl:nil "uint8 algorithm~%uint8 node_state~%string node_state_desc~%float32 node_state_time~%uint8 bug_state~%string bug_state_desc~%float32 bug_state_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nodeState>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'node_state_desc))
     4
     1
     4 (cl:length (cl:slot-value msg 'bug_state_desc))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nodeState>))
  "Converts a ROS message object to a list"
  (cl:list 'nodeState
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':node_state (node_state msg))
    (cl:cons ':node_state_desc (node_state_desc msg))
    (cl:cons ':node_state_time (node_state_time msg))
    (cl:cons ':bug_state (bug_state msg))
    (cl:cons ':bug_state_desc (bug_state_desc msg))
    (cl:cons ':bug_state_time (bug_state_time msg))
))
