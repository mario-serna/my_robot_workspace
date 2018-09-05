; Auto-generated. Do not edit!


(cl:in-package bug_algorithms-srv)


;//! \htmlinclude bugService-request.msg.html

(cl:defclass <bugService-request> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:integer
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (initial_x
    :reader initial_x
    :initarg :initial_x
    :type cl:float
    :initform 0.0)
   (initial_y
    :reader initial_y
    :initarg :initial_y
    :type cl:float
    :initform 0.0)
   (desired_x
    :reader desired_x
    :initarg :desired_x
    :type cl:float
    :initform 0.0)
   (desired_y
    :reader desired_y
    :initarg :desired_y
    :type cl:float
    :initform 0.0)
   (simulation
    :reader simulation
    :initarg :simulation
    :type cl:boolean
    :initform cl:nil)
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

(cl:defclass bugService-request (<bugService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bugService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bugService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-srv:<bugService-request> is deprecated: use bug_algorithms-srv:bugService-request instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:algorithm-val is deprecated.  Use bug_algorithms-srv:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:velocity-val is deprecated.  Use bug_algorithms-srv:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'initial_x-val :lambda-list '(m))
(cl:defmethod initial_x-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:initial_x-val is deprecated.  Use bug_algorithms-srv:initial_x instead.")
  (initial_x m))

(cl:ensure-generic-function 'initial_y-val :lambda-list '(m))
(cl:defmethod initial_y-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:initial_y-val is deprecated.  Use bug_algorithms-srv:initial_y instead.")
  (initial_y m))

(cl:ensure-generic-function 'desired_x-val :lambda-list '(m))
(cl:defmethod desired_x-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:desired_x-val is deprecated.  Use bug_algorithms-srv:desired_x instead.")
  (desired_x m))

(cl:ensure-generic-function 'desired_y-val :lambda-list '(m))
(cl:defmethod desired_y-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:desired_y-val is deprecated.  Use bug_algorithms-srv:desired_y instead.")
  (desired_y m))

(cl:ensure-generic-function 'simulation-val :lambda-list '(m))
(cl:defmethod simulation-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:simulation-val is deprecated.  Use bug_algorithms-srv:simulation instead.")
  (simulation m))

(cl:ensure-generic-function 'reverse-val :lambda-list '(m))
(cl:defmethod reverse-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:reverse-val is deprecated.  Use bug_algorithms-srv:reverse instead.")
  (reverse m))

(cl:ensure-generic-function 'choose-val :lambda-list '(m))
(cl:defmethod choose-val ((m <bugService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:choose-val is deprecated.  Use bug_algorithms-srv:choose instead.")
  (choose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bugService-request>) ostream)
  "Serializes a message object of type '<bugService-request>"
  (cl:let* ((signed (cl:slot-value msg 'algorithm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desired_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desired_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'simulation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reverse) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'choose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bugService-request>) istream)
  "Deserializes a message object of type '<bugService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'algorithm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desired_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desired_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'simulation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reverse) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'choose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bugService-request>)))
  "Returns string type for a service object of type '<bugService-request>"
  "bug_algorithms/bugServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bugService-request)))
  "Returns string type for a service object of type 'bugService-request"
  "bug_algorithms/bugServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bugService-request>)))
  "Returns md5sum for a message object of type '<bugService-request>"
  "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bugService-request)))
  "Returns md5sum for a message object of type 'bugService-request"
  "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bugService-request>)))
  "Returns full string definition for message of type '<bugService-request>"
  (cl:format cl:nil "int32 algorithm~%float32 velocity~%float32 initial_x~%float32 initial_y~%float32 desired_x~%float32 desired_y~%bool simulation~%bool reverse~%bool choose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bugService-request)))
  "Returns full string definition for message of type 'bugService-request"
  (cl:format cl:nil "int32 algorithm~%float32 velocity~%float32 initial_x~%float32 initial_y~%float32 desired_x~%float32 desired_y~%bool simulation~%bool reverse~%bool choose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bugService-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bugService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'bugService-request
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':initial_x (initial_x msg))
    (cl:cons ':initial_y (initial_y msg))
    (cl:cons ':desired_x (desired_x msg))
    (cl:cons ':desired_y (desired_y msg))
    (cl:cons ':simulation (simulation msg))
    (cl:cons ':reverse (reverse msg))
    (cl:cons ':choose (choose msg))
))
;//! \htmlinclude bugService-response.msg.html

(cl:defclass <bugService-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass bugService-response (<bugService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bugService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bugService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-srv:<bugService-response> is deprecated: use bug_algorithms-srv:bugService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <bugService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:success-val is deprecated.  Use bug_algorithms-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <bugService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-srv:message-val is deprecated.  Use bug_algorithms-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bugService-response>) ostream)
  "Serializes a message object of type '<bugService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bugService-response>) istream)
  "Deserializes a message object of type '<bugService-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bugService-response>)))
  "Returns string type for a service object of type '<bugService-response>"
  "bug_algorithms/bugServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bugService-response)))
  "Returns string type for a service object of type 'bugService-response"
  "bug_algorithms/bugServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bugService-response>)))
  "Returns md5sum for a message object of type '<bugService-response>"
  "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bugService-response)))
  "Returns md5sum for a message object of type 'bugService-response"
  "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bugService-response>)))
  "Returns full string definition for message of type '<bugService-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bugService-response)))
  "Returns full string definition for message of type 'bugService-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bugService-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bugService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'bugService-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'bugService)))
  'bugService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'bugService)))
  'bugService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bugService)))
  "Returns string type for a service object of type '<bugService>"
  "bug_algorithms/bugService")