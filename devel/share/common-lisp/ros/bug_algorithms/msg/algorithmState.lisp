; Auto-generated. Do not edit!


(cl:in-package bug_algorithms-msg)


;//! \htmlinclude algorithmState.msg.html

(cl:defclass <algorithmState> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (pose_x
    :reader pose_x
    :initarg :pose_x
    :type cl:float
    :initform 0.0)
   (pose_y
    :reader pose_y
    :initarg :pose_y
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (initial_to_goal_distance
    :reader initial_to_goal_distance
    :initarg :initial_to_goal_distance
    :type cl:float
    :initform 0.0)
   (current_to_goal_distance
    :reader current_to_goal_distance
    :initarg :current_to_goal_distance
    :type cl:float
    :initform 0.0)
   (best_distance
    :reader best_distance
    :initarg :best_distance
    :type cl:float
    :initform 0.0)
   (path_length
    :reader path_length
    :initarg :path_length
    :type cl:float
    :initform 0.0))
)

(cl:defclass algorithmState (<algorithmState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <algorithmState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'algorithmState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bug_algorithms-msg:<algorithmState> is deprecated: use bug_algorithms-msg:algorithmState instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:algorithm-val is deprecated.  Use bug_algorithms-msg:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:name-val is deprecated.  Use bug_algorithms-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'pose_x-val :lambda-list '(m))
(cl:defmethod pose_x-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:pose_x-val is deprecated.  Use bug_algorithms-msg:pose_x instead.")
  (pose_x m))

(cl:ensure-generic-function 'pose_y-val :lambda-list '(m))
(cl:defmethod pose_y-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:pose_y-val is deprecated.  Use bug_algorithms-msg:pose_y instead.")
  (pose_y m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:yaw-val is deprecated.  Use bug_algorithms-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'initial_to_goal_distance-val :lambda-list '(m))
(cl:defmethod initial_to_goal_distance-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:initial_to_goal_distance-val is deprecated.  Use bug_algorithms-msg:initial_to_goal_distance instead.")
  (initial_to_goal_distance m))

(cl:ensure-generic-function 'current_to_goal_distance-val :lambda-list '(m))
(cl:defmethod current_to_goal_distance-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:current_to_goal_distance-val is deprecated.  Use bug_algorithms-msg:current_to_goal_distance instead.")
  (current_to_goal_distance m))

(cl:ensure-generic-function 'best_distance-val :lambda-list '(m))
(cl:defmethod best_distance-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:best_distance-val is deprecated.  Use bug_algorithms-msg:best_distance instead.")
  (best_distance m))

(cl:ensure-generic-function 'path_length-val :lambda-list '(m))
(cl:defmethod path_length-val ((m <algorithmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bug_algorithms-msg:path_length-val is deprecated.  Use bug_algorithms-msg:path_length instead.")
  (path_length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <algorithmState>) ostream)
  "Serializes a message object of type '<algorithmState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'algorithm)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pose_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pose_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_to_goal_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_to_goal_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'best_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'path_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <algorithmState>) istream)
  "Deserializes a message object of type '<algorithmState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'algorithm)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pose_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pose_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_to_goal_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_to_goal_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'best_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'path_length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<algorithmState>)))
  "Returns string type for a message object of type '<algorithmState>"
  "bug_algorithms/algorithmState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'algorithmState)))
  "Returns string type for a message object of type 'algorithmState"
  "bug_algorithms/algorithmState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<algorithmState>)))
  "Returns md5sum for a message object of type '<algorithmState>"
  "eed00cdf20aebf4db1b70bfc80462703")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'algorithmState)))
  "Returns md5sum for a message object of type 'algorithmState"
  "eed00cdf20aebf4db1b70bfc80462703")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<algorithmState>)))
  "Returns full string definition for message of type '<algorithmState>"
  (cl:format cl:nil "uint8 algorithm~%string name~%float32 pose_x~%float32 pose_y~%float32 yaw~%float32 initial_to_goal_distance~%float32 current_to_goal_distance~%float32 best_distance~%float32 path_length~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'algorithmState)))
  "Returns full string definition for message of type 'algorithmState"
  (cl:format cl:nil "uint8 algorithm~%string name~%float32 pose_x~%float32 pose_y~%float32 yaw~%float32 initial_to_goal_distance~%float32 current_to_goal_distance~%float32 best_distance~%float32 path_length~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <algorithmState>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'name))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <algorithmState>))
  "Converts a ROS message object to a list"
  (cl:list 'algorithmState
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':name (name msg))
    (cl:cons ':pose_x (pose_x msg))
    (cl:cons ':pose_y (pose_y msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':initial_to_goal_distance (initial_to_goal_distance msg))
    (cl:cons ':current_to_goal_distance (current_to_goal_distance msg))
    (cl:cons ':best_distance (best_distance msg))
    (cl:cons ':path_length (path_length msg))
))
