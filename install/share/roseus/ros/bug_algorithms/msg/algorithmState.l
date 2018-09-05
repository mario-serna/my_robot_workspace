;; Auto-generated. Do not edit!


(when (boundp 'bug_algorithms::algorithmState)
  (if (not (find-package "BUG_ALGORITHMS"))
    (make-package "BUG_ALGORITHMS"))
  (shadow 'algorithmState (find-package "BUG_ALGORITHMS")))
(unless (find-package "BUG_ALGORITHMS::ALGORITHMSTATE")
  (make-package "BUG_ALGORITHMS::ALGORITHMSTATE"))

(in-package "ROS")
;;//! \htmlinclude algorithmState.msg.html


(defclass bug_algorithms::algorithmState
  :super ros::object
  :slots (_algorithm _name _pose_x _pose_y _yaw _initial_to_goal_distance _current_to_goal_distance _best_distance _path_length _time ))

(defmethod bug_algorithms::algorithmState
  (:init
   (&key
    ((:algorithm __algorithm) 0)
    ((:name __name) "")
    ((:pose_x __pose_x) 0.0)
    ((:pose_y __pose_y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:initial_to_goal_distance __initial_to_goal_distance) 0.0)
    ((:current_to_goal_distance __current_to_goal_distance) 0.0)
    ((:best_distance __best_distance) 0.0)
    ((:path_length __path_length) 0.0)
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _algorithm (round __algorithm))
   (setq _name (string __name))
   (setq _pose_x (float __pose_x))
   (setq _pose_y (float __pose_y))
   (setq _yaw (float __yaw))
   (setq _initial_to_goal_distance (float __initial_to_goal_distance))
   (setq _current_to_goal_distance (float __current_to_goal_distance))
   (setq _best_distance (float __best_distance))
   (setq _path_length (float __path_length))
   (setq _time (float __time))
   self)
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:pose_x
   (&optional __pose_x)
   (if __pose_x (setq _pose_x __pose_x)) _pose_x)
  (:pose_y
   (&optional __pose_y)
   (if __pose_y (setq _pose_y __pose_y)) _pose_y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:initial_to_goal_distance
   (&optional __initial_to_goal_distance)
   (if __initial_to_goal_distance (setq _initial_to_goal_distance __initial_to_goal_distance)) _initial_to_goal_distance)
  (:current_to_goal_distance
   (&optional __current_to_goal_distance)
   (if __current_to_goal_distance (setq _current_to_goal_distance __current_to_goal_distance)) _current_to_goal_distance)
  (:best_distance
   (&optional __best_distance)
   (if __best_distance (setq _best_distance __best_distance)) _best_distance)
  (:path_length
   (&optional __path_length)
   (if __path_length (setq _path_length __path_length)) _path_length)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; uint8 _algorithm
    1
    ;; string _name
    4 (length _name)
    ;; float32 _pose_x
    4
    ;; float32 _pose_y
    4
    ;; float32 _yaw
    4
    ;; float32 _initial_to_goal_distance
    4
    ;; float32 _current_to_goal_distance
    4
    ;; float32 _best_distance
    4
    ;; float32 _path_length
    4
    ;; float32 _time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _algorithm
       (write-byte _algorithm s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _pose_x
       (sys::poke _pose_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pose_y
       (sys::poke _pose_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initial_to_goal_distance
       (sys::poke _initial_to_goal_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_to_goal_distance
       (sys::poke _current_to_goal_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _best_distance
       (sys::poke _best_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _path_length
       (sys::poke _path_length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time
       (sys::poke _time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _algorithm
     (setq _algorithm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _pose_x
     (setq _pose_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pose_y
     (setq _pose_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initial_to_goal_distance
     (setq _initial_to_goal_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_to_goal_distance
     (setq _current_to_goal_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _best_distance
     (setq _best_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _path_length
     (setq _path_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time
     (setq _time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bug_algorithms::algorithmState :md5sum-) "6f90eba9625cb29111498ea4fb81eca4")
(setf (get bug_algorithms::algorithmState :datatype-) "bug_algorithms/algorithmState")
(setf (get bug_algorithms::algorithmState :definition-)
      "uint8 algorithm
string name
float32 pose_x
float32 pose_y
float32 yaw
float32 initial_to_goal_distance
float32 current_to_goal_distance
float32 best_distance
float32 path_length
float32 time

")



(provide :bug_algorithms/algorithmState "6f90eba9625cb29111498ea4fb81eca4")


