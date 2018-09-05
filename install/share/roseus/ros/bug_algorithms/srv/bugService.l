;; Auto-generated. Do not edit!


(when (boundp 'bug_algorithms::bugService)
  (if (not (find-package "BUG_ALGORITHMS"))
    (make-package "BUG_ALGORITHMS"))
  (shadow 'bugService (find-package "BUG_ALGORITHMS")))
(unless (find-package "BUG_ALGORITHMS::BUGSERVICE")
  (make-package "BUG_ALGORITHMS::BUGSERVICE"))
(unless (find-package "BUG_ALGORITHMS::BUGSERVICEREQUEST")
  (make-package "BUG_ALGORITHMS::BUGSERVICEREQUEST"))
(unless (find-package "BUG_ALGORITHMS::BUGSERVICERESPONSE")
  (make-package "BUG_ALGORITHMS::BUGSERVICERESPONSE"))

(in-package "ROS")





(defclass bug_algorithms::bugServiceRequest
  :super ros::object
  :slots (_algorithm _velocity _initial_x _initial_y _desired_x _desired_y _simulation _reverse _choose ))

(defmethod bug_algorithms::bugServiceRequest
  (:init
   (&key
    ((:algorithm __algorithm) 0)
    ((:velocity __velocity) 0.0)
    ((:initial_x __initial_x) 0.0)
    ((:initial_y __initial_y) 0.0)
    ((:desired_x __desired_x) 0.0)
    ((:desired_y __desired_y) 0.0)
    ((:simulation __simulation) nil)
    ((:reverse __reverse) nil)
    ((:choose __choose) nil)
    )
   (send-super :init)
   (setq _algorithm (round __algorithm))
   (setq _velocity (float __velocity))
   (setq _initial_x (float __initial_x))
   (setq _initial_y (float __initial_y))
   (setq _desired_x (float __desired_x))
   (setq _desired_y (float __desired_y))
   (setq _simulation __simulation)
   (setq _reverse __reverse)
   (setq _choose __choose)
   self)
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:initial_x
   (&optional __initial_x)
   (if __initial_x (setq _initial_x __initial_x)) _initial_x)
  (:initial_y
   (&optional __initial_y)
   (if __initial_y (setq _initial_y __initial_y)) _initial_y)
  (:desired_x
   (&optional __desired_x)
   (if __desired_x (setq _desired_x __desired_x)) _desired_x)
  (:desired_y
   (&optional __desired_y)
   (if __desired_y (setq _desired_y __desired_y)) _desired_y)
  (:simulation
   (&optional __simulation)
   (if __simulation (setq _simulation __simulation)) _simulation)
  (:reverse
   (&optional __reverse)
   (if __reverse (setq _reverse __reverse)) _reverse)
  (:choose
   (&optional __choose)
   (if __choose (setq _choose __choose)) _choose)
  (:serialization-length
   ()
   (+
    ;; int32 _algorithm
    4
    ;; float32 _velocity
    4
    ;; float32 _initial_x
    4
    ;; float32 _initial_y
    4
    ;; float32 _desired_x
    4
    ;; float32 _desired_y
    4
    ;; bool _simulation
    1
    ;; bool _reverse
    1
    ;; bool _choose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _algorithm
       (write-long _algorithm s)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initial_x
       (sys::poke _initial_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initial_y
       (sys::poke _initial_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desired_x
       (sys::poke _desired_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desired_y
       (sys::poke _desired_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _simulation
       (if _simulation (write-byte -1 s) (write-byte 0 s))
     ;; bool _reverse
       (if _reverse (write-byte -1 s) (write-byte 0 s))
     ;; bool _choose
       (if _choose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _algorithm
     (setq _algorithm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initial_x
     (setq _initial_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initial_y
     (setq _initial_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desired_x
     (setq _desired_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desired_y
     (setq _desired_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _simulation
     (setq _simulation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reverse
     (setq _reverse (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _choose
     (setq _choose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bug_algorithms::bugServiceResponse
  :super ros::object
  :slots (_success _message ))

(defmethod bug_algorithms::bugServiceResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass bug_algorithms::bugService
  :super ros::object
  :slots ())

(setf (get bug_algorithms::bugService :md5sum-) "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(setf (get bug_algorithms::bugService :datatype-) "bug_algorithms/bugService")
(setf (get bug_algorithms::bugService :request) bug_algorithms::bugServiceRequest)
(setf (get bug_algorithms::bugService :response) bug_algorithms::bugServiceResponse)

(defmethod bug_algorithms::bugServiceRequest
  (:response () (instance bug_algorithms::bugServiceResponse :init)))

(setf (get bug_algorithms::bugServiceRequest :md5sum-) "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(setf (get bug_algorithms::bugServiceRequest :datatype-) "bug_algorithms/bugServiceRequest")
(setf (get bug_algorithms::bugServiceRequest :definition-)
      "int32 algorithm
float32 velocity
float32 initial_x
float32 initial_y
float32 desired_x
float32 desired_y
bool simulation
bool reverse
bool choose
---
bool success
string message

")

(setf (get bug_algorithms::bugServiceResponse :md5sum-) "c7f9b178e6a7aa0cd2e64b98d5c26b60")
(setf (get bug_algorithms::bugServiceResponse :datatype-) "bug_algorithms/bugServiceResponse")
(setf (get bug_algorithms::bugServiceResponse :definition-)
      "int32 algorithm
float32 velocity
float32 initial_x
float32 initial_y
float32 desired_x
float32 desired_y
bool simulation
bool reverse
bool choose
---
bool success
string message

")



(provide :bug_algorithms/bugService "c7f9b178e6a7aa0cd2e64b98d5c26b60")


