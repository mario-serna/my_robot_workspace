;; Auto-generated. Do not edit!


(when (boundp 'bug_algorithms::bugSwitch)
  (if (not (find-package "BUG_ALGORITHMS"))
    (make-package "BUG_ALGORITHMS"))
  (shadow 'bugSwitch (find-package "BUG_ALGORITHMS")))
(unless (find-package "BUG_ALGORITHMS::BUGSWITCH")
  (make-package "BUG_ALGORITHMS::BUGSWITCH"))
(unless (find-package "BUG_ALGORITHMS::BUGSWITCHREQUEST")
  (make-package "BUG_ALGORITHMS::BUGSWITCHREQUEST"))
(unless (find-package "BUG_ALGORITHMS::BUGSWITCHRESPONSE")
  (make-package "BUG_ALGORITHMS::BUGSWITCHRESPONSE"))

(in-package "ROS")





(defclass bug_algorithms::bugSwitchRequest
  :super ros::object
  :slots (_algorithm _state ))

(defmethod bug_algorithms::bugSwitchRequest
  (:init
   (&key
    ((:algorithm __algorithm) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _algorithm (round __algorithm))
   (setq _state (round __state))
   self)
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; int32 _algorithm
    4
    ;; int32 _state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _algorithm
       (write-long _algorithm s)
     ;; int32 _state
       (write-long _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _algorithm
     (setq _algorithm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass bug_algorithms::bugSwitchResponse
  :super ros::object
  :slots (_success _message ))

(defmethod bug_algorithms::bugSwitchResponse
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

(defclass bug_algorithms::bugSwitch
  :super ros::object
  :slots ())

(setf (get bug_algorithms::bugSwitch :md5sum-) "06a69ed785453bce53ed290be7818f28")
(setf (get bug_algorithms::bugSwitch :datatype-) "bug_algorithms/bugSwitch")
(setf (get bug_algorithms::bugSwitch :request) bug_algorithms::bugSwitchRequest)
(setf (get bug_algorithms::bugSwitch :response) bug_algorithms::bugSwitchResponse)

(defmethod bug_algorithms::bugSwitchRequest
  (:response () (instance bug_algorithms::bugSwitchResponse :init)))

(setf (get bug_algorithms::bugSwitchRequest :md5sum-) "06a69ed785453bce53ed290be7818f28")
(setf (get bug_algorithms::bugSwitchRequest :datatype-) "bug_algorithms/bugSwitchRequest")
(setf (get bug_algorithms::bugSwitchRequest :definition-)
      "int32 algorithm
int32 state
---
bool success
string message

")

(setf (get bug_algorithms::bugSwitchResponse :md5sum-) "06a69ed785453bce53ed290be7818f28")
(setf (get bug_algorithms::bugSwitchResponse :datatype-) "bug_algorithms/bugSwitchResponse")
(setf (get bug_algorithms::bugSwitchResponse :definition-)
      "int32 algorithm
int32 state
---
bool success
string message

")



(provide :bug_algorithms/bugSwitch "06a69ed785453bce53ed290be7818f28")


