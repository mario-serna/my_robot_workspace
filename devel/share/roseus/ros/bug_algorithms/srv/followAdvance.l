;; Auto-generated. Do not edit!


(when (boundp 'bug_algorithms::followAdvance)
  (if (not (find-package "BUG_ALGORITHMS"))
    (make-package "BUG_ALGORITHMS"))
  (shadow 'followAdvance (find-package "BUG_ALGORITHMS")))
(unless (find-package "BUG_ALGORITHMS::FOLLOWADVANCE")
  (make-package "BUG_ALGORITHMS::FOLLOWADVANCE"))
(unless (find-package "BUG_ALGORITHMS::FOLLOWADVANCEREQUEST")
  (make-package "BUG_ALGORITHMS::FOLLOWADVANCEREQUEST"))
(unless (find-package "BUG_ALGORITHMS::FOLLOWADVANCERESPONSE")
  (make-package "BUG_ALGORITHMS::FOLLOWADVANCERESPONSE"))

(in-package "ROS")





(defclass bug_algorithms::followAdvanceRequest
  :super ros::object
  :slots (_algorithm _state _reverse _choose ))

(defmethod bug_algorithms::followAdvanceRequest
  (:init
   (&key
    ((:algorithm __algorithm) 0)
    ((:state __state) 0)
    ((:reverse __reverse) nil)
    ((:choose __choose) nil)
    )
   (send-super :init)
   (setq _algorithm (round __algorithm))
   (setq _state (round __state))
   (setq _reverse __reverse)
   (setq _choose __choose)
   self)
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
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
    ;; int32 _state
    4
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
     ;; int32 _state
       (write-long _state s)
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
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _reverse
     (setq _reverse (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _choose
     (setq _choose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bug_algorithms::followAdvanceResponse
  :super ros::object
  :slots (_success _message ))

(defmethod bug_algorithms::followAdvanceResponse
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

(defclass bug_algorithms::followAdvance
  :super ros::object
  :slots ())

(setf (get bug_algorithms::followAdvance :md5sum-) "0cdc9d6762a3b9cb2474f738c3771c80")
(setf (get bug_algorithms::followAdvance :datatype-) "bug_algorithms/followAdvance")
(setf (get bug_algorithms::followAdvance :request) bug_algorithms::followAdvanceRequest)
(setf (get bug_algorithms::followAdvance :response) bug_algorithms::followAdvanceResponse)

(defmethod bug_algorithms::followAdvanceRequest
  (:response () (instance bug_algorithms::followAdvanceResponse :init)))

(setf (get bug_algorithms::followAdvanceRequest :md5sum-) "0cdc9d6762a3b9cb2474f738c3771c80")
(setf (get bug_algorithms::followAdvanceRequest :datatype-) "bug_algorithms/followAdvanceRequest")
(setf (get bug_algorithms::followAdvanceRequest :definition-)
      "int32 algorithm
int32 state
bool reverse
bool choose
---
bool success
string message

")

(setf (get bug_algorithms::followAdvanceResponse :md5sum-) "0cdc9d6762a3b9cb2474f738c3771c80")
(setf (get bug_algorithms::followAdvanceResponse :datatype-) "bug_algorithms/followAdvanceResponse")
(setf (get bug_algorithms::followAdvanceResponse :definition-)
      "int32 algorithm
int32 state
bool reverse
bool choose
---
bool success
string message

")



(provide :bug_algorithms/followAdvance "0cdc9d6762a3b9cb2474f738c3771c80")


