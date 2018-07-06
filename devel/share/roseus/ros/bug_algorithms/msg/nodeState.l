;; Auto-generated. Do not edit!


(when (boundp 'bug_algorithms::nodeState)
  (if (not (find-package "BUG_ALGORITHMS"))
    (make-package "BUG_ALGORITHMS"))
  (shadow 'nodeState (find-package "BUG_ALGORITHMS")))
(unless (find-package "BUG_ALGORITHMS::NODESTATE")
  (make-package "BUG_ALGORITHMS::NODESTATE"))

(in-package "ROS")
;;//! \htmlinclude nodeState.msg.html


(defclass bug_algorithms::nodeState
  :super ros::object
  :slots (_algorithm _node_state _node_state_desc _bug_state _bug_state_desc ))

(defmethod bug_algorithms::nodeState
  (:init
   (&key
    ((:algorithm __algorithm) 0)
    ((:node_state __node_state) 0)
    ((:node_state_desc __node_state_desc) "")
    ((:bug_state __bug_state) 0)
    ((:bug_state_desc __bug_state_desc) "")
    )
   (send-super :init)
   (setq _algorithm (round __algorithm))
   (setq _node_state (round __node_state))
   (setq _node_state_desc (string __node_state_desc))
   (setq _bug_state (round __bug_state))
   (setq _bug_state_desc (string __bug_state_desc))
   self)
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:node_state
   (&optional __node_state)
   (if __node_state (setq _node_state __node_state)) _node_state)
  (:node_state_desc
   (&optional __node_state_desc)
   (if __node_state_desc (setq _node_state_desc __node_state_desc)) _node_state_desc)
  (:bug_state
   (&optional __bug_state)
   (if __bug_state (setq _bug_state __bug_state)) _bug_state)
  (:bug_state_desc
   (&optional __bug_state_desc)
   (if __bug_state_desc (setq _bug_state_desc __bug_state_desc)) _bug_state_desc)
  (:serialization-length
   ()
   (+
    ;; uint8 _algorithm
    1
    ;; uint8 _node_state
    1
    ;; string _node_state_desc
    4 (length _node_state_desc)
    ;; uint8 _bug_state
    1
    ;; string _bug_state_desc
    4 (length _bug_state_desc)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _algorithm
       (write-byte _algorithm s)
     ;; uint8 _node_state
       (write-byte _node_state s)
     ;; string _node_state_desc
       (write-long (length _node_state_desc) s) (princ _node_state_desc s)
     ;; uint8 _bug_state
       (write-byte _bug_state s)
     ;; string _bug_state_desc
       (write-long (length _bug_state_desc) s) (princ _bug_state_desc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _algorithm
     (setq _algorithm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _node_state
     (setq _node_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _node_state_desc
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _node_state_desc (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _bug_state
     (setq _bug_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _bug_state_desc
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bug_state_desc (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get bug_algorithms::nodeState :md5sum-) "71ca713f8eb7d4e1de50aee1074f0a21")
(setf (get bug_algorithms::nodeState :datatype-) "bug_algorithms/nodeState")
(setf (get bug_algorithms::nodeState :definition-)
      "uint8 algorithm
uint8 node_state
string node_state_desc
uint8 bug_state
string bug_state_desc

")



(provide :bug_algorithms/nodeState "71ca713f8eb7d4e1de50aee1074f0a21")


