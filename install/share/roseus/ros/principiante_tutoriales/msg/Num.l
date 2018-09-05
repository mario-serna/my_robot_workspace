;; Auto-generated. Do not edit!


(when (boundp 'principiante_tutoriales::Num)
  (if (not (find-package "PRINCIPIANTE_TUTORIALES"))
    (make-package "PRINCIPIANTE_TUTORIALES"))
  (shadow 'Num (find-package "PRINCIPIANTE_TUTORIALES")))
(unless (find-package "PRINCIPIANTE_TUTORIALES::NUM")
  (make-package "PRINCIPIANTE_TUTORIALES::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass principiante_tutoriales::Num
  :super ros::object
  :slots (_num ))

(defmethod principiante_tutoriales::Num
  (:init
   (&key
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _num (round __num))
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; int64 _num
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num) (= (length (_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num . bv) 0) 0) s)
              (write-long (ash (elt (_num . bv) 1) -1) s))
             ((and (class _num) (= (length (_num . bv)) 1)) ;; big1
              (write-long (elt (_num . bv) 0) s)
              (write-long (if (>= _num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num s)(write-long (if (>= _num 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _num
#+(or :alpha :irix6 :x86_64)
      (setf _num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get principiante_tutoriales::Num :md5sum-) "57d3c40ec3ac3754af76a83e6e73127a")
(setf (get principiante_tutoriales::Num :datatype-) "principiante_tutoriales/Num")
(setf (get principiante_tutoriales::Num :definition-)
      "int64 num

")



(provide :principiante_tutoriales/Num "57d3c40ec3ac3754af76a83e6e73127a")


