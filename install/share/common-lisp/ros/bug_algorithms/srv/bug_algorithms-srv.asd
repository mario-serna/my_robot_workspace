
(cl:in-package :asdf)

(defsystem "bug_algorithms-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "bugService" :depends-on ("_package_bugService"))
    (:file "_package_bugService" :depends-on ("_package"))
    (:file "bugSwitch" :depends-on ("_package_bugSwitch"))
    (:file "_package_bugSwitch" :depends-on ("_package"))
  ))