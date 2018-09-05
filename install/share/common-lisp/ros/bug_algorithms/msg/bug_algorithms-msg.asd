
(cl:in-package :asdf)

(defsystem "bug_algorithms-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "algorithmState" :depends-on ("_package_algorithmState"))
    (:file "_package_algorithmState" :depends-on ("_package"))
    (:file "nodeState" :depends-on ("_package_nodeState"))
    (:file "_package_nodeState" :depends-on ("_package"))
  ))