
(cl:in-package :asdf)

(defsystem "marsworks_vis_2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Tvec" :depends-on ("_package_Tvec"))
    (:file "_package_Tvec" :depends-on ("_package"))
  ))