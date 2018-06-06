
(cl:in-package :asdf)

(defsystem "ariaClientDriver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AriaNavData" :depends-on ("_package_AriaNavData"))
    (:file "_package_AriaNavData" :depends-on ("_package"))
    (:file "AriaCommandData" :depends-on ("_package_AriaCommandData"))
    (:file "_package_AriaCommandData" :depends-on ("_package"))
  ))