
(cl:in-package :asdf)

(defsystem "spyndra-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BeaconPos" :depends-on ("_package_BeaconPos"))
    (:file "_package_BeaconPos" :depends-on ("_package"))
    (:file "MotorSignal" :depends-on ("_package_MotorSignal"))
    (:file "_package_MotorSignal" :depends-on ("_package"))
  ))