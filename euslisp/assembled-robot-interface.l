(require :robot-interface "package://pr2eus/robot-interface.l")

(defclass assembled-robot-interface
  :super robot-interface
  :slots ())
(defmethod assembled-robot-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot *assembled-robot-class* :controller-timeout ct args)
     ;; do something
     ))
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/follow_joint_trajectory")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  )

(defun assembled-robot-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* assembled-robot-interface :init args)))
  (if (not (boundp '*robot*))
      (setq *robot* (instance *assembled-robot-class* :init))))
;;
;;
;;
(cond
 ((and (boundp 'euscollada-robot) (class euscollada-robot))
  (let ((clst (remove-if-not #'(lambda (cl) (eq (send cl :super) euscollada-robot))
                             (system::list-all-classes))))
    (cond
     ((= (length clst) 1)
      (setq *assembled-robot-class* (car clst)))
     ((null clst) ;; あまりないはず
      (warn ";; you should load robot-file before loading this file!")
      )
     (t
      ;; ２タイプ以上のロボットをロードしている
      ))
    ))
 (t
  (warn ";; you should load robot-file before loading this file!")
  ))
