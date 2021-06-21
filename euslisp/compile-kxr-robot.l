(defun compile-kxr-robot (roboasm-file-name urdf-file-name &key (robotname) (euslisp-model-file-name) (collada-yaml-file-name))
  (load "package://robot_assembler/euslisp/robot-assembler-viewer")
  (robot-assembler-initialize
   (ros::resolve-ros-path  "package://robot_assembler/config/robot_assembler_kxr_settings.yaml")
   :project-dir (ros::resolve-ros-path  "package://robot_assembler"))
  ;;
  (setq roboasm (with-open-file (f roboasm-file-name) (read f)))
  (when roboasm
    (setq rbt (make-robot-from-roboasm roboasm))
    (unless robotname
      (setq robotname (send rbt :name)))
    (send rbt :write-urdf urdf-file-name :robotname robotname)
    )
  ;;
  (when euslisp-model-file-name
    (unless collada-yaml-file-name
      (setq collada-yaml-file-name (format nil "~A.euscollada.yaml" urdf-file-name)))
    (let ((strm (piped-fork (format nil "rosrun euscollada collada2eus -I ~A -C ~A -O ~A"
                                    urdf-file-name collada-yaml-file-name euslisp-model-file-name))))
      (while (read-line strm nil nil))
      (close strm)
      ))
  )
