#!/usr/bin/env roseus

;; for noVNC
(push (list x::*visual*  24) x::*visuals*)

(require :robot-assembler "package://robot_assembler/euslisp/robot-assembler-viewer")

(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "jsk_rviz_plugins")

(defun message-warn (&rest args)
  (apply #'ros::ros-warn args))
(defun message-info (&rest args)
  (apply #'ros::ros-info args))

(defun setting-new-services () ;; ROS
  (ros::advertise-service "~/select_fixedpoint" std_srvs::Empty #'callback-select :fixed-point)
  (ros::advertise-service "~/select_actuator"   std_srvs::Empty #'callback-select :actuator)
  (ros::advertise-service "~/select_parts"      std_srvs::Empty #'callback-select :parts)

  (ros::advertise-service "~/service_command"   jsk_rviz_plugins::EusCommand #'callback-command)

  (ros::advertise "~/print_information" std_msgs::String)
  )
(defun print-information-gui (message) ;; ROS
  (format *error-output* "print_info: ~A~%" message)
  (let ((str (instance std_msgs::String :init :data message)))
    (ros::publish "~/print_information" str)
    ))
(defun robot-assembler-viewer-main (&key (node-name "robot_assembler") (use-new-feature t))
  (ros::roseus node-name :anonymous nil)

  (setq *project-dir* (ros::rospack-find "robot_assembler"))

  (setq *yaml-file* (format nil "~A/config/robot_assembler_parts_settings.yaml" *project-dir*))
  ;;(setq *output-dir* "/tmp")
  ;;(setq *robot-name* "assembled_robot")
  (setq *start-desgin-file* nil)

  (if (ros::has-param "~parts_settings_yaml")
      (setq *yaml-file* (ros::get-param "~parts_settings_yaml")))
  (if (ros::has-param "~output_directory")
      (setq *output-dir* (ros::get-param "~output_directory")))
  (if (ros::has-param "~robot_name")
      (setq *robot-name* (ros::get-param "~robot_name")))
  (if (ros::has-param "~start_desgin_file")
      (setq *start-desgin-file* (ros::get-param "~start_desgin_file")))

  (unless (and *start-desgin-file*
               (eq (probe-file *start-desgin-file*) :file))
    (ros::ros-warn ";; start_desgin_file (~A) does not exists"
                   *start-desgin-file*)
    (setq *start-desgin-file* nil))

  (robot-assembler-initialize *yaml-file* :project-dir *project-dir*)

  (setq *parts-id-prefix* (unix::getpid));; parts should have uniq name

  ;; viewer
  (make-robot-assembler-viewer)

  ;; set initial robot from desgin file
  (when *start-desgin-file*
    (ros::ros-info "start from design-file[~A]" *start-desgin-file*)
    (with-open-file
     (f *start-desgin-file*)
     (start-with-history (read f))
     ))

  (ros::advertise-service "~/delete_all" std_srvs::Empty #'callback-delete-all)
  (ros::advertise-service "~/write_urdf" std_srvs::Empty #'callback-write-urdf)
  (ros::advertise-service "~/attach"     std_srvs::Empty #'callback-attach)
  (ros::advertise-service "~/align"      std_srvs::Empty #'callback-align)
  (ros::advertise-service "~/unalign"    std_srvs::Empty #'callback-unalign)
  (ros::advertise-service "~/undo"       std_srvs::Empty #'callback-undo)
  (ros::advertise-service "~/reload"     std_srvs::Empty #'callback-reload)
  ;;
  #|
  (dolist (parts-name (list "xm430"  ;; TODO: use param
                            "fr12_h101" "fr12_h102" "fr12_h103" ;; "fr12_h104"
                            "fr12_s101" "fr12_s102" "fr12_s103"
                            "fr12_x102" "fr12_x103"
                            "hr12_f101"
                            "fr12_e170" "fr12_e171"
                            "body_a" "body_b"))
    (ros::advertise-service (format nil "~~use_~A" parts-name)
                            std_srvs::Empty #'callback-use-service-common parts-name)
    )
  |#
  ;; parse from setting.yaml
  (send *robot-assembler-functions-htable* :map
        #'(lambda (k v)
            (let ((parts-key k)
                  (parts-name (parts-key->parts-name k)))
              (ros::advertise-service (format nil "~~use_~A" parts-name)
                                      std_srvs::Empty #'callback-use-service-common parts-key)
              )))
  (if use-new-feature
      (setting-new-services))
  ;; ros-main
  (ros::rate 120)
  (while (ros::ok)
    (ros::spin-once)
    (x::window-main-one)
    (send *robot-assembler-viewer* :draw-objects)
    (ros::sleep))
  )

(robot-assembler-viewer-main)
