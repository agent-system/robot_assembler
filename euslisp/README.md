# Conversion rcb4robot to roboasm-robot

## For making information table of kxr-parts in robot-assembler
~~~
;; Here is the process on robot-assembler side
(load "package://robot_assembler/euslisp/robot-assembler-viewer.l")
(load "package://robot_assembler/euslisp/convert-rcb4robots.l")
;; "/tmp/ra_parts_attach_list.l" is the information table
(make-parts-association-file "/tmp/ra_parts_attach_list.l")
~~~

## Convert rcb4robots to roboasm-robot
~~~
;; Here is the process on rcb4robot side
(load "rcb4robots.l")
(load "package://robot_assembler/euslisp/convert-rcb4robots.l")
(load-parts-association-file "/tmp/ra_parts_attach_list.l")

;; convert kxr-robot to roboasm-robot
(setq *robot* (rename-and-add-bbox (kxr-make-robot "kxrl2l6a6h2")))
(setq ht (convert-robot-to-roboasm *robot*)) ;; make hash-table from kxr-robot
(dump-hash "/tmp/roboasm.l" ht) ;; dump hash-table to roboasm.l
~~~

## Check created roboasm.l
~~~
;; load roboasm.l and create roboasm-robot
(load "package://robot_assembler/euslisp/robot-assembler-viewer.l")
(require :eus-assimp "package://eus_assimp/euslisp/eus-assimp.l")
(robot-assembler-initialize
 (ros::resolve-ros-path  "package://robot_assembler/config/robot_assembler_kxr_settings.yaml")
 :project-dir (ros::resolve-ros-path  "package://robot_assembler"))
(setq ht (load-hash "/tmp/roboasm.l"))
(setq roboasm (create-roboasm-from-parsed-table ht))
(setq rbt (make-robot-from-roboasm roboasm))
;; if the conversion is successed, you can see correct robot
(objects (list rbt))
~~~