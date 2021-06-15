(require :unittest "lib/llib/unittest.l")

(require :eus-assimp "package://eus_assimp/euslisp/eus-assimp.l")
(require :robot-assembler "package://robot_assembler/euslisp/robot-assembler.l")

(init-unit-test)

(deftest test-load-sample ()
  (let (robot)
    ;; initialize
    (robot-assembler-initialize
     (ros::resolve-ros-path "package://robot_assembler/config/robot_assembler_parts_settings.yaml")
     :project-dir (ros::resolve-ros-path "package://robot_assembler"))
    ;;
    (setq robot (parse-roboasm-list
                 (with-open-file (f (ros::resolve-ros-path "package://robot_assembler/test/SAMPLE_HUMANOID.roboasm.l"))
                                 (read f))))
    (assert robot)
    (assert (= (length (send robot :all-parts)) 62))
    ;;
    (send robot :write-urdf "/tmp/sample.urdf")
    )
  )

(deftest test-load-sample-kxr ()
  (let (robot)
    ;; initialize kxr
    (robot-assembler-initialize
     (ros::resolve-ros-path "package://robot_assembler/config/robot_assembler_kxr_settings.yaml")
     :project-dir (ros::resolve-ros-path "package://robot_assembler"))
    ;;
    (setq robot (parse-roboasm-list
                 (with-open-file (f (ros::resolve-ros-path "package://robot_assembler/test/kxr_fullbody.roboasm.l"))
                                 (read f))))
    (assert robot)
    (assert (= (length (send robot :all-parts)) 112))
    ;;
    (send robot :write-urdf "/tmp/sample_kxr.urdf")
    )
  )

(deftest test-load-sample-kxr-sensor-config ()
  (let (robot robot2 robot3)
    ;; initialize kxr
    (robot-assembler-initialize
     (ros::resolve-ros-path "package://robot_assembler/config/robot_assembler_kxr_settings.yaml")
     :project-dir (ros::resolve-ros-path "package://robot_assembler"))
    ;;
    (setq robot (parse-roboasm-list
                 (with-open-file (f (ros::resolve-ros-path "package://robot_assembler/test/kxr4r.roboasm.l"))
                                 (read f))))
    (assert robot)
    (assert (= (length (send robot :all-parts)) 44))
    (assert (eps-coords= (send (send robot :worldcoords) :transformation
                               (send (car (send robot :descendants)) :worldcoords))
                         (make-coords) 1e-9))
    (send robot :write-urdf "/tmp/sample_kxr4r.urdf")
    ;; check sensor, joint-name

    (setq robot2 (parse-roboasm-list
                  (with-open-file (f (ros::resolve-ros-path "package://robot_assembler/test/kxr4r.test2.roboasm.l"))
                                  (read f))))
    (assert robot2)
    (assert (= (length (send robot2 :all-parts)) 44))
    (assert (eps-coords= (send (send robot2 :worldcoords) :transformation
                               (send (car (send robot2 :descendants)) :worldcoords))
                         (make-coords :rpy (list 0 pi/2 0)) 1e-9))
    (send robot2 :write-urdf "/tmp/sample_kxr4r_2.urdf")
    ;; check sensor, joint-name

    (setq robot3 (parse-roboasm-list
                 (with-open-file (f (ros::resolve-ros-path "package://robot_assembler/test/kxr4r.test3.roboasm.l"))
                                 (read f))))
    (assert robot3)
    (assert (= (length (send robot3 :all-parts)) 44))
    (assert (eps-coords= (send (send robot3 :worldcoords) :transformation
                               (send (car (send robot3 :descendants)) :worldcoords))
                         (make-coords :rpy (list 0 pi/2 0)) 1e-9))
    (send robot3 :write-urdf "/tmp/sample_kxr4r_3.urdf")
    ;; check sensor
    )
  )
;; :iniial-angle
;; equality write-roboasm/read-roboasm
;; search method
;; roboasm-from-parts
;; attach robot
;; processing roboasm | merge/rename/root-change/re-write
;; TEST: Viewer ?GUI
;; TEST: Node / send command by service ?GUI
(run-all-tests)
(exit)
