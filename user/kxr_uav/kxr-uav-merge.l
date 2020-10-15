;; use roseus
(setq *project-dir* (ros::rospack-find "robot_assembler"))
(setq *yaml-file* (format nil "~A/config/robot_assembler_kxr_settings.yaml" *project-dir*))

(load "package://robot_assembler/euslisp/robot-assembler.l")
(robot-assembler-initialize *yaml-file* :project-dir *project-dir*)

;;
;; sample for merging some roboasm.l files
;;
(setq ht-torso (parse-roboasm (with-open-file (f "torso.roboasm.l") (read f))))
(setq ht-rtilt  (parse-roboasm (with-open-file (f "rtilt.roboasm.l") (read f))))
(setq ht-ltilt  (parse-roboasm (with-open-file (f "ltilt.roboasm.l") (read f))))
(setq ht-rleg  (parse-roboasm (with-open-file (f "rleg.roboasm.l") (read f))))
(setq ht-lleg  (parse-roboasm (with-open-file (f "lleg.roboasm.l") (read f))))
(setq ht-rarm  (parse-roboasm (with-open-file (f "rarm.roboasm.l") (read f))))
(setq ht-larm  (parse-roboasm (with-open-file (f "larm.roboasm.l") (read f))))

(setq ht-merged ht-torso)
(setq ht-merged
      (merge-parsed-table ht-merged ht-rtilt
                          :s3301_18721_10/horn
                          :joint-base_19059_0/horn-hole
			  :configuration :rotate-90))
(setq ht-merged
      (merge-parsed-table ht-merged ht-ltilt
                          :s3301_18721_18/horn
                          :joint-base_19060_0/horn-hole
			  :configuration :rotate90))
(setq ht-merged
      (merge-parsed-table ht-merged ht-lleg
                          :s3301_27908_1/horn
			  :joint-base_5143_0/horn-hole
			  :configuration :rotate-90))
(setq ht-merged
      (merge-parsed-table ht-merged ht-rleg
                          :s3301_27908_0/horn
			  :joint-base_5142_0/horn-hole
			  :configuration :rotate90))
(setq ht-merged
      (merge-parsed-table ht-merged ht-larm
                          :s3301_18721_28/horn
                          :joint-base_7080_0/horn-hole
			  :configuration :rotate-90))
(setq ht-merged
      (merge-parsed-table ht-merged ht-rarm
                          :s3301_18721_24/horn
                          :joint-base_7079_0/horn-hole
			  :configuration :rotate90))

(setq asm (create-roboasm-from-parsed-table ht-merged))

(with-open-file (f "kxr_uav_fullbody.roboasm.l" :direction :io) (pprint asm f))

#|
<body>
rarm
:s3301_30133_8/horn
/ (make-coords :pos (float-vector -51.000000000000 7.250000000000 -43.500000000000) :rpy (float-vector 1.570796326795 0.000000000000 -1.570796326795))

larm
:s3301_30133_18/horn
/ (make-coords :pos (float-vector 51.000000000000 7.250000000000 -43.500000000000) :rpy (float-vector 1.570796326795 -0.000000000000 1.570796326795))

head
:s3301_30133_13/horn
/ (make-coords :pos (float-vector -0.000000000000 7.250000000000 -61.500000000000) :rpy (float-vector 1.570796326795 -0.000000000000 -3.141592653590))

rleg
:battery-leg-plate_30133_3/4holes0
/ (make-coords :pos (float-vector -23.500000000000 0.000000000000 24.000000000000) :rpy (float-vector 1.570796326795 0.000000000000 -3.141592653590))

lleg
:battery-leg-plate_30133_3/4holes1
/ (make-coords :pos (float-vector 23.500000000000 -0.000000000000 24.000000000000) :rpy (float-vector 1.570796326795 0.000000000000 -3.141592653590))

---

<rleg>
:joint-base_9085_0/bottom-4holes
/ (make-coords :pos (float-vector 0.000000000000 0.000000000000 -3.000000000000) :rpy (float-vector 0.000000000000 -0.000000000000 3.141592653590))

<lleg>
:joint-base_8085_0/bottom-4holes

<rarm>
:joint-base_7089_0/horn-hole

<larm>
:joint-base_8089_0/horn-hole

<head>
:joint-base_23576_0/horn-hole
|#
