;;
;; sample for merging some roboasm.l files
;;
(setq ht-torso (parse-roboasm (with-open-file (f "kxr_body.roboasm.l") (read f))))
(setq ht-head  (parse-roboasm (with-open-file (f "kxr_head.roboasm.l") (read f))))
(setq ht-lleg  (parse-roboasm (with-open-file (f "kxr_lleg.roboasm.l") (read f))))
(setq ht-rleg  (parse-roboasm (with-open-file (f "kxr_rleg.roboasm.l") (read f))))
(setq ht-larm  (parse-roboasm (with-open-file (f "kxr_larm.roboasm.l") (read f))))
(setq ht-rarm  (parse-roboasm (with-open-file (f "kxr_rarm.roboasm.l") (read f))))

(setq ht-merged ht-torso)
(setq ht-merged
      (merge-parsed-table ht-merged ht-head
                          :s3301_30133_13/horn
                          :joint-base_23576_0/horn-hole
                          :configuration :rotate90))
(setq ht-merged
      (merge-parsed-table ht-merged ht-lleg
                          :battery-leg-plate_30133_3/4holes1
                          :joint-base_8085_0/bottom-4holes
                          :configuration :invert))
(setq ht-merged
      (merge-parsed-table ht-merged ht-rleg
                          :battery-leg-plate_30133_3/4holes0
                          :joint-base_9085_0/bottom-4holes
                          :configuration :invert))
(setq ht-merged
      (merge-parsed-table ht-merged ht-larm
                          :s3301_30133_18/horn
                          :joint-base_8089_0/horn-hole))
(setq ht-merged
      (merge-parsed-table ht-merged ht-rarm
                          :s3301_30133_8/horn
                          :joint-base_7089_0/horn-hole))

(setq asm (create-roboasm-from-parsed-table ht-merged))

(with-open-file (f "kxr_fullbody.roboasm.l" :direction :io) (pprint asm f))

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
