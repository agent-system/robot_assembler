(load "package://eus_pcl/test/create-octomap-from-eusmodel.l")
(load "package://eus_assimp/euslisp/eus-assimp.l")

(defun make-camera-from-param (&key pwidth pheight fx fy cx cy (tx 0) (ty 0) parent-coords name
                                    (near-clip 10.0) (far-clip 10000.0)
                                    create-viewer)
  "Create camera object from given parameters."
  (let* ((b (body+ (make-cube 40 30 30)
                   (send (make-cylinder 2 30) :rotate pi/2 :x)
                   (make-cylinder 13 25)))
	 c (vd 1.0))
    (setq c (instance camera-model :init b :width pwidth :height pheight
                      :viewdistance vd :name name
                      :yon far-clip :hither near-clip))
    (send (c . vwing) :screen (/ (* vd pwidth 0.5) fx) (/ (* vd pheight 0.5) fy))
    (send c :newprojection
          (make-matrix 4 4
                       (list (float-vector fx  0 cx (* fx tx))
                             (float-vector  0 fy cy (* fy ty))
                             (float-vector  0  0  1  0)
                             (float-vector  0  0  0  1))))
    (if parent-coords (send c :newcoords (send parent-coords :copy-worldcoords)))
    (send c :translate (float-vector (- tx) (- ty) 0))
    (send (c . vwing) :translate (float-vector tx ty 0))
    (if parent-coords (send parent-coords :assoc c))
    (if create-viewer (send c :create-viewer))
    c))

#|
"meshes/mesh_WHEEL.dae" -> cylinder
|#

(setq bds-x (list
"meshes/mesh_ANGLE-BRACKET.dae" ;; 0 :x
"meshes/mesh_FINGER.dae" ;; 3 x
))
(setq bds-y (list
"meshes/mesh_BACK-PACK.dae" ;; 6 y
"meshes/mesh_SOLE.dae" ;; 17 y
"meshes/mesh_BOTTOM-SPACER.dae" ;; y
))
(setq bds (list
"meshes/mesh_DUMMY-SERVO.dae"
"meshes/mesh_ARM-SUPPORTER-A.dae"
"meshes/mesh_ARM-SUPPORTER-B.dae"
"meshes/mesh_FLAT-FRAME.dae"
"meshes/mesh_JOINT-BASE.dae"
"meshes/mesh_BATTERY-BOX.dae"
"meshes/mesh_JOINT-FRAME-A.dae"
"meshes/mesh_BATTERY-LEG-PLATE.dae"
"meshes/mesh_JOINT-FRAME-B-A.dae"
"meshes/mesh_BODY-PLATE.dae"
"meshes/mesh_JOINT-FRAME-B-B.dae"
"meshes/mesh_BOTTOM-ARM20.dae"
"meshes/mesh_SERVO-S3301.dae"
"meshes/mesh_BOTTOM-ARM26.dae"
"meshes/mesh_BOTTOM-ARM28.dae"
"meshes/mesh_UPPER-ARM20.dae"
"meshes/mesh_BOTTOM-ARM38.dae"
"meshes/mesh_UPPER-ARM26.dae"
"meshes/mesh_UPPER-ARM28.dae"
"meshes/mesh_CROSS-SERVO-FRAME-A.dae"
"meshes/mesh_UPPER-ARM38.dae"
"meshes/mesh_CROSS-SERVO-FRAME-B.dae"
"meshes/mesh_DUMMY-HALF-SERVO.dae"
))

(setq *cam-model*
      (make-camera-from-param :pwidth 160 :pheight 160
                              :fx 100 :fy 100
                              :cx 79.5 :cy 59.5 :name "camtest"
                              :create-viewer t))

(defun create-model (meshname &key (rotate))
  (setq *glv* (load-mesh-file meshname :scale 1000.0))
  (setq *glv-org* (load-mesh-file meshname :scale 1000.0))

  (case rotate
    (:x
     (send *glv* :rotate pi/2 :x))
    (:y
     (send *glv* :rotate pi/2 :y))
    )
  (let ((bb (make-bounding-box
             (let ((cds (send *glv* :worldcoords))) (mapcar #'(lambda (v) (send cds :transform-vector v)) (send *glv* :vertices)))
             0.0)))
  ;;(setq bb (make-bounding-box (send glv :vertices) 0.0))
    (setq *mv-cent* (send bb :center))
    (send *glv* :locate (v- *mv-cent*) :world)
    (setq *glv* (send *glv* :convert-to-world))

    (setq *obj* (instance bodyset :init (make-cascoords) :bodies (list *glv*)))

    ))

(defun create-omap (target-obj &key (resolution 0.4))
  (setq *omap (create-pointcloud-octomap
               target-obj :resolution resolution :camera-model *cam-model* :interactive nil))
  ;;(send *omap* :update-visual-color :transparent nil :depth -2)

  (setq *pt* (send (instance pointcloud :init) :copy-from *omap*))
  )

(defun parse-to-box-list (points &key (threshold 1.7))
  (setq *hash* (make-hash-table))
  (dolist (p (send points :point-list))
    (let ((r (gethash (elt p 2) *hash* )))
      (if r
          (nconc r (cons p nil))
        (setf (gethash (elt p 2) *hash* ) (list p)))
      ))
  ;;
  (setq *bbox-list* nil)
  (maphash #'(lambda (k v) (push (make-bounding-box v 0.0) *bbox-list*))  *hash*)
  (sort *bbox-list* #'(lambda (a b) (> (elt (send a :center) 2) (elt (send b :center) 2))))

  (let ((prev-max (send (car *bbox-list*) :maxpoint))
        (prev-min (send (car *bbox-list*) :minpoint))
        elv box-lst
        )
    (setq elv (elt prev-max 2))
    (setq prev-max (float-vector (elt prev-max 0) (elt prev-max 1)))
    (setq prev-min (float-vector (elt prev-min 0) (elt prev-min 1)))

    (setq box-lst (list (list prev-max prev-min elv)))

    (dolist (bb (cdr *bbox-list*))
      ;;(send bb :center)
      (let ((maxp (send bb :maxpoint))
            (minp (send bb :minpoint))
            elv
            )
        (setq elv (elt maxp 2))
        (setq maxp (float-vector (elt maxp 0) (elt maxp 1)))
        (setq minp (float-vector (elt minp 0) (elt minp 1)))
        (when (> (+ (distance maxp prev-max)
                    (distance minp prev-min)) threshold)
          #|
          (pprint (list maxp prev-max
                        minp prev-min
                        (+ (distance maxp prev-max)
                           (distance minp prev-min))))
          |#
          (setq prev-max maxp
                prev-min minp)
          (push (list maxp minp elv) box-lst)
          )
        ))

    (let ((lst-bb (car (last *bbox-list*)))
          l-elv)
      (setq l-elv (elt (send lst-bb :maxpoint) 2))
      (push (list prev-max prev-min l-elv) box-lst)
      )
    box-lst
    ))

(defun make-box-list (box-lst)
  (let (ret prev-c)
    (setq prev-c (caddr (car box-lst)))
    (dolist (p (cdr box-lst))
      (let ((a (car p))
            (b (cadr p))
            (c (caddr p))
            cent bx
            )
        (setq cent (v+ a b))
        (setq b (v- a b))

        (setq cent (scale 0.5 (float-vector (elt cent 0) (elt cent 1)  (+ c prev-c))))

        (setq bx (make-cube (elt b 0) (elt b 1) (- c prev-c)))
        (push bx ret)
        (send bx :translate cent)

        (setq prev-c c)
        ))
    ret))

(defun optimize-box (points &key (resolution 0.4) (maximum-box-num 7))
  (let (ret (N 2))
    (dotimes (i 14)
      (setq ret (parse-to-box-list points :threshold (+ 0.1 (* N resolution))))
      ;;(if (< (length ret) 7) (return))
      (when (< (length ret) maximum-box-num)
        ;;(pprint (length ret))
        ;;(pprint ret)
        (return))
      (incf N))
    ret))

(defun print-box-list (boxlst &optional (offset "       "))
  (format t "~Acollision:~%" offset)
  (dolist (abox boxlst)
    (let ((pos (scale 0.001 (send abox :worldpos)))
          (binfo (cdr (car (send abox :csg))))
          (ypr (car (send (send abox :worldcoords) :rpy-angle)))
          )
      (format t "~A  -~%" offset)
      (format t "~A    translate: [~8,6f, ~8,6f, ~8,6f]~%"
              offset
              (elt pos 0) (elt pos 1) (elt pos 2))
      (cond
       ((not (eps= (elt ypr 1) 0.0)) ;; rotate y
        (format t "~A    box: [~8,6f, ~8,6f, ~8,6f]~%"
                offset
                (* 0.001 (elt binfo 2))
                (* 0.001 (elt binfo 1))
                (* 0.001 (elt binfo 0))
                )
        )
       ((not (eps= (elt ypr 2) 0.0)) ;; rotate x
        (format t "~A    box: [~8,6f, ~8,6f, ~8,6f]~%"
                offset
                (* 0.001 (elt binfo 0))
                (* 0.001 (elt binfo 2))
                (* 0.001 (elt binfo 1))
                )
        )
       (t ;; no rotate
        (format t "~A    box: [~8,6f, ~8,6f, ~8,6f]~%"
                offset
                (* 0.001 (elt binfo 0))
                (* 0.001 (elt binfo 1))
                (* 0.001 (elt binfo 2))
                )
        ))
      ))
  )

(defun make-all-boxes (bdslst &key (rotate))
  (dotimes (i (length bdslst))
    (format t "# boxes for ~s~%" (elt bdslst i))
    ;; create model => *obj*
    (create-model (elt bdslst i) :rotate rotate)

    ;; create ocotomap => *omap* and *pt (center points of boxes)
    (create-omap *obj* :resolution 0.4)

    ;; fitting *pt* to some boxes
    (setq ret (optimize-box *pt*))

    ;; create boxes(eus) for visualize
    (setq boxlst (make-box-list ret))

    ;; move boxes to original position
    (send-all boxlst :translate *mv-cent*)
    (when rotate
      (let ((orig (make-cascoords)))
        (dolist (b boxlst) (send orig :assoc b))
        (case rotate
          (:x
           (send orig :rotate -pi/2 :x))
          (:y
           (send orig :rotate -pi/2 :y))
          )
        ))

    ;; print collision description for settings.yaml
    (print-box-list boxlst)
    )
  )

(warn "
(make-all-boxes bds)
(make-all-boxes bds-x :rotate :x)
(make-all-boxes bds-y :rotate :y)
")
