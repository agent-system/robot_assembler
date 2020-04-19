(require :robot-assembler "robot-assembler.l")

(ros::roseus-add-srvs "std_srvs")

(defun make-robot-asselbler-viewer (&key (width 1200) (height 900))
  ;;
  (setq *robot-assembler-viewer*
        (instance robot-assembler-viewer :create
                  :name "Robot Assembler" :width width :height height))
  ;;
  (send (send *robot-assembler-viewer* :viewer)
        :viewsurface :set-event-proc :buttonrelease-right
        :select-fixed-point *robot-assembler-viewer*)
  ;;
  (setq *current-robot* nil)
  (setq *current-robot-point* nil)
  (setq *current-parts* nil)
  (setq *current-parts-point* nil)
  (setq *current-configuration* nil)
  ;;
  (setq *attach-history* nil)
  (setq *parts-create-counter* 0)
  )

(defclass robot-assembler-viewer :super x::irtviewer
  :slots (pick-face-pos pick-facesets))
;;(defmethod robot-assembler-viewer
(defmethod robot-assembler-viewer
  (:string (x y str &optional ;;(fid x::font-helvetica-bold-12))
              (fid x::font-a14))
   (send-all x::subwindows :2d-mode)
   ;;(gl::glxUseXfont fid 32 96 (+ 1000 32))
   (gl::glxUseXfont fid 32 96 (+ 1000 32))
   (gl::glRasterPos2i (round x) (- (send self :height) (round y)))
   (gl::glListBase 1000)
   (gl::glCallLists (length str) gl::GL_UNSIGNED_BYTE str)
   (send-all x::subwindows :3d-mode)
   (send self :flush)
   )
  (:pick-face
   (spos body-list)
   (let* ((eye (send self :viewer :viewing))
          (vp (send self :viewer :viewport :screen-point-to-ndc spos))
          (wc (send eye :worldcoords))
          ;;(st (send eye :viewpoint))
          ;;(st (send eye :worldpos))
          ;;(direc (send eye :view-orient vp))
          ;;(vline (make-line st (v+ st (scale 2000 direc))))
          ret picked
          )
     (bodies-vector-limit-face-pos body-list
                                   (send eye :viewpoint)
                                   (send eye :view-orient vp)
                                   #'<)
     ))
  (:pick-nearest-fixed-point
   (spos body-list)
   (let* ((eye (send self :viewer :viewing))
          (vp (send self :viewer :viewport :screen-point-to-ndc spos))
          (wc (send eye :worldcoords))
          (st (send eye :viewpoint))
          ;;(st (send eye :worldpos))
          (direc (send eye :view-orient vp))
          (vline (make-line st (v+ st (scale 20000 direc))))
          ret picked
          )
     (setq ret
           (mapcar #'(lambda (x) (let ((len (send vline :distance (send x :worldpos))))
                                   (if (< len 9.0) (list x (send vline :foot (send x :worldpos))))))
                   body-list))
     (setq picked (find-extream ret #'(lambda (x) (if x (cadr x) 1.0)) #'<))
     (when picked
       (search-assoced-fixed-point (car picked)))
     ))
  (:select-fixed-point
   (&rest args)
   (let ((body-list (remove-if-not #'(lambda (x) (send x :get :related-fixed-point)) x::objects))
         picked)
     #|
     (setq picked
           (send self :pick-face
                 (float-vector (elt x::previous-cursor-pos 0) (elt x::previous-cursor-pos 1) 0)
                 body-list))
     (print picked)
     (when picked
       (let ((bd ((car picked) . mbody)))
         (search-assoced-fixed-point bd)))
     |#
     (send self :pick-nearest-fixed-point
           (float-vector (elt x::previous-cursor-pos 0) (elt x::previous-cursor-pos 1) 0)
           body-list)
     ))
  )

(defun bodies-vector-limit-face-pos (bb &optional (pos (float-vector 0 0 -10000))
                                        (v (float-vector 0 0 1)) (test #'>))
  (find-extream
   (apply #'append
          (send-all bb :faces-intersect-with-point-vector pos v))
   #'(lambda (x) (if x (v. (cadr x) v) 0.0))
   test))

(defun search-assoced-fixed-point (fmarker)
  (let* ((fp (get fmarker :related-fixed-point)))
    (cond
     ((and fp (not (or (eq *current-robot-point* fp)
                       (eq *current-parts-point* fp)))) ;; new point
      (cond
       ((and *current-parts* (memq fp (send *current-parts* :all-parts)))
        (when *current-parts-point*
          ;; normal light
          )
        (setq *current-parts-point* fp)
        ;; high light
        (change-colors :look-all nil)
        )
       ((and *current-robot* (memq fp (send *current-robot* :all-parts roboasm-fixed-point)))
        (when *current-robot-point*
          ;; normal light
          )
        (setq *current-robot-point* fp)
        ;; high light
        (change-colors :look-all nil)
        )
       ))
     ((and fp (eq *current-robot-point* fp)) ;; pick same
      ;; toggle candidates
      )
     ((and fp (eq *current-parts-point* fp)) ;; pick same
      ;; toggle candidates
      )
     )
    ))
;;
;; match check
;;
(defun change-colors (&key (look-all t))
  (let (objs)
    (when *current-robot*
      (setq objs
            (append objs (change-obj-colors *current-robot* *current-robot-point*)))
      )
    (when *current-parts*
      (setq objs
            (append objs (change-obj-colors *current-parts* *current-parts-point*)))
      )
    (send *robot-assembler-viewer* :objects objs)
    (when look-all (send *robot-assembler-viewer* :look-all))
    ))
(defun change-obj-colors (r cur-point)
  (let ((bds (list r)))
    (dolist (p (send r :all-active-fixed-points roboasm-fixed-point)) ;;(send r :all-parts roboasm-fixed-point))
      (when (eq roboasm-fixed-point (class p))
        (let ((tp-lst (p . types))
              ;;(hlightq (eq p cur-point))
              (bd (make-default-fixed-point-type-geometry p))
              )
          (cond
           ((eq p cur-point) (send bd :set-color #f(1.0 0.0 0.0)))
           (t                (send bd :set-color #f(0.3 0.3 0.6)))
           )
          (push bd bds)
          )
        )
      )
    bds))

;;
;; service function
;;
(defun make-parts-from-name-string (parts-key)
  (let ((fsymbol
         (gethash parts-key *robot-assembler-functions-htable*))
        (pname
         (intern (string-upcase (format nil "~A_roboasm~D" *parts-create-counter*)) *keyword-package*))
        )
    (cond
     ((fboundp fsymbol)
      (funcall fsymbol :name pname)
      (incf *parts-create-counter*)
      )
     (t (warn ";; do not find function for ~A~%" parts-key)))
    ))
(defun start-with-parts (parts-key)
  (let ((iparts (make-parts-from-name-string parts-key)))
    (when iparts
      (setq *current-robot* (make-robot-assembler-robot iparts :name "created_by_viewer")))
    ))
(defun change-current-parts (parts-key)
  (let ((iparts (make-parts-from-name-string parts-key)))
    (setq *current-parts* iparts)
    ))
(defun parts-name->parts-key (parts-name)
  (let ((resolved-name (string parts-name)))
    (dotimes (i (length parts-name))
      (when (= (elt resolved-name i) #\_)
        (setf (elt resolved-name i) #\-)))
    (intern (string-upcase resolved-name) *keyword-package*)
    ))
;;
(defun callback-use-service-common (parts-name req)
  ;;(warn ";; ~A~%" parts-name)
  (let ((parts-key (parts-name->parts-key parts-name)))
    (cond
     ((null *current-robot*)
      (let ((ret (start-with-parts parts-key)))
        (when ret
          (setq *current-robot-point* nil))
        ))
     (t
      (let ((ret (change-current-parts parts-key)))
        (when ret
          (setq *current-parts-point* nil))
        ))
     )
    )
  ;; fix view size and locate parts
  (let (bb)
    (cond
     ((and *current-robot* (null *current-parts*))
      (setq bb (make-bounding-box (flatten (send-all (send *current-robot* :bodies) :vertices)) 0.0))
      (send bb :grow 50 t)
      )
     ((and *current-robot* *current-parts*)
      (let* ((r-bb (make-bounding-box (flatten (send-all (send *current-robot* :bodies) :vertices)) 0.0))
             (p-bb (make-bounding-box (flatten (send-all (send *current-parts* :bodies) :vertices)) 0.0))
             diag minp
             )
        (send r-bb :grow 30 t)
        (send p-bb :grow 30 t)
        (setq diag (coerce (send r-bb :diagonal) cons)
              minp (position (apply #'min diag) diag))
        (if (> (abs (elt (send r-bb :minpoint) minp))
               (abs (elt (send r-bb :maxpoint) minp)))
            (let ((fv (float-vector 0 0 0)))
              (setf (elt fv minp)
                    (- (elt (send r-bb :minpoint) minp) (elt (send p-bb :maxpoint) minp)))
              (send *current-parts* :translate fv))
          (let ((fv (float-vector 0 0 0)))
            (setf (elt fv minp)
                  (- (elt (send r-bb :maxpoint) minp) (elt (send p-bb :minpoint) minp)))
            (send *current-parts* :translate fv))
          )
        (setq bb
              (make-bounding-box
               (flatten (append (send-all (send *current-robot* :bodies) :vertices)
                                (send-all (send *current-parts* :bodies) :vertices)))
               0.0))
        (send bb :grow 50 t)
        ))
     (t
      ;;error ?
      ))
    (change-colors :look-all nil)
    (send *robot-assembler-viewer* :look-all bb)
    )
  ;;
  (send req :response)
  )
(defun callback-align (req)
  (when (and *current-robot* *current-robot-point*
             *current-parts* *current-parts-point*)
    (cond
     (*current-configuration*
      ;; already aligned -> next configuration
      )
     (t
      ;; align default configuration
      )
     )
    )
  (send req :response)
  )
(defun callback-attach (req)
  (when (and *current-robot* *current-robot-point*
             *current-parts* *current-parts-point*)
    (let (ret)
      (setq ret
            (send *current-robot* :attach *current-parts*
                  :robot-point *current-robot-point*
                  :parts-point *current-parts-point*))
      (cond
       (ret
        (setq *current-parts* nil
              *current-parts-point* nil
              *current-robot-point* nil)
        (push (list (list :robot-parts-point )
                    (list :robot-parts-name )
                    (list :parts-name )
                    (list :parts-type )
                    (list :parts-point )
                    (list :configuration )) *attach-history*)
        (setq *current-robot* ret)
        (send *robot-assembler-viewer* :objects (list *current-robot*))
        (change-colors)
        (let ((bb (make-bounding-box
                   (flatten (send-all (send *current-robot* :bodies) :vertices)) 0.0)))
          (send bb :grow 50 t)
          (send *robot-assembler-viewer* :look-all bb))
        )
       (t
        ;; attach error
        )
      ))
    )
  (send req :response)
  )

(defun callback-write-urdf (req)
  (when *current-robot*
    (send *current-robot* :write-urdf "/tmp/hoge.urdf"))
  ;;
  (send req :response)
  )

(defun callback-delete-all (req)
  (setq *current-robot* nil)
  (setq *current-robot-point* nil)
  (setq *current-parts* nil)
  (setq *current-parts-point* nil)

  (send *robot-assembler-viewer* :objects nil)
  (send *robot-assembler-viewer* :look-all
        (make-bounding-box (list #f(-300 -300 -300) #f(300 300 300))))
  ;;
  (send req :response)
  )

(defun robot-assembler-viewer-main (&key (node-name "robot_assembler"))
  (ros::roseus node-name :anonymous nil)

  ;; parts TODO:use param
  (robot-assembler-initialize "robot_assembler_config.yaml")

  ;; viewer
  (make-robot-asselbler-viewer)

  (ros::advertise-service "~/delete_all" std_srvs::Empty #'callback-delete-all)
  (ros::advertise-service "~/write_urdf" std_srvs::Empty #'callback-write-urdf)
  (ros::advertise-service "~/attach" std_srvs::Empty #'callback-attach)
  (dolist (parts-name (list "xm430"  ;; TODO: use param
                            "fr12_h101" "fr12_h102" "fr12_h103" ;; "fr12_h104"
                            "fr12_s101" "fr12_s102" "fr12_s103"
                            "fr12_x102" "fr12_x103"
                            "hr12_f101"
                            "fr12_e170" "fr12_f171"))
    (ros::advertise-service (format nil "~~/use_~A" parts-name)
                            std_srvs::Empty #'callback-use-service-common parts-name)
    )

  (ros::rate 120)
  (while (ros::ok)
    (ros::spin-once)
    (x::window-main-one)
    (send *robot-assembler-viewer* :draw-objects)
    (ros::sleep))

  )
