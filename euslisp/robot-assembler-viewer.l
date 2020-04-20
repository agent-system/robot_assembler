(require :robot-assembler "robot-assembler.l")

(ros::roseus-add-srvs "std_srvs")

(defun make-robot-asselbler-viewer (&key (width 1200) (height 900))
  ;;
  (setq *robot-assembler-viewer*
        (instance robot-assembler-viewer :create
                  :name "Robot Assembler" :width height :height height))
  (send *robot-assembler-viewer* :resize width height) ;; require for viewport aspect
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
     ;;(send self :objects (axis-box direc 1000 st)) ;; debug
     (setq ret
           (mapcar #'(lambda (x) (let ((len (send vline :distance (send x :worldpos))))
                                   (if (< len 2.5) (list x (send vline :foot (send x :worldpos))))))
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
          ;; change
          )
        (setq *current-parts-point* fp)
        (setq *current-configuration* nil)
        ;; high light
        (change-colors :look-all nil :after-pick t :pick-robot nil)
        )
       ((and *current-robot* (memq fp (send *current-robot* :all-parts roboasm-fixed-point)))
        (when *current-robot-point*
          ;; normal light
          )
        (setq *current-robot-point* fp)
        (setq *current-configuration* nil)
        ;; high light
        (change-colors :look-all nil :after-pick t :pick-robot t)
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
(defun change-colors (&key (look-all t) (after-pick) (pick-robot))
  (let (objs target-types can-attach)
    (when (and *current-parts-point* *current-robot-point*)
      (setq can-attach
            (search-match-candidates (*current-parts-point* . types)
                                     (*current-robot-point* . types))))
    (when *current-robot*
      (if (and after-pick (not pick-robot))
          (setq target-types (search-target-types (*current-parts-point* . types))))
      (setq objs
            (append objs (change-obj-colors *current-robot* *current-robot-point*
                                            :can-attach can-attach
                                            :target-types target-types)))
      )
    (when *current-parts*
      (if (and after-pick pick-robot)
          (setq target-types (search-target-types (*current-robot-point* . types))))
      (setq objs
            (append objs (change-obj-colors *current-parts* *current-parts-point*
                                            :can-attach can-attach
                                            :target-types target-types)))
      )
    (send *robot-assembler-viewer* :objects objs)
    (when look-all (send *robot-assembler-viewer* :look-all))
    ))
(defun change-obj-colors (r cur-point &key (target-types) (can-attach))
  (let ((bds (list r)))
    (dolist (p (send r :active-fixed-points)) ;;(send r :all-parts roboasm-fixed-point))
      (when (eq roboasm-fixed-point (class p))
        (let ((tp-lst (p . types))
              hlightq
              (bd (make-default-fixed-point-type-geometry p :scale 0.7))
              )
          (when target-types
            (let (suc)
              (setq hlightq
                    (dolist (tp (p . types) suc)
                      (when (memq tp target-types) (setq suc t) (return))))))
          (cond
           ((and can-attach
                 (eq p cur-point))
                             (send bd :set-color #f(0.0 1.0 0.0)))
           ((eq p cur-point) (send bd :set-color #f(1.0 0.0 0.0)))
           (hlightq          (send bd :set-color #f(0.0 1.0 1.0)))
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
         (intern (string-upcase (format nil "~A_roboasm~D" (string parts-key) *parts-create-counter*))
                 *keyword-package*))
        )
    (cond
     ((fboundp fsymbol)
      (prog1 (funcall fsymbol :name pname :namespace t)
        (incf *parts-create-counter*))
      )
     (t (warn ";; do not find function for ~A~%" parts-key)))
    ))
(defun start-with-parts (parts-key)
  (let ((iparts (make-parts-from-name-string parts-key)))
    (when iparts
      (push (list (list :initial-parts t)
                  (list :parts-type (iparts . type))
                  (list :parts-name (send iparts :name))
                  ) *attach-history*)
      (setq *current-robot* (make-robot-assembler-robot iparts :name "created_by_viewer")))
    ))
(defun change-current-parts (parts-key)
  (let ((iparts (make-parts-from-name-string parts-key)))
    (setq *current-parts* iparts)
    ))
(defun parts-name->parts-key (parts-name)
  (if (keywordp parts-name) parts-name
    (let ((resolved-name (string parts-name)))
      (dotimes (i (length resolved-name))
        (when (= (elt resolved-name i) #\_)
          (setf (elt resolved-name i) #\-)))
      (intern (string-upcase resolved-name) *keyword-package*)
      ))
  )
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
        (send *current-parts* :worldcoords)
        (send *current-robot* :worldcoords)
        (setq bb
              (make-bounding-box
               (flatten (append (send-all (send *current-robot* :bodies) :vertices-compatible)
                                (send-all (send *current-parts* :bodies) :vertices-compatible)))
               0.0))
        ;;(send bb :grow 50 t)
        (send bb :grow 0.2)
        ))
     (t
      ;;error ?
      ))
    (change-colors :look-all nil)
    (send *robot-assembler-viewer* :look-all bb)
    )
  ;;
  (when req (send req :response))
  )
(defun callback-unalign (req)
  (when *current-parts*
    (callback-use-service-common (*current-parts* . type) nil)
    )
  ;;
  (when req (send req :response))
  )
(defun callback-align (req)
  (callback-attach nil t)
  (when req  (send req :response))
  )
(defun callback-attach (req &optional (just-align))
  (when (and *current-robot* *current-robot-point*
             *current-parts* *current-parts-point*)
    (let ((ret)
          (use-config
           (when *current-configuration*
             (when just-align (incf *current-configuration*)) ;; already aligned -> next configuration
             (let* ((a-config
                     (search-allowed-configuration
                      (*current-parts-point* . types) (*current-robot-point* . types)))
                    )
               (when a-config
                 (elt a-config (mod *current-configuration* (length a-config))))
               )))
          )
      (unless *current-configuration* (setq *current-configuration* 0))
      (setq ret
            (send *current-robot* :attach *current-parts* :just-align just-align
                  :robot-point *current-robot-point*
                  :parts-point *current-parts-point*
                  :attach-configuration (if use-config (use-config . type))))
      ;;
      (when just-align
        ;; draw?
        (return-from callback-attach (when req (send req :response))))
      ;;
      (cond
       (ret
        (push (list (list :robot-parts-point (send *current-robot-point* :name))
                    ;;(list :robot-parts-name  (*current-robot-point* . name))
                    (list :parts-name (send *current-parts* :name))
                    (list :parts-type (*current-parts* . type))
                    (list :parts-point (send *current-parts-point* :name))
                    (list :configuration (if use-config (use-config . type) :default))
                    ) *attach-history*)
        (setq *current-parts* nil
              *current-parts-point* nil
              *current-robot-point* nil
              *current-configuration* nil
              )
        (setq *current-robot* ret)
        (send *robot-assembler-viewer* :objects (list *current-robot*))
        (change-colors)
        (send *current-robot* :worldcoords)
        (let ((bb (make-bounding-box
                   (flatten (send-all (send *current-robot* :bodies) :vertices-compatible)) 0.0)))
          (send bb :grow 50 t)
          (send *robot-assembler-viewer* :look-all bb))
        )
       (t
        ;; attach error
        )
      ))
    )
  (when req (send req :response))
  )
(defun callback-write-urdf (req)
  (when *current-robot*
    (send *current-robot* :write-urdf
          (format nil "~A/~A.urdf" *output-dir* *robot-name*)))
  (pprint (reverse *attach-history*))
  ;;
  (when req (send req :response))
  )
(defun callback-delete-all (req)
  (setq *current-robot* nil)
  (setq *current-robot-point* nil)
  (setq *current-parts* nil)
  (setq *current-parts-point* nil)
  (setq *current-configuration* nil)

  (setq *attach-history* nil)
  (setq *parts-create-counter* 0)

  (send *robot-assembler-viewer* :objects nil)
  (send *robot-assembler-viewer* :look-all
        (make-bounding-box (list #f(-300 -300 -300) #f(300 300 300))))
  ;;
  (when req (send req :response))
  )
(defun callback-undo (req)
  (when (>= (length *attach-history*) 2)
    (let ((rb
           (parse-attach-history (reverse (cdr *attach-history*)))))
      (when rb
        (setq *attach-history (cdr *attach-history*))
        (setq *current-robot* rb
              *current-parts* nil
              *current-robot-point* nil
              *current-parts-point* nil
              *current-configuration* nil)
        (change-colors)
        )
      ))
  (when req  (send req :response))
  )

(defun robot-assembler-viewer-main (&key (node-name "robot_assembler"))
  (ros::roseus node-name :anonymous nil)

  (setq *project-dir* (ros::rospack-find "robot_assembler"))
  (setq *yaml-file* (format nil "~A/config/robot_assembler_parts_settings.yaml" *project-dir*))
  (setq *output-dir* "/tmp")
  (setq *robot-name* "assembled_robot")

  (if (ros::has-param "~parts_settings_yaml")
      (setq *yaml-file* (ros::get-param "~parts_settings_yaml")))
  (if (ros::has-param "~output_directory")
      (setq *output-dir* (ros::get-param "~output_directory")))
  (if (ros::has-param "~robot_name")
      (setq *robot-name* (ros::get-param "~robot_name")))

  (robot-assembler-initialize *yaml-file* :project-dir *project-dir*)

  ;; viewer
  (make-robot-asselbler-viewer)

  (ros::advertise-service "~/delete_all" std_srvs::Empty #'callback-delete-all)
  (ros::advertise-service "~/write_urdf" std_srvs::Empty #'callback-write-urdf)
  (ros::advertise-service "~/attach" std_srvs::Empty #'callback-attach)
  (ros::advertise-service "~/align" std_srvs::Empty #'callback-align)
  (ros::advertise-service "~/unalign" std_srvs::Empty #'callback-unalign)
  (ros::advertise-service "~/undo" std_srvs::Empty #'callback-undo)
  ;;
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

  (ros::rate 120)
  (while (ros::ok)
    (ros::spin-once)
    (x::window-main-one)
    (send *robot-assembler-viewer* :draw-objects)
    (ros::sleep))

  )

;;
(defmethod gl::glvertices
  (:vertices-compatible ()
   (unless gl::bbox
     (send self :calc-bounding-box))
   (mapcar #'(lambda (p) (send self :transform-vector p))
           (send (send gl::bbox :body) :vertices))
   ))
