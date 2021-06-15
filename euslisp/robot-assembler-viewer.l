;;
;; robot-assembler-viewer
;;
;; GUI 3D viewer and functions for parsing command
;;
(eval-when (load)
;; compiled code
(require :robot-assembler "package://robot_assembler/euslisp/robot-assembler")
)
(eval-when (eval compile)
;; source code
(require :robot-assembler       "robot-assembler.l")
(require :robot-assembler-utils "robot-assembler-utils.l")
)
(defvar *robot-assembler-viewer* nil)
(defvar *current-robot*       nil)
(defvar *current-robot-point* nil)
(defvar *current-parts*       nil)
(defvar *current-parts-point* nil)
(defvar *current-configuration* nil)
;;
(defvar *attach-history*       nil)
(defvar *parts-create-counter* 0)
;;
(defvar *select-mode* :fixed-point)
;;
(defvar *selected-actuator* nil)
(defvar *selected-parts*    nil)
(defvar *selected-link*     nil)

;;
(defvar *parts-id-prefix* 0)
(defvar *output-dir* "/tmp")
(defvar *robot-name* "assembled_robot")

(defun make-robot-assembler-viewer (&key (width 1200) (height 900) (background (float-vector 0.91 0.91 0.93)))
  ;;
  (setq *robot-assembler-viewer*
        (instance robot-assembler-viewer :create
                  :name "RobotAssemblerViewer"
                  :title "RobotAssemblerViewer"
                  :width height :height height))
  (send *robot-assembler-viewer* :change-background background)

  (x::window-main-one)
  (send *robot-assembler-viewer* :resize width height) ;; require for viewport aspect
  (x::window-main-one)
  ;;
  (setq *current-robot* nil)
  (setq *current-robot-point* nil)
  (setq *current-parts* nil)
  (setq *current-parts-point* nil)
  (setq *current-configuration* nil)
  ;;
  (setq *attach-history* nil)
  (setq *parts-create-counter* 0)
  ;;
  (setq *select-mode* :fixed-point)
  ;;
  (setq *selected-actuator* nil)
  (setq *selected-parts* nil)
  (setq *selected-link* nil)
  )

(defclass robot-assembler-viewer :super x::irtviewer
  :slots (pick-face-pos pick-facesets button-mode))

(defmethod robot-assembler-viewer
  (:create ;; overwrite
   (&rest args)
   (prog1 (send-super* :create args)
     ;;(setq button-mode :view) ;; :view, :model, :design
     (setq button-mode :view)
     (send self :set-button-mode button-mode)
     )
   )
  (:draw-objects ;; overwrite
   (&rest args)
   (send x::viewer :viewsurface :makecurrent)
   (if x::logging-flag (send self :push-image))
   (apply #'gl::draw-globjects x::viewer x::draw-things
          :draw-origin x::draw-origin
          :draw-floor  x::draw-floor
          :floor-color x::floor-color
          :flush nil args)
   ;;(when *selected-link* )
   (when *selected-parts*
     (dolist (e (send *selected-parts* :box :edges))
       (send e :draw-on :color (float-vector 0 1 0) :flush nil)
       ))
   (let ((existf (memq :flush args)))
     (if (or (not existf) (cadr existf))
         (send x::viewer :viewsurface :flush)))
   )
  (:do-nothing (&rest args) )
  (:set-button-mode
   (mode)
   (case mode
     (:view
      ;; :left move-view, :right assembler-button
      (send x::viewer :viewsurface :replace-event-proc :buttonrelease-left
            :draw-event self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-left
            :move-coords-event self)
      (send x::viewer :viewsurface :replace-event-proc :buttonrelease-right
            :right-button-selector self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-right
            :do-nothing self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-middle
            :move-coords-event self)
      ;;(send viewer :viewsurface :replace-event-proc :buttonpress
      ;;:set-cursor-pos-event self)
      )
     (:model
      ;; :left assembler-button, :right move-view
      (send x::viewer :viewsurface :replace-event-proc :buttonrelease-left
            :right-button-selector self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-left
            :do-nothing self)
      (send x::viewer :viewsurface :replace-event-proc :buttonrelease-right
            :draw-event self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-right
            :move-coords-event self)
      ;;(send x::viewer :viewsurface :replace-event-proc :motionnotify-middle
      ;;:move-coords-event self)
      )
     (:design
      ;; not implemented
      (send x::viewer :viewsurface :replace-event-proc :buttonrelease-left
            :right-button-selector self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-left
            :do-nothing self)
      (send x::viewer :viewsurface :replace-event-proc :buttonrelease-right
            :draw-event self)
      (send x::viewer :viewsurface :replace-event-proc :motionnotify-right
            :move-coords-event self)
      ;;(send x::viewer :viewsurface :replace-event-proc :motionnotify-middle
      ;;:move-coords-event self)
      )
     (t
      (warn "unknown button-mode ~A~%" mode)
      )
     )
   )
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
  (:pick-nearest-fixed-point
   (spos body-list &key (pick-threshold 3.0))
   (let* ((eye (send self :viewer :viewing))
          (vp (send self :viewer :viewport :screen-point-to-ndc spos))
          ;; (wc (send eye :worldcoords))
          (st (send eye :viewpoint))
          ;;(st (send eye :worldpos))
          (direc (send eye :view-orient vp))
          (vline (make-line st (v+ st (scale 20000 direc))))
          ret picked
          )
     ;;(send self :objects (axis-box direc 1000 st)) ;; debug
     (setq ret
           (mapcar #'(lambda (x) (let ((len (send vline :distance (send x :worldpos))))
                                   (if (< len pick-threshold) (list x (send vline :foot (send x :worldpos))))))
                   body-list))
     (setq ret (remove nil ret))
     ;;(setq picked (find-extream ret #'(lambda (x) (if x (cadr x) 1.0)) #'<))
     (if ret (sort ret #'(lambda (x y) (< (cadr x) (cadr y)))))
     ret))
  (:look-all
   (&optional bbox)
   (cond
    ((null bbox)
     (send-all x::draw-things :worldcoords)
     (let ((vs (flatten (collect-vertices x::draw-things))))
       (if vs
           (setq bbox (geo::make-bounding-box vs 0.2))
         (setq bbox nil)))
     )
    ((numberp bbox)
     (send-all x::draw-things :worldcoords)
     (let ((vs (flatten (collect-vertices x::draw-things))))
       (if vs
           (setq bbox (geo::make-bounding-box vs bbox))
         (setq bbox nil)))
     )
    )
   (if bbox (send-super :look-all bbox))
   )
  (:right-button-selector (event)
   (cond
    ((eq *select-mode* :fixed-point)
     (send self :select-fixed-point event)
     )
    ((eq *select-mode* :actuator)
     (send self :select-actuator event)
     )
    ((eq *select-mode* :parts)
     ;;(warn "not implemented yet~%")
     (send self :select-parts event)
     )
    ))
  (:select-fixed-point (event)
   (let ((body-list (remove-if-not #'(lambda (x) (send x :get :related-fixed-point)) x::objects))
         picked)
     (setq picked
           (send self :pick-nearest-fixed-point
                 (float-vector (elt x::previous-cursor-pos 0) (elt x::previous-cursor-pos 1) 0)
                 body-list))
     (when picked
       #|
       (let ((fp (get (car picked) :related-fixed-point)))
         (format t "picked: ~A / ~A~%" (send fp :name)
                 (write-coords-as-string (send fp :worldcoords)))
         )
       |#
       (search-assoced-fixed-point picked)
       )
     ))
  (:select-actuator
   (event)
   (let ((body-list (remove-if-not #'(lambda (x) (send x :get :related-fixed-point)) x::objects))
         picked)
     (setq picked
           (send self :pick-nearest-fixed-point
                 (float-vector (elt x::previous-cursor-pos 0) (elt x::previous-cursor-pos 1) 0)
                 body-list :pick-threshold 4.5))
     (when picked
       (setq *selected-actuator* (send (caar picked) :get :related-fixed-point))
       (print-information-gui (format nil ":clear~%selected: ~A" *selected-actuator*))
       (draw-actuator :look-all nil)
       )
     ))
  (:select-parts
   (event)
   (let* ((body-list (if *current-robot* (send *current-robot* :all-parts)))
          (spos (float-vector (elt x::previous-cursor-pos 0) (elt x::previous-cursor-pos 1) 0))
          (eye (send self :viewer :viewing))
          (vp (send self :viewer :viewport :screen-point-to-ndc spos))
          ;; (wc (send eye :worldcoords))
          (pos (send eye :viewpoint))
          (vec (send eye :view-orient vp))
          bd-dist-list)
     (setq bd-dist-list
           (mapcan #'(lambda (bd)
                       ;;(send bd :worldcoords)
                       (let ((d (send bd :nearest-intersect-distance pos vec)))
                         (if d (list (cons bd d))))) body-list))
     (if bd-dist-list
         (setq *selected-parts* (caar (find-extreams bd-dist-list :key #'cdr :bigger #'<))))
     (print-information-gui (format nil ":clear~%selected: ~A" *selected-parts*))
     )
   )
  (:modify-parts
   (event)
   )
  )
(defun search-assoced-fixed-point (fmarkers)
  (let* ((fp (get (caar fmarkers) :related-fixed-point))
         (mks fmarkers)
         )
    (while mks
      (cond
       ((and fp (not (or (eq *current-robot-point* fp)
                         (eq *current-parts-point* fp)))) ;; new point
        (cond
         ((and *current-parts* (memq fp
                                     (if (derivedp *current-parts* roboasm-robot)
                                         (send *current-parts* :all-parts roboasm-fixed-point)
                                       (send *current-parts* :all-parts))) )
          (when *current-parts-point*
            ;; change
            )
          (setq *current-parts-point* fp)
          (setq *current-configuration* nil)
          ;; high light
          (draw-fixed-points :look-all nil :after-pick t :pick-robot nil)
          (return)
          )
         ((and *current-robot* (memq fp (send *current-robot* :all-parts roboasm-fixed-point)))
          (when *current-robot-point*
            ;; normal light
            )
          (setq *current-robot-point* fp)
          (setq *current-configuration* nil)
          ;; high light
          (draw-fixed-points :look-all nil :after-pick t :pick-robot t)
          (return)
          )
         (t
          (warn ";; select error ~A~%" (list fp *current-robot-point* *current-parts-point*))
          )
         )
        (pprint (list fp *current-robot-point* *current-parts-point*))
        )
       ((and fp (eq *current-robot-point* fp)) ;; pick same
        ;; toggle
        ;;(pprint (list :tgr fp))
        )
       ((and fp (eq *current-parts-point* fp)) ;; pick same
        ;; toggle
        ;;(pprint (list :tgp fp))
        )
       )
      (setq mks (cdr mks))
      (unless mks (return))
      (setq fp (get (caar mks) :related-fixed-point))
      )
    (print-information-gui (format nil ":clear~%selected: ~A" fp))
    ))
;;
;; match check
;;
(defun draw-fixed-points (&key (look-all t) (after-pick) (pick-robot))
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
    (if look-all (send *robot-assembler-viewer* :look-all))
    ))
(defun draw-actuator (&key (look-all t) (after-pick) (pick-robot))
  (let (hns objs target-types can-attach)
    (when *current-parts-point*
      (push *current-parts-point* objs)
      (if (derivedp *current-parts-point* roboasm-actuator)
          (setq hns (send *current-parts-point* :horns)))
      )
    (when *current-robot*
      (push *current-robot* objs)
      (setq hns (append hns (send *current-robot* :not-active-horns))))
    (dolist (hn hns)
      (let ((geom-hn (make-default-fixed-point-type-geometry
                      hn :base 3.5 :x-len 9.0 :y-len 2.0 :z-len 18.0)))
        (if (eq hn *selected-actuator*)
            (send geom-hn :set-color #f(1 1 0))
          (send geom-hn :set-color #f(0.6 0.3 0)))
        (push geom-hn objs)
        ))
    (send *robot-assembler-viewer* :objects objs)
    (if look-all (send *robot-assembler-viewer* :look-all))
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
         (intern (string-upcase (format nil "~A_~A_~D"
                                        (string parts-key)
                                        (string *parts-id-prefix*)
                                        *parts-create-counter*))
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
      (if *robot-name*
          (setq *current-robot* (make-robot-assembler-robot iparts :name *robot-name*))
        (setq *current-robot* (make-robot-assembler-robot iparts :name "created_by_viewer"))))
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
(defun parts-key->parts-name (key-name)
  (if (stringp key-name) key-name
    (let ((resolved-name (string key-name)))
      (dotimes (i (length resolved-name))
        (when (= (elt resolved-name i) #\-)
          (setf (elt resolved-name i) #\_)))
      (string-downcase resolved-name))))
(defun subfunction-look-robot-and-parts ()
  (let (bb)
    (prog1
        (cond
         ((and *current-robot* (null *current-parts*))
          (setq bb (make-bounding-box (flatten (collect-vertices (send *current-robot* :bodies))) 0.0))
          (send bb :grow 50 t)
          t)
         ((and *current-robot* *current-parts*)
          (let* ((r-bb (make-bounding-box (flatten (collect-vertices (send *current-robot* :bodies))) 0.0))
                 (p-bb (make-bounding-box (flatten (collect-vertices (send *current-parts* :bodies))) 0.0))
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
                   (flatten (append (collect-vertices (send *current-robot* :bodies))
                                    (collect-vertices (send *current-parts* :bodies))))
                   0.0))
            ;;(send bb :grow 50 t)
            (send bb :grow 0.4)
            )
          t)
         (t
          ;;error ?
          nil))
      (draw-fixed-points :look-all nil)
      (send *robot-assembler-viewer* :look-all bb)
      )
    ))
(defun function-use-service-common (parts-name)
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
     ))
  ;; fix view size and locate parts
  (subfunction-look-robot-and-parts))

(defun callback-use-service-common (parts-name req)
  (function-use-service-common parts-name)
  (if req (send req :response))
  )
(defun function-unalign (&rest args)
  (when *current-parts*
    (callback-use-service-common (*current-parts* . type) nil)
    t))
(defun callback-unalign (req)
  (function-unalign)
  (if req (send req :response))
  )
(defun function-align (&rest args)
  (function-attach :just-align t :configuration nil)
  (draw-fixed-points)
  t)
(defun callback-align (req)
  (function-align)
  (if req (send req :response))
  )
(defun function-attach (&key (just-align) (configuration))
  (when (and *current-robot* *current-robot-point*
             *current-parts* *current-parts-point*)
    (if (derivedp *current-configuration* coordinates)
        (setq *current-configuration* nil))
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
      (cond
       (configuration (setq use-config configuration))
       (use-config    (setq use-config (use-config . type))))
      (setq ret
            (send *current-robot* :attach *current-parts* :just-align just-align
                  :robot-point *current-robot-point*
                  :parts-point *current-parts-point*
                  :attach-configuration (if use-config use-config)))
      ;;
      (when just-align
        ;; draw?
        (return-from function-attach t))
      ;;
      (cond
       ((and ret (derivedp *current-parts* roboasm-robot))
        (let ((root-parts (car (send *current-parts-point* :descendants))))
          (setq *attach-history*
                (append (nreverse (send root-parts :roboasm-from-parts)) *attach-history*))
          ))
       (ret ;; *current-parts* is roboasm-parts
        (push (list (list :robot-parts-point (send *current-robot-point* :name))
                    ;;(list :robot-parts-name  (*current-robot-point* . name))
                    (list :parts-name (send *current-parts* :name))
                    (list :parts-type (*current-parts* . type))
                    (list :parts-point (send *current-parts-point* :name))
                    (list :configuration (if use-config
                                             (if (derivedp use-config coordinates)
                                                 (write-coords-as-string use-config)
                                               use-config)
                                           :default))
                    ) *attach-history*)
        ))
      (when ret
        (setq *current-parts* nil
              *current-parts-point* nil
              *current-robot-point* nil
              *current-configuration* nil
              )
        (setq *current-robot* ret)
        (send *robot-assembler-viewer* :objects (list *current-robot*))
        (draw-fixed-points)
        (send *current-robot* :worldcoords)
        (let ((bb (make-bounding-box
                   (flatten (collect-vertices (send *current-robot* :bodies))) 0.0)))
          (send bb :grow 50 t)
          (send *robot-assembler-viewer* :look-all bb))
        t))
    ))
(defun callback-attach (req &optional (just-align) (configuration))
  (function-attach :just-align just-align :configuration configuration)
  (if req (send req :response))
  )
(defun function-write-files (&rest args)
  (when *current-robot*
    ;; write urdf
    (send *current-robot* :write-urdf
          (format nil "~A/~A.urdf" *output-dir* (send *current-robot* :name))
          :robotname (send *current-robot* :name)
          :package-prefix "package://robot_assembler")
    (message-info ";; OUTPUT: ~A/~A.urdf" *output-dir* (send *current-robot* :name))
    ;; write history
    (with-open-file
     (ff (format nil "~A/~A.roboasm.l" *output-dir* (send *current-robot* :name))
         :direction :io)
     (let* ((ac (send *current-robot* :assemble-config))
            (hist (reverse (append (if ac (list ac)) *attach-history*)))
            )
       #|(setq hist
             (cons
              (append (car hist)
                      (list (list :yaml-string *current-setting-yaml*)))
              (cdr hist)))|#
       (pprint hist ff)
       ))
    (message-info ";; OUTPUT: ~A/~A.roboasm.l" *output-dir* (send *current-robot* :name))
    t))
(defun callback-write-urdf (req)
  (function-write-files)
  (if req (send req :response))
  )
(defun function-delete-all (&rest args)
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
  t
  )
(defun callback-delete-all (req)
  (function-delete-all)
  (if req (send req :response))
  )
(defun function-undo (&rest args)
  (when (>= (length *attach-history*) 2)
    (start-with-history (reverse (cdr *attach-history*)))
    t))
(defun callback-undo (req)
  (function-undo)
  (if req (send req :response))
  )
(defun function-parse-robot (&rest args)
  (when *current-robot*
    (send *current-robot* :write-urdf nil)
    t))
(defun callback-parse (req)
  (function-parse-robot)
  (if req (send req :response))
  )
(defun callback-reload (req)
  ;; reload ...
  (if req (send req :response))
  )
(defun start-with-history (hist &key (parse t))
  (let ((rb
         (parse-roboasm-list hist)))
    (send rb :name *robot-name*)
    (when rb
      (setq *current-robot*       rb
            *current-parts*       nil
            *current-robot-point* nil
            *current-parts-point* nil
            *current-configuration* nil
            *selected-actuator* nil
            *selected-parts*    nil
            *selected-link*     nil)
      ;; remove
      (let ((aconfig (assoc :assemble-config hist)))
        (setq hist (remove aconfig hist)))
      ;;
      (setq *attach-history* (reverse hist))
      (if parse (send *current-robot* :write-urdf nil))
      (draw-fixed-points)
      )
    ))
;;
;; compatibility with body:vertices
;;
(defmethod gl::glvertices
  (:vertices-compatible ()
   (unless gl::bbox
     (send self :calc-bounding-box))
   (mapcar #'(lambda (p) (send self :transform-vector p))
           (send (send gl::bbox :body) :vertices))
   ))
(defun collect-vertices (bds)
  (mapcar #'(lambda (b)
              (if (derivedp b gl::glvertices)
                  (send b :vertices-compatible)
                (send b :vertices))) bds))
;;
;; new feature
;;
#|
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
|#
;;
(defun function-select-mode (key)
  (setq *select-mode* key)
  (print-information-gui (format nil ":clear~%"))
  (cond
   ((eq *select-mode* :fixed-point)
    (setq *current-robot-point* nil)
    (setq *current-parts-point* nil)
    (draw-fixed-points :look-all t)
    )
   ((eq *select-mode* :actuator)
    (setq *selected-actuator* nil)
    (draw-actuator :look-all t)
    )
   ((eq *select-mode* :parts)
    (setq *selected-parts* nil)
    )
   ((eq *select-mode* :link)
    (setq *selected-link* nil)
    ))
  t)
(defun callback-select (key req)
  (function-select-mode key)
  (if req (send req :response))
  )
(defun function-mouse-mode (key)
  (print-information-gui (format nil ":mode ~A" key))
  (send *robot-assembler-viewer* :set-button-mode key)
  t)
(defun callback-mode (key req)
  (function-mouse-mode key)
  (if req (send req :response))
  )

(defun function-command (com)
  (if (< (length com) 2)
      (return-from function-command nil))
  (print-information-gui (format nil "command received / ~A" com))
  (cond
   ((eq *select-mode* :fixed-point)
    (parse-command-at-fixedpoint com))
   ((eq *select-mode* :actuator)
    (parse-command-at-actuator com))
   ((eq *select-mode* :parts)
    (parse-command-at-parts com))
   ((eq *select-mode* :link)
    (parse-command-at-link com))
   ))
(defun callback-command (req)
  (function-command (send req :command))
  (if req (send req :response))
  )
(defun com-set-actuator-name (act-nm new-nm &rest args)
  (if (send *current-robot* :add-actuator-name act-nm new-nm)
      (print-information-gui (format nil "success :add-actuator-name ~A ~A" act-nm new-nm))
    (print-information-gui (format nil "failed :add-actuator-name ~A ~A" act-nm new-nm))))
(defun com-set-initial-angle (act-nm angle &rest args)
  (if (send *current-robot* :add-initial-angle act-nm angle)
      (print-information-gui (format nil "success :add-actuator-name ~A ~A" act-nm angle))
    (print-information-gui (format nil "failed :add-actuator-name ~A ~A" act-nm angle))))
(defun com-set-actuator-axis (act-nm axis &rest args)
  (if (send *current-robot* :add-actuator-axis act-nm axis)
      (print-information-gui (format nil "success :add-actuator-axis ~A ~A" act-nm axis))
    (print-information-gui (format nil "failed :add-actuator-axis ~A ~A" act-nm axis))))
(defun com-robot-name (nm &rest args)
  (cond
   (nm
    (setq *robot-name* nm)
    (send *current-robot* :name nm)
    (print-information-gui (format nil "success :robot-name ~A" nm))
    )
   (t
    (print-information-gui (format nil "fail :robot-name ~A" nm))
    )
   ))
(defun com-set-output-dir (dirname)
  (cond
   (dirname
    (setq *output-dir* dirname)
    (print-information-gui (format nil ";; output-directory is ~A" *output-dir*))
    )
   (t
    (print-information-gui (format nil ";; output-directory is ~A" *output-dir*))
    )
   ))
(defun com-open-roboasm (fname)
  (cond
   ((eq (probe-file fname) :file)
    (with-open-file (f fname)
                    (start-with-history (read f)))
    )
   (t
    (print-information-gui (format nil ";; file not found ~A" fname))
    )
   ))
(defun com-open-as-parts (fname)
  (cond
   ((null *current-robot*)
    (com-open-roboasm fname)
    )
   ((eq (probe-file fname) :file)
    (let ((asm (with-open-file (f fname) (read f)))
          renamed rb)
      (setq renamed
            (create-roboasm-from-parsed-table
             (rename-parts-name-on-parsed-table
              (parse-roboasm asm))))
      (setq rb (parse-roboasm-list renamed))
      (let ((aconfig (assoc :assemble-config renamed))) ;; remove...
        (setq renamed (remove aconfig renamed)))
      ;;
      (when rb
        ;;
        (warn ";; ~A as parts~%" rb)
        (setq *current-parts* rb)
        (setq *current-parts-point* nil)
        (subfunction-look-robot-and-parts)
        )
      ))
   (t
    (print-information-gui (format nil ";; file not found ~A" fname))
    ))
  )
(defun com-set-initial-coords (exp)
  (if (stringp exp)
      (setq exp (read-from-string exp)))
  (let ((cds (eval exp)))
    (when (and *current-robot* cds)
      (send *current-robot* :set-initial-coords cds))
    ))
(defun parse-command-common (exp)
  (cond
   ((eq (car exp) :list-actuator)
    (when *current-robot*
      (let ((sstrm (make-string-output-stream)))
        (dolist (hn (send *current-robot* :all-horns))
          (when (send hn :descendants)
            (let* ((nm (send hn :name))
                   (anm (send *current-robot* :get-actuator-name nm)))
              (format sstrm "~A" nm)
              (if anm (format sstrm ":= ~A~%" anm) (format sstrm "~%"))
              )))
        (print-information-gui (get-output-stream-string sstrm))
        ))
    )
   ((eq (car exp) :list-parts)
    )
   ((eq (car exp) :list-joint)
    )
   ((eq (car exp) :list-link)
    )
   ((eq (car exp) :free-actuator)
    )
   ((eq (car exp) :free-fixed-point)
    )
   ((eq (car exp) :robot-name)
    (let ((nm (cadr exp)))
      (com-robot-name nm))
    )
   ((eq (car exp) :set-actuator-name)
    (let ((act-nm (cadr exp))
          (new-nm (caddr exp)))
        (com-set-actuator-name act-nm new-nm))
    )
   ((eq (car exp) :set-initial-angle)
    (let ((act-nm (cadr exp))
          (ang (caddr exp)))
        (com-set-initial-angle act-nm ang))
    )
   ((eq (car exp) :set-actuator-axis)
    (let ((act-nm (cadr exp))
          (axis (cadr exp)))
      (com-set-actuator-axis act-nm axis))
    )
   ((or (eq (car exp) :open-as-parts)
        (eq (car exp) :load-as-parts))
    (let ((fname (cadr exp)))
      (com-open-as-parts fname))
    )
   ((or (eq (car exp) :open-roboasm)
        (eq (car exp) :load-roboasm))
    (let ((fname (cadr exp)))
      (com-open-roboasm fname))
    )
   ((eq (car exp) :output-directory)
    (let ((dirname (cadr exp)))
      (com-set-output-dir dirname))
    )
   ((eq (car exp) :select-fixed-point)
    )
   ((eq (car exp) :select-parts)
    )
   ((eq (car exp) :initial-coords)
    (let ((cds (cadr exp)))
      (com-set-initial-coords cds))
    )
   (t
    (print-information-gui (format nil ";; unknown command ~A" exp))
    nil)
   ))
(defun parse-command-at-fixedpoint (com)
  (let ((exp
         (if (= (elt com 0) #\()
             (read-from-string com)
           (read-from-string (format nil "(~A)" com))))
        )
    (cond
     ((eq (car exp) :set-angle)
      ;; not implemented
      )
     (t
      (parse-command-common exp))
     )
    ))
(defun parse-command-at-actuator (com)
  (let ((exp
         (if (= (elt com 0) #\()
             (read-from-string com)
           (read-from-string (format nil "(~A)" com))))
        (act-nm (if *selected-actuator* (send *selected-actuator* :name)))
        )
    (cond
     ((or (eq (car exp) :actuator-name)
          (eq (car exp) :joint-name)
          (eq (car exp) :set-name))
      (let (;;(act-nm (cadr exp))
            (new-nm (cadr exp)))
        (com-set-actuator-name act-nm new-nm)))
     ((eq (car exp) :initial-angle)
      (let (;;(act-nm (cadr exp))
            (ang (cadr exp)))
        (com-set-initial-angle act-nm ang)))
     ((eq (car exp) :actuator-axis)
      (let (;;(act-nm (cadr exp))
            (axis (cadr exp)))
        (com-set-actuator-axis act-nm axis)))
     (t
      (parse-command-common exp))
     )
    ))
(defun parse-command-at-parts (com)
  (let ((exp
         (if (= (elt com 0) #\()
             (read-from-string com)
           (read-from-string (format nil "(~A)" com))))
        ;; (if *selected-parts*
        )
    (cond
     (t
      (parse-command-common exp)
      )
     )
    ))
(defun parse-command-at-link (com)
  (let ((exp
         (if (= (elt com 0) #\()
             (read-from-string com)
           (read-from-string (format nil "(~A)" com))))
        ;; (if *selected-link*
        )
    (cond
     (t
      (parse-command-common exp))
     )
    ))

(in-package "X")
(defmethod Xwindow
  (:replace-event-proc (type method receiver)
   (let* ((procs (get self :event-procs))
          (tgt (assoc type procs)))
     (send self :put :event-procs
           (if tgt
               (cons (list type method receiver) (remove tgt procs))
             (cons (list type method receiver) procs)))
     )
   )
  )
(in-package "USER")
