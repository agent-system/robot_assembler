;;
;; robot-assembler
;;
;; Core robot-assembler class/method/functions
;;
(eval-when (load)
;; compiled code
(require :eus-assimp "package://eus_assimp/euslisp/eus-assimp.l")
(require :eus-yaml   "package://robot_assembler/euslisp/eus-yaml") ;; loading c-parse-yaml-file
(require :robot-assembler-utils "package://robot_assembler/euslisp/robot-assembler-utils")
)
(eval-when (eval compile)
;; source code
(require :eus-yaml   "eus-yaml.l")
(require :robot-assembler-utils "robot-assembler-utils.l")
)
(require :robot-assembler-gazebo "robot-assembler-gazebo.l")

(defvar *robot-assembler-debug* nil)

(defvar *project-dir* nil)
(defvar *fixed-point-htable* nil)
(defvar *fixed-point-match-htable* nil)
(defvar *pre-defined-configuration-htable* nil)
(defvar *robot-assembler-functions-htable* nil)

;;
;; class for robot-assembler
;;
(defclass fix-configuration
  :super coordinates
  :slots (type
          description)
  )
(defclass fixed-point-match
  :super propertied-object
  :slots (pair
          allowed-configuration)
  )
(defmethod fixed-point-match
  (:counter-part (p)
   (if (eq p (car p))
       (cadr p)
     (if (eq p (cadr p))
         (car p))
     ))
  )
(defun pair-test (x y)
  (if (or (atom x) (atom y))
      nil
    (or (and (eq (car x) (car y)) (eq (cadr x) (cadr y)))
        (and (eq (car x) (cadr y)) (eq (cadr x) (car y))))))
(defun pair-sxhash (x)
  (if (string<= (string (car x)) (string (cadr x)))
      (sxhash x)
    (sxhash (list (cadr x) (car x)))))
(defclass roboasm-coords
  :super bodyset
  )
(defmethod roboasm-coords
  (:dissoc-parent ()
   (let ((p (send self :parent)))
     (send p :dissoc self))
   self)
  (:to-root-list (obj)
   (labels ((parents-list (cds)
             (if (send cds :parent)
                 (append (list cds) (parents-list (send cds :parent))) (list cds))
             ))
     (parents-list obj)
     ))
  (:all-descendants (&optional (target-class roboasm-coords))
   (labels ((all-descendants (p)
             (when p
               (append (list p)
                       (mapcan #'all-descendants (send p :descendants))))

             ))
     (remove-if-not #'(lambda (x) (derivedp x target-class))
                    (all-descendants self))
     ))
  (:to-next-link ;; search descendants until reaches a parts/points in next-link-list
   (next-link-list &optional (target-class roboasm-coords))
   (labels ((all-descendants (p)
             (when (and p (not (memq p next-link-list)))
               (append (list p)
                       (mapcan #'all-descendants (send p :descendants))))

             ))
     (remove-if-not #'(lambda (x) (derivedp x target-class))
                    (all-descendants self))
     ))
  (:faces-intersect-with-point-vector (&rest args) nil)
  ;; search method
  (:fixed-points (&optional (type :free)) nil)
  (:active-fixed-points () nil)
  (:horns (&rest args) nil)
  (:active-horns () nil)
  (:find (name search-method &key ((:class cls)))
   (let ((res (send self search-method))) ;; type??
     (find-if #'(lambda (x) (eq name (send x :name))) res)
     ))
  )
(defclass roboasm-fixed-point
  :super roboasm-coords
  :slots (types ;; in *fixed-point-type-list*
          current-config
          type-geometry-list
          actuator-type ;; (if (not (null actuator-type)) 'this is actuator') :rotational, :linear, ( :sphere ), :fixed
          axis limit vlimit tqlimit ;;
          )
  )
(defmethod roboasm-fixed-point
  (:actuator-type (&optional (val :no-input))
   (when (not (eq val :no-input))
     (if (stringp val)
         (setq actuator-type (intern (string-upcase val) *keyword-package*))
       (setq actuator-type val)))
   actuator-type)
  (:axis (&optional (val :no-input))
   (when (not (eq val :no-input))
     (if (stringp val)
         (setq axis (intern (string-upcase val) *keyword-package*))
       (setq axis val)))
   axis)
  (:limit (&optional (val :no-input))
   (if (not (eq val :no-input))
       (setq limit val))
   limit)
  (:vlimit (&optional (val :no-input))
   (if (not (eq val :no-input))
       (setq vlimit val))
   vlimit)
  (:tqlimit (&optional (val :no-input))
   (if (not (eq val :no-input))
       (setq tqlimit val))
   tqlimit)
  )
(defclass roboasm-parts
  :super roboasm-coords
  :slots (type
          fixed-points ;; list of roboasm-fixed-point
          mass
          center-of-mass
          inertia-tensor
          extra-data
          b-box
          )
  )
(defmethod roboasm-parts
  (:remove-point (pt)
   (unless (eq self (send pt :parent))
     (return-from :remove-point))
   (cond
    ((memq pt fixed-points)
     (send self :dissoc pt)
     (setq fixed-points (remove pt fixed-points))
     t)
    (t
     nil)
    ))
  (:all-parts (&rest args) (copy-seq fixed-points))
  (:fixed-points (&optional (type :free)) (copy-seq fixed-points))
  (:active-fixed-points ()
   (remove-if #'(lambda (x) (send x :descendants)) fixed-points))
  (:not-active-fixed-points ()
   (remove-if-not #'(lambda (x) (send x :descendants)) fixed-points))
  (:horns (&rest args) nil)
  (:active-horns () nil)
  (:not-active-horns () nil)
  (:sensor-parts? () (if extra-data t))
  (:sensor-type? (tp)
   (if extra-data (some #'(lambda (exdata) (eq tp (send exdata :get :type))) extra-data)))
  (:set-bounding-box ()
   (let ((bb (make-bounding-box (flatten (collect-vertices (send self :bodies))) 0.0)))
     (if (and b-box (send b-box :parent)) (send (send b-box :parent) :dissoc b-box))
     (let ((bx (apply #'make-cube (coerce (send bb :diagonal) cons))))
       (send bx :locate (send bb :center) :world)
       (setq b-box bx)
       (send self :assoc bx)
       ))
   b-box)
  (:box (&rest args) (if b-box (send* b-box args)))
  (:nearest-intersect-distance (pos vec)
   (when b-box
     (send b-box :worldcoords)
     (let ((ret
            (mapcar #'(lambda (fp) (distance pos (cadr fp)))
                    (send b-box :faces-intersect-with-point-vector pos vec))))
       (if ret (car (sort ret #'<))))
     ))
  (:roboasm-from-parts
   (&key (start-as-root nil))
   (cond
    (start-as-root
     (send self :descendants-roboasm))
    (t
     (let* ((p-p (send self :parent))
            (r-p (if p-p (send p-p :parent))))
       (if (and p-p r-p)
           (send self :descendants-roboasm p-p)
         (send self :descendants-roboasm))
       ))
    ))
  (:descendants-roboasm
   (&optional connected-point lst)
   (let ((r-p (if connected-point (send connected-point :parent))))
     ;;print
     (if connected-point
         (setq lst (append lst (list (list
                                      (list :robot-parts-point (send r-p :name))
                                      (list :parts-name (send self :name))
                                      (list :parts-type type)
                                      (list :parts-point (send connected-point :name))
                                      (list :configuration (roboasm-fixed-point-current-config connected-point))))))
       (setq lst (append lst (list (list
                                    (list :initial-parts t)
                                    (list :parts-name (send self :name))
                                    (list :parts-type type)))))
       )
       (dolist (nap (send self :not-active-fixed-points))
         (dolist (dp (send nap :descendants))
           (dolist (d (send dp :descendants))
             (if (derivedp d roboasm-parts)
                 (setq lst (send d :descendants-roboasm dp lst)))
             )))
       )
   lst)
  )
(defclass roboasm-actuator
  :super roboasm-parts
  :slots (horns ;; list of roboasm-fixed-point // TODO: horns -> actuated-points
          )
  )
(defmethod roboasm-actuator
  (:remove-point (pt)
   (unless (eq self (send pt :parent))
     (return-from :remove-point))
   (when (memq pt horns)
     (send self :dissoc pt)
     (setq horns (remove pt horns))
     (return-from :remove-point t))
   (send-message roboasm-parts :remove-point pt)
   )
  (:all-parts (&rest args) (copy-seq (append horns fixed-points)))
  (:fixed-points (&optional (type :all)) (copy-seq (append horns fixed-points)))
  (:active-fixed-points ()
    (remove-if #'(lambda (x) (send x :descendants)) (append horns fixed-points)))
  (:not-active-fixed-points ()
    (remove-if-not #'(lambda (x) (send x :descendants)) (append horns fixed-points)))
  (:horns (&optional (type :all)) (copy-seq horns))
  (:active-horns ()
   (remove-if #'(lambda (x) (send x :descendants)) horns))
  (:not-active-horns ()
   (remove-if-not #'(lambda (x) (send x :descendants)) horns))
  )

(defclass roboasm-robot
  :super roboasm-coords
  :slots (name-tbl angle-tbl axis-tbl)
  )
(defmethod roboasm-robot
  (:init (&rest args)
   (send-super* :init args)
   (setq name-tbl  (make-hash-table))
   (setq angle-tbl (make-hash-table))
   (setq axis-tbl  (make-hash-table))
   )
  ;;
  (:attach
   (dx-obj &rest args &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   (when (send dx-obj :parent)
     (warn ";; :attach / ~A already attached~%" dx-obj)
     (return-from :attach nil))
   (unless (or (derivedp dx-obj roboasm-parts)
               (derivedp dx-obj roboasm-robot))
     (return-from :attach nil))
   (unless (send* self :_attach-points-check dx-obj args)
     (return-from :attach nil))
   ;;
   ;;
   (let ((config-coords
          (send self :_convert-configuration-to-coords robot-point parts-point attach-configuration)))
     (unless config-coords
       (return-from :attach nil))
     ;;(setq attach-configuration (send config-coords :name))
     ;;move obj / parts-point == robot-point
     (let ((target-point (send robot-point :worldcoords))
           (to-parts
            (send (send dx-obj :worldcoords) :transformation
                  (send (send parts-point :copy-worldcoords) :transform config-coords)))
           )
       (send dx-obj :newcoords (send to-parts :transformation target-point :world))
       ))
   ;;
   (unless just-align
     (cond
      ((derivedp dx-obj roboasm-robot)
       ;; invert child-parent
       (let* ((r-lst (send dx-obj :to-root-list parts-point))
              (prev (car r-lst)))
         (dolist (cur (cdr r-lst))
           (send cur :dissoc prev)
           (setq prev cur))
         ;;
         (setq r-lst (cdr (reverse r-lst)))
         (setq prev (car r-lst))
         (dolist (cur (cdr r-lst))
           (send cur :assoc prev)
           (setq prev cur))
         )
       )
      ((derivedp dx-obj roboasm-parts)
       ;; swap parent-children
       (send dx-obj :dissoc parts-point)
       (send parts-point :assoc dx-obj)
       )
      (t ;; never enter here
       ))
     ;; assoc
     (send self :_attach-points robot-point parts-point attach-configuration) ;; new-connection point-to-point
     )
   (let ((cs (send self :get :current-structure)))
     (cond
      (cs (send self :put :current-structure (+ cs (sys::address dx-obj))))
      (t  (send self :put :current-structure (sys::address self)))))
   self
   )
  (:_attach-points-check
   (input-obj &rest args &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   (unless (derivedp robot-point roboasm-fixed-point)
     (warn ";;mismatch(attach-configuration) :robot-point(~A) is not fixed-point~%"
           robot-point)
     (return-from :_attach-points-check nil))
   (unless (derivedp parts-point roboasm-fixed-point)
     (warn ";;mismatch(attach-configuration) :parts-point(~A) is not fixed-point~%"
           parts-point)
     (return-from :_attach-points-check nil))
   ;;; just for input-obj is parts
   (unless (or (derivedp input-obj roboasm-robot)
               (eq input-obj (send parts-point :parent)))  ;; parts-point may be jsut child of parts
     (warn ";;mismatch(attach-configuration) :parts-point(~A) is not child of ~A~%"
           parts-point input-obj)
     (return-from :_attach-points-check nil))
   (unless (memq robot-point (send self :all-parts roboasm-coords)) ;; robot-point is descendants of self
     (warn ";;mismatch(attach-configuration) :robot-point(~A) is not descendants of ~A~%"
           robot-point self)
     (return-from :_attach-points-check nil))
   (when (remove-if #'(lambda (x) (derivedp x roboasm-coords)) (send robot-point :descendants))
     (warn ";;mismatch(attach-configuration) :robot-point(~A) is already used for (~A)~%"
           robot-point (remove-if #'(lambda (x) (derivedp x roboasm-coords)) (send robot-point :descendants)))
     (return-from :_attach-points-check nil))
   (when (and (derivedp input-obj roboasm-robot)
              (remove-if #'(lambda (x) (derivedp x roboasm-coords)) (send parts-point :descendants)))
     (warn ";;mismatch(attach-configuration) :robot-point(~A) is already used for (~A)~%"
           robot-point (remove-if #'(lambda (x) (derivedp x roboasm-coords)) (send robot-point :descendants)))
     (return-from :_attach-points-check nil))
   t)
  (:_convert-configuration-to-coords
   (robot-point parts-point attach-configuration)
   (let ((ret (search-match-candidates (parts-point . types) (robot-point . types)))
         ret-attach)
     (unless ret
       (warn ";; type-mismatch(:attach configuration-check) :robot-point(~A) and :parts-point(~A)~% is not a pair~%"
             robot-point parts-point)
       (return-from :_convert-configuration-to-coords nil))
     (setq ret (reverse ret))
     ;;
     (cond
      ((null attach-configuration)
       (setq ret-attach (car ((car ret) . allowed-configuration))) ;; use default-one
       ;; warning
       )
      ((derivedp attach-configuration coordinates);; not use predefined-configuration
       (setq ret-attach attach-configuration))
      #|((listp attach-configuration);; not use predefined-configuration
       (setq ret-attach (eval attach-configuration))
       )|#
      ((cdr ret) ;; many match
       (dolist (r ret)
         (let ((ac-lst (r . allowed-configuration)))
           (setq ret-attach
                 (find-if #'(lambda (x) (eq (x . type) attach-configuration)) ac-lst))
           (when ret-attach (return))))
       )
      (t ;; just one match
       (let ((ac-lst ((car ret) . allowed-configuration)))
         (setq ret-attach
               (find-if #'(lambda (x) (eq (x . type) attach-configuration)) ac-lst))
         )
       ))
     ;;
     (unless ret-attach
       (warn ";; type-mismatch(:attach configuration-check) :robot-point(~A) and :parts-point(~A)~% does not has allowd-configuration(~A)~%"
             robot-point ;;rp-t-lst
             parts-point ;;pp-t-lst
             attach-configuration)
       (return-from :_convert-configuration-to-coords nil))
     ;;
     (when *robot-assembler-debug*
       (warn ";; attach robot: ~A, robot-point: ~A, " self robot-point)
       (warn "parts-point: ~A, :pair ~A, :attach-config ~A, " parts-point (car ret) ret-attach)
       (warn "pair-candidates: ~A~%" ret))
     ret-attach
     ))
  (:_attach-points
   (robot-point parts-point attach-configuration)
   (if (not attach-configuration) (setq attach-configuration :default))
   (setf (roboasm-fixed-point-current-config robot-point) attach-configuration)
   (setf (roboasm-fixed-point-current-config parts-point) attach-configuration)
   (send robot-point :assoc parts-point)
   )
  #|
  ;;
  (:attach-old
   (dx-obj &rest args &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   (when (send dx-obj :parent)
     (warn ";; :attach / ~A already attached~%" dx-obj)
     (return-from :attach-old nil))
   (cond
    ((derivedp dx-obj roboasm-robot)
     (if (send* self :_attach-points-check dx-obj args)
         (send* self :attach-robot dx-obj args)))
    ((derivedp dx-obj roboasm-parts)
     (if (send* self :_attach-points-check dx-obj args)
         (send* self :attach-parts dx-obj args)))
    (t
     ;; err
     nil
     ))
   )
  (:attach-robot ;; do not use attached robot after this method
   (obj &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)

   (let ((config-coords
          (send self :_convert-configuration-to-coords robot-point parts-point attach-configuration)))
     (unless config-coords
       (return-from :attach-robot nil))
     ;;move obj / parts-point == robot-point
     (let ((target-point (send robot-point :worldcoords))
           (to-parts
            (send (send obj :worldcoords) :transformation
                  (send (send parts-point :copy-worldcoords) :transform config-coords)))
           )
       (send obj :newcoords (send to-parts :transformation target-point :world))
       ))
   ;;
   (unless just-align
     ;; invert child-parent
     (let* ((r-lst (send obj :to-root-list parts-point))
            (prev (car r-lst)))
       (dolist (cur (cdr r-lst))
         (send cur :dissoc prev)
         (setq prev cur))
       ;;
       (setq r-lst (cdr (reverse r-lst)))
       (setq prev (car r-lst))
       (dolist (cur (cdr r-lst))
         (send prev :assoc cur))
       )
     ;; assoc
     (send self :_attach-points robot-point parts-point attach-configuration) ;; new-connection point-to-point
     )
   self
   )
  (:attach-parts
   (aparts &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   (let ((config-coords
          (send self :_convert-configuration-to-coords robot-point parts-point attach-configuration)))
     (unless config-coords
       (return-from :attach-parts nil))
   ;;move aparts / parts-point == robot-point
     (let ((target-point (send robot-point :worldcoords))
           (to-parts
            (send (send aparts :worldcoords) :transformation
                  (send (send parts-point :copy-worldcoords) :transform config-coords)))
           )
       ;; target-point may be changed
       (send aparts :newcoords (send to-parts :transformation target-point :world))
       ))
   (unless just-align
     ;; swap parent-children
     (send aparts :dissoc parts-point)
     (send parts-point :assoc aparts)
     ;;
     (send self :_attach-points robot-point parts-point attach-configuration) ;; new-connection point-to-point
     )
   self
   )
  ;;
  |#
  #|
  (:horns (&optional (type :all)) ;; obsolate
   (let ((obj-lst (send self :all-descendants roboasm-actuator))
         ret)
     (setq ret (mapcan #'(lambda (obj)
                           (remove-if #'(lambda (x) (send x :descendants)) (obj . horns))) obj-lst))
     ))
  (:fixed-points (&optional (type :free)) ;; obsolate
   (let ((obj-lst (send self :all-descendants roboasm-parts))
         ret)
     (setq ret (mapcan #'(lambda (obj)
                           (remove-if #'(lambda (x) (send x :descendants)) (obj . fixed-points))) obj-lst))
     ))
  |#
  (:all-horns (&optional (type :all))
   (let ((obj-lst (send self :all-descendants roboasm-actuator)))
     (mapcan #'(lambda (obj) (send obj :horns)) obj-lst)
     ))
  (:active-horns (&optional (type :all))
   (let ((obj-lst (send self :all-descendants roboasm-actuator)))
     (mapcan #'(lambda (obj) (send obj :active-horns)) obj-lst)
     ))
  (:not-active-horns (&optional (type :all))
   (let ((obj-lst (send self :all-descendants roboasm-actuator)))
     (mapcan #'(lambda (obj) (send obj :not-active-horns)) obj-lst)
     ))
  (:all-fixed-points (&optional (type :all))
   (let ((obj-lst (send self :all-descendants roboasm-parts)))
     (mapcan #'(lambda (obj) (send obj :fixed-points)) obj-lst)
     ))
  (:active-fixed-points ()
   (let ((obj-lst (send self :all-descendants roboasm-parts)))
     (mapcan #'(lambda (obj) (send obj :active-fixed-points)) obj-lst)
     ))
  (:not-active-fixed-points ()
   (let ((obj-lst (send self :all-descendants roboasm-parts)))
     (mapcan #'(lambda (obj) (send obj :not-active-fixed-points)) obj-lst)
     ))
  ;;
  (:descendant-link-list
   ()
   (labels ((all-descendant-list
             (p)
             (if (send p :descendants)
                 (if (cdr (send p :descendants))
                     (append (list p) (mapcar #'all-descendant-list (send p :descendants)))
                   (list p (all-descendant-list (car (send p :descendants))))
                   )
               (list p))
             ))
     (all-descendant-list self)
     ))
  (:parse-link-list ()
   (let ((start-link-coords
          (mapcan #'(lambda (x) (remove-if-not #'(lambda (x) (send x :descendants)) (send x :horns)))
                  (send self :all-descendants roboasm-actuator)))
         root-link
         )

     (mapcar #'(lambda (x) (send (car (send x :descendants)) :to-next-link start-link-coords))
             start-link-coords)

     (setq root-link (send self :to-next-link start-link-coords))
     ))
  ;;
  (:_write-urdf-coords
   (strm cds &optional (offset ""))
   (let ((pos (scale 0.001 (send cds :worldpos)))
         (rpy (car (send cds :rpy-angle))))
     (format strm "~A<origin xyz=\"~6,6F ~6,6F ~6,6F\" rpy=\"~8,8F ~8,8F ~8,8F\"/>~%"
             offset
             (elt pos 0) (elt pos 1) (elt pos 2)
             (elt rpy 2) (elt rpy 1) (elt rpy 0))
     ))
  (:_write-urdf-geometry
   (strm parts-list origin &key (offset "") (package-prefix) (bb-collision))
   (let* (
          (bds (flatten (mapcar #'(lambda (x) (send x :bodies)) parts-list)))
          )
     (dolist (bd bds)
       (let* ((trs (send origin :transformation bd))
              (pos (scale 0.001 (send trs :pos)))
              (rpy (car (send trs :rpy-angle)))
              (fname (send bd :get :filename))
              (scl   (send bd :get :scale))
              (col   (send bd :get :collision-model))
              )
         ;; for visual
         (cond
          ((and (derivedp bd gl::glvertices) fname)
           (format strm "~A<visual>~%" offset)
           (send self :_write-urdf-coords strm trs (format nil "~A   " offset))
           (format strm "~A   <geometry>~%" offset)
           (format strm "~A   <mesh filename=\"~A\" scale=\"~6,6F ~6,6F ~6,6F\"/>~%"
                   offset
                   (if package-prefix
                       (format nil "~A/~A" package-prefix (send bd :get :original-filename))
                     (format nil "file://~A" (send (truename fname) :namestring)))
                   (* 0.001 scl) (* 0.001 scl) (* 0.001 scl))
           (format strm "~A   </geometry>~%" offset)
           (format strm "~A</visual>~%" offset)
           )
          (t
           (warn "geometry not implimented for ~A~%" bd)
           ))
         ;; for collision
         (cond
          (bb-collision
           ;; use bounding-box for collision / not implemented
           )
          (col
           (dolist (c col)
             (let (
                   (box   (cadr (assoc :box c)))
                   (cyl   (cadr (assoc :cylinder c)))
                   (sph   (cadr (assoc :sphere c)))
                   (cds   (yaml-parse-trans-and-rot c))
                   )
               ;;
               (if (null cds) (setq cds (make-coords)))
               (setq cds (send (send trs :copy-worldcoords) :transform cds))
               ;;
               (format strm "~A<collision>~%" offset)
               (send self :_write-urdf-coords strm cds (format nil "~A   " offset))
               (format strm "~A   <geometry>~%" offset)
               (cond
                (box
                 (format strm "~A     <box size=\"~8,6F ~8,6F ~8,6F\" />~%" offset
                         (* 0.001 (elt box 0)) (* 0.001 (elt box 1)) (* 0.001 (elt box 2)))
                 )
                (cyl
                 (format strm "~A     <cylinder radius=\"~8,6F\" length=\"~8,6F\" />~%" offset
                         (* 0.001 (elt cyl 0)) (* 0.001 (elt cyl 1)))
                 )
                (sph
                 (format strm "~A     <sphere radius=\"~8,6F\" />~%" offset
                         (* 0.001 (elt sph 0)))
                 ))
               (format strm "~A   </geometry>~%" offset)
               (format strm "~A</collision>~%" offset)
               ))
           )
          ((and (derivedp bd gl::glvertices) fname)
           (format strm "~A<collision>~%" offset)
           (send self :_write-urdf-coords strm trs (format nil "~A   " offset))
           (format strm "~A   <geometry>~%" offset)
           (format strm "~A   <mesh filename=\"~A\" scale=\"~6,6F ~6,6F ~6,6F\"/>~%"
                   offset
                   (if package-prefix
                       (format nil "~A/~A" package-prefix (send bd :get :original-filename))
                     (format nil "file://~A" (send (truename fname) :namestring)))
                   (* 0.001 scl) (* 0.001 scl) (* 0.001 scl))
           (format strm "~A   </geometry>~%" offset)
           (format strm "~A</collision>~%" offset)
           )
          (t
           (warn "geometry not implimented for ~A~%" bd)
           ))
         ))
     ))
  (:_write-urdf-link  ;; .body link link and joint
   (strm coords-list origin &key (name) (offset "") (package-prefix) (fixed-robot))
   (let ((parts-list (remove-if-not #'(lambda (x) (derivedp x roboasm-parts)) coords-list)))
     (format strm "~A<xacro:gazebo_link_reference link_name=\"~A\" />~%" offset name)
     (format strm "~A<link name=\"~A\">~%" offset name);; uniq name
     (let ((t-mass 0.0)
           (n-cent (float-vector 0 0 0))
           (n-iner (make-matrix 3 3))
           )
       (dolist (pt parts-list)
         (setq t-mass (+ t-mass (pt . mass))))
       (dolist (pt parts-list)
         (let ((com (pt . center-of-mass))
               (mass (pt . mass))
               )
           (v+ n-cent
               (scale mass
                      (send (send origin :worldcoords) :inverse-transform-vector
                            (send (send pt :worldcoords) :transform-vector com)))
               n-cent)
           ))
       (scale (/ 1 t-mass) n-cent n-cent)
       (dolist (pt parts-list)
         (let ((com (pt . center-of-mass))
               (mass (pt . mass))
               (iner (pt . inertia-tensor)))
           (m+ n-iner
               (move-inertia mass iner
                             (send (send origin :worldcoords) :inverse-transform-vector
                                   (send (send pt :worldcoords) :transform-vector com)) ;; old com at new coords
                             (send (send (send origin :worldcoords)
                                         :transformation (send pt :worldcoords)) :worldrot)
                             n-cent)
               n-iner)
           ))
       (scale 0.001 n-cent n-cent)
       (scale 1e-9 (array-entity n-iner) (array-entity n-iner))
       (format strm "~A   <inertial>~%" offset)
       (format strm "~A      <mass value=\"~A\"/>~%" offset (* 0.001 t-mass))
       (format strm "~A      <origin rpy=\"0 0 0\" xyz=\"~A ~A ~A\"/>~%" offset
               (elt n-cent 0) (elt n-cent 1) (elt n-cent 2))
       (format strm "~A      <inertia ixx=\"~A\" ixy=\"~A\" ixz=\"~A\" iyy=\"~A\" iyz=\"~A\" izz=\"~A\"/>~%" offset
               (aref n-iner 0 0) (aref n-iner 1 0) (aref n-iner 2 0)
               (aref n-iner 1 1) (aref n-iner 1 2)
               (aref n-iner 2 2)
               )
       (format strm "~A   </inertial>~%" offset)
       ;; for extra-data (sensor)
       (dolist (pt parts-list)
         (send pt :put :robot-link-name name)
         (let ((ex-data-list (pt . extra-data)))
           (dolist (ex-data ex-data-list)
             (let ((trs (send (send origin :worldcoords) :transformation
                              (send (send pt :copy-worldcoords) :transform (send ex-data :worldcoords)))))
               ;; debug
               (format t ";; name: ~A, type: ~A, params: ~A, link: ~A, from-link-origin: ~A~%"
                       (send ex-data :get :name)
                       (send ex-data :get :type)
                       (send ex-data :get :parameters)
                       name trs
                       )
               ;;
               (send ex-data :put :robot-link-name name)
               (send ex-data :put :from-link-origin trs)
               ))))
       )
     ;;;
     (send self :_write-urdf-geometry strm parts-list origin
           :offset (format nil "~A   " offset) :package-prefix package-prefix)
     (format strm "~A</link>~%" offset)
     ))
  (:_write-urdf-joint ;; return-joint-name
   (strm start-coords parent-name child-name coords
         &key (name) (limit) (offset "") )
   (let ((initial-angle (send self :get-initial-angle (send start-coords :name)))
         (new-name (send self :get-actuator-name (send start-coords :name)))
         (axis (send start-coords :axis))
         (type (send start-coords :actuator-type))
         (limit (send start-coords :limit))
         (vlimit (send start-coords :vlimit))
         (tqlimit (send start-coords :tqlimit))
         type-name
         )
     (if new-name (setq name new-name))
     (setq type-name
           (case type
             (:rotational (if limit "revolute" "continuous"))
             (:linear "prismatic")
             (:fixed "fixed")
             (:sphere "not implemented")
             (t "revolute")))
     (format strm "~A<xacro:gazebo_transmission joint_name=\"~A\" />~%" offset name)
     (format strm "~A<joint name=\"~A\" type=\"~A\">~%" offset name type-name);; uniq name
     (format strm "~A   <parent link=\"~A\"/>~%" offset parent-name)
     (format strm "~A   <child link=\"~A\"/>~%"  offset child-name)
     (when initial-angle ;; type check
       (case type
        (:rotational
         (send coords :rotate (deg2rad initial-angle) axis))
        (:linear
         ;;(send coords :translate initial-angle axis)
         (send coords :translate-with-axis initial-angle axis)
         )
        (t (warn ";; invalid setting(:initial-angle) for ~A(~A)~%" name type))
        )
       )
     (send self :_write-urdf-coords strm coords (format nil "~A   " offset))
     (unless (eq type :fixed)
       ;;(format strm "   <origin rpy=\"0 -0 0\" xyz=\"0 0.09 0\"/>~%" offset)
       (cond
        ((null axis)
         (format strm "~A   <axis xyz=\" 0  0  1\"/>~%" offset))
        ((eq axis :z)
         (format strm "~A   <axis xyz=\" 0  0  1\"/>~%" offset))
        ((eq axis :-z)
         (format strm "~A   <axis xyz=\" 0  0 -1\"/>~%" offset))
        ((eq axis :y)
         (format strm "~A   <axis xyz=\" 0  1  0\"/>~%" offset))
        ((eq axis :-y)
         (format strm "~A   <axis xyz=\" 0 -1  0\"/>~%" offset))
        ((eq axis :x)
         (format strm "~A   <axis xyz=\" 1  0  0\"/>~%" offset))
        ((eq axis :-x)
         (format strm "~A   <axis xyz=\"-1  0  0\"/>~%" offset))
        ((or (float-vector-p axis) (listp axis))
         (format strm "~A   <axis xyz=\"~12,12F ~12,12F ~12,12F\"/>~%"
                 offset (elt axis 0) (elt axis 1) (elt axis 2)))
        )
       (let ((lower (car limit))
             (upper (cadr limit))
             (velocity 40)
             (effort 200))
         (if (and vlimit (atom vlimit))
             (setq velocity vlimit)
           (if vlimit
               (setq velocity (if (> (abs (car vlimit)) (abs (cadr vlimit)))
                                  (abs (car vlimit)) (abs (cadr vlimit))))))
         (if (and tqlimit (atom tqlimit))
             (setq effort tqlimit)
           (if tqlimit
               (setq effort (if (> (abs (car tqlimit)) (abs (cadr tqlimit)))
                                (abs (car tqlimit)) (abs (cadr tqlimit))))))
         (if (null lower) (setq lower -pi))
         (if (null upper) (setq upper pi))
         (format strm "~A   <limit lower=\"~8,8F\" upper=\"~8,8F\" velocity=\"~2,2F\" effort=\"~2,2F\" />~%"
                 offset lower upper velocity effort)
         )
       )
     (format strm "~A</joint>~%" offset)
     name
     ))
  (:_write-controller-config
   (fname joint-name-list &key (p-gain 25) (i-gain 0) (d-gain 0))
   (with-open-file
    (strm fname :direction :output)
    (format strm "## auto-generated by robot-assembler.l~%")
    (format strm "# Publish all joint states -----------------------------------~%")
    (format strm "joint_state_controller:~%")
    (format strm "  type: joint_state_controller/JointStateController~%")
    (format strm "  publish_rate: 50~%")
    (format strm "fullbody_controller:~%")
    (format strm "#  type: effort_controllers/JointTrajectoryController~%")
    (format strm "  type: velocity_controllers/JointTrajectoryController~%")
    (format strm "  joints:~%")
    (dolist (jt joint-name-list)
      (format strm "    - ~A~%" jt))
    (format strm "  gains:~%")
    (dolist (jt joint-name-list)
      (format strm "    ~A:  {p: ~A, i: ~A, d: ~A}~%"
              jt p-gain i-gain d-gain))
    )
   )
  (:_write-euscollada-yaml
   (fname joint-name-list &key (limb) (end-coords) );;
   (with-open-file
    (strm fname :direction :output)
    (format strm "## auto-generated by robot-assembler.l~%")
    (format strm "rarm:~%") ;; fixed limb
    (dolist (jt joint-name-list)
      (format strm "  - ~A : rarm-~A~%" jt jt))
    ;; end-coords
    ;; angle-vector
    )
   )
  (:write-urdf (fname &key (robotname "robot") (fixed-robot) (package-prefix nil))
   (let ((start-coords-list ;; (list of link-origin) ;; should be horn(joint)
          (mapcan #'(lambda (x) (remove-if-not #'(lambda (x) (send x :descendants)) (send x :horns)))
                  (send self :all-descendants roboasm-actuator)))
         root-link
         link-list-list
         parent-list
         joint-names
         )
     ;; for setting sensor parts as link?
     (dolist (pt (send self :all-parts))
       (when (send pt :sensor-type? :touch) ;; touch sensor should be link of single(geometry)
         ;; cut from parent
         (let ((p-p (send pt :parent)))
           (cond
            (p-p
             (let ((pp (send p-p :parent)))
               (unless (memq pp start-coords-list) ;; parent may connected to horns
                 (push pp start-coords-list)
                 (send (car start-coords-list) :actuator-type :fixed)
                 ))
             )
            (t ;; pt is root-parts
             ;; not implemented
             (warn ";; sensor parts should not be the root~%")
             )
            ))
         ;; cut to child
         (dolist (chd (remove-if-not #'(lambda (x) (and (derivedp x roboasm-fixed-point)
                                                        (send x :descendants))) (send pt :descendants)))
           (let ((dsc (send chd :descendants)))
             (if (> (length dsc) 1) (warn ";; ~A has many descendants ~A~%" chd dsc))
             (unless (memq (car dsc) start-coords-list) ;; child may connected to horns
               (push (car dsc) start-coords-list)
               (send (car start-coords-list) :actuator-type :fixed)
               )
             ))
         ))
     ;;
     (setq link-list-list
           (mapcar #'(lambda (x) (send (car (send x :descendants)) :to-next-link start-coords-list))
                   start-coords-list))

     (setq root-link (send self :to-next-link start-coords-list))

     (setq parent-list
           (mapcar #'(lambda (start-coords)
                       (if (memq (send start-coords :parent) root-link)
                           :root-link
                         (position-if #'(lambda (link-list) (memq (send start-coords :parent) link-list))
                                      link-list-list)))
                   start-coords-list))
     ;; linknames "BASE" "LINK0" ...
     ;; joint-names "JOINT0" ...
     (with-open-file
      (strm (if fname fname "/dev/null") :direction :output)
      (format strm "<?xml version=\"1.0\" ?>~%")
      (format strm "<!-- This file was generated by robot assembler -->~%")
      (format strm "<robot name=\"~A\"~%" robotname)
      (format strm "       xmlns:xi=\"http://www.w3.org/2001/XInclude\"~%")
      (format strm "       xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\"~%")
      (format strm "       xmlns:model=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#model\"~%")
      (format strm "       xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"~%")
      (format strm "       xmlns:body=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#body\"~%")
      (format strm "       xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\"~%")
      (format strm "       xmlns:joint=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#joint\"~%")
      (format strm "       xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"~%")
      (format strm "       xmlns:rendering=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering\"~%")
      (format strm "       xmlns:renderable=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable\"~%")
      (format strm "       xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"~%")
      (format strm "       xmlns:physics=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#physics\"~%")
      (format strm "       xmlns:xacro=\"http://www.ros.org/wiki/xacro\">~%")
      (format strm "  <xacro:macro name=\"gazebo_link_reference\" params=\"link_name mu1:=0.9 mu2:=0.9\" >~%")
      (format strm "    <gazebo reference=\"${link_name}\">~%")
      (format strm "      <mu1>${mu1}</mu1>~%")
      (format strm "      <mu2>${mu2}</mu2>~%")
      (format strm "    </gazebo>~%")
      (format strm "  </xacro:macro>~%")
      (format strm "  <xacro:macro name=\"gazebo_transmission\" params=\"joint_name\" >~%")
      (format strm "    <transmission name=\"${joint_name}_trans\">~%")
      (format strm "      <type>transmission_interface/SimpleTransmission</type>~%")
      (format strm "      <joint name=\"${joint_name}\">~%")
      (format strm "        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>~%")
      (format strm "        <!--hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface-->~%")
      (format strm "      </joint>~%")
      (format strm "      <actuator name=\"${joint_name}_motor\">~%")
      (format strm "        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>~%")
      (format strm "        <!--hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface-->~%")
      (format strm "        <mechanicalReduction>1</mechanicalReduction>~%")
      (format strm "      </actuator>~%")
      (format strm "    </transmission>~%")
      (format strm "  </xacro:macro>~%~%")
      (format strm "  <xacro:macro name=\"no_gazebo_transmission\" params=\"joint_name\" >~%")
      (format strm "    <!-- dummy for fixed joint -->~%")
      (format strm "  </xacro:macro>~%")
      (let ((cntr 0))
        ;; write root link
        (send self :_write-urdf-link strm root-link (send self :copy-worldcoords)
              :name "BASE" :package-prefix package-prefix :offset "  ")
        ;; write links
        (mapcar #'(lambda (link-list start-coords)
                    (send self :_write-urdf-link strm link-list
                          (send start-coords :copy-worldcoords)
                          :name (format nil "LINK~D" cntr) ;; from database??
                          :offset "  "
                          :package-prefix package-prefix)
                    (incf cntr))
                link-list-list start-coords-list)
        )
      ;; wirte joints
      (let ((cntr 0))
        (mapcar #'(lambda (parent-pos start-coords)
                    (let* ((p-coords
                            (case parent-pos
                              (:root-link (send (car root-link) :worldcoords))
                              (t (send (elt start-coords-list parent-pos) :worldcoords))))
                           (p-c-coords
                            (send p-coords :transformation (send start-coords :worldcoords)))
                           )
                      (push
                      (send self :_write-urdf-joint strm start-coords
                            (case parent-pos
                              (:root-link "BASE")
                              (t (format nil "LINK~D" parent-pos)))
                            (format nil "LINK~D" cntr)
                            p-c-coords
                            :name (format nil "JOINT~D" cntr)
                            :offset "  ")
                      joint-names)
                      (incf cntr)))
                parent-list start-coords-list)
        (setq joint-names (reverse joint-names))
        ;;
        )
      ;;
      ;; write sensor data
      ;;
      (let ((sensor-counter 0))
        (dolist (pt (send self :all-parts))
          (dolist (exdata (pt . extra-data))
            (let ((nm  (send exdata :get :name))
                  (tp  (send exdata :get :type))
                  (pm  (send exdata :get :parameters))
                  (lk  (send exdata :get :robot-link-name))
                  (cds (send exdata :get :from-link-origin))
                  )
              (cond
               ((eq tp :imu)
                (let ((xyz.rpy (gazebo-xyz-rpy cds))
                      args)
                  (dolist (p (list :rate))
                    (let ((val (yaml-get-value pm p)))
                      (when (not (eq val *yaml-not-exist*))
                        (push val args) (push p args)
                        )))
                  (apply #'gazebo-imu-sensor
                         strm :offset "  "
                         :name (format nil "~A/~A" nm (string (send pt :name)))
                         :parent-link lk
                         :xyz (car xyz.rpy) :rpy (cdr xyz.rpy)
                         :topic (format nil "sensor_imu~2,2D" sensor-counter)
                         args)
                  ))
               ((eq tp :camera)
                (let ( args)
                  (dolist (p (list :rate :fov :width :height :near-clip :far-clip))
                    (let ((val (yaml-get-value pm p)))
                      (when (not (eq val *yaml-not-exist*))
                        (push val args) (push p args)
                        )))
                  (apply #'gazebo-camera
                         strm :offset "  "
                         :name (format nil "~A/~A" nm (string (send pt :name)))
                         :parent-link lk
                         :camera-coords cds
                         :topic (format nil "camera~2,2D" sensor-counter)
                         args)
                  ))
               ((eq tp :range)
                (let ((xyz.rpy (gazebo-xyz-rpy cds))
                      args)
                  (dolist (p (list :rate :samples :resolution :min-angle :max-angle :min-range :max-range))
                    (let ((val (yaml-get-value pm p)))
                      (when (not (eq val *yaml-not-exist*))
                        (push val args) (push p args)
                        )))
                  (apply #'gazebo-range
                         strm :offset "  "
                         :name (format nil "~A/~A" nm (string (send pt :name)))
                         :parent-link lk
                         :xyz (car xyz.rpy) :rpy (cdr xyz.rpy)
                         :topic (format nil "range~2,2D" sensor-counter)
                         args)
                  ))
               ((eq tp :touch)
                (let ( args )
                  (dolist (p (list :rate))
                    (let ((val (yaml-get-value pm p)))
                      (when (not (eq val *yaml-not-exist*))
                        (push val args) (push p args)
                        )))
                  (apply #'gazebo-touch-sensor
                         strm :offset "  "
                         :name (format nil "~A/~A" nm (string (send pt :name)))
                         :parent-link lk
                         :topic (format nil "sensor_touch~2,2D" sensor-counter)
                         args)
                  ))
               (t
                (warn "unknown sensor type ~A~%" tp)
                )
               ))
            (incf sensor-counter))
          ))
      ;; write gazebo control
      (format strm "~%")
      (gazebo-control strm :offset "  ")
      ;;
      (format strm "</robot>~%")
      );; with-open-file
     (if fname
         (send self :_write-controller-config
               (format nil "~A.gz_controller.yaml" fname)  joint-names))
     (if fname
         (send self :_write-euscollada-yaml
               (format nil "~A.euscollada.yaml" fname)  joint-names))
     t
     ))
  (:write-body ());; choreonoid
  (:write-robot-assembler-yaml ())
  ;;
  (:write-roboasm
   (fname)
   (let ((roboasm (send self :get-roboasm))
         (config  (send self :assemble-config)))
     (when roboasm
       (setq roboasm (append roboasm (if config (list config))))
       (with-open-file
        (strm (if fname fname "/dev/null") :direction :output)
        (pprint roboasm strm)
        ))
     ))
  (:get-roboasm ()
   (let* ((ds (send self :descendants))
          (rt (car ds)))
     (when (> (length ds) 1)
       (warn ";; There are more than 1 root parts (~A)~%" ds)
       (return-from :get-roboasm))
     (unless (derivedp rt roboasm-parts)
       (warn ";; root parts(~A) is not a instance of roboasm-parts~%" rt)
       (return-from :get-roboasm))
     (send rt :descendants-roboasm)
     ))
  ;;
  (:all-parts (&optional (target-class roboasm-parts))
   (send self :all-descendants target-class))
  (:bodies () ;; for display at irtviewer
   (let ((vs (send self :get :view-structure))
         (cs (send self :get :current-structure)))
     (cond
      ((and vs cs (= vs cs))
       (send self :get :view-bodies))
      (cs
       (warn "update structure~%")
       (send self :put :view-structure cs)
       (send self :put :view-bodies (flatten (send-all (remove self (send self :all-parts roboasm-coords) :count 1) :bodies))))
      (t
       (warn "initialize structure~%")
       (send self :put :view-structure 0)
       (send self :put :current-structure 0)
       (send self :put :view-bodies (flatten (send-all (remove self (send self :all-parts roboasm-coords) :count 1) :bodies))))
      )
     ))
  ;; new feature
  (:add-actuator-name
   (horn-name new-name &optional (no-check nil))
   (when (or no-check
             (find-if #'(lambda (h) (eq (send h :name) horn-name)) (send self :all-horns)))
     (setf (gethash horn-name name-tbl) new-name)
     t)
   )
  (:add-initial-angle
   (horn-name init-angle &optional (no-check nil))
   (when (or no-check
             (find-if #'(lambda (h) (eq (send h :name) horn-name)) (send self :all-horns)))
     (setf (gethash horn-name angle-tbl) init-angle)
     t)
   )
  (:add-actuator-axis
   (horn-name axis &optional (no-check nil))
   (when (or no-check
             (find-if #'(lambda (h) (eq (send h :name) horn-name)) (send self :all-horns)))
     (setf (gethash horn-name axis-tbl) axis)
     t)
   )
  (:set-initial-coords
   (cds)
   (let ((cc (send self :copy-worldcoords))
         (root-parts (car (send self :descendants))))
     (send cc :transform cds)
     (send root-parts :newcoords cc)
     (send root-parts :worldcoords)
     (send self :worldcoords)
     self
     ))
  (:get-actuator-name (horn-name) (gethash horn-name name-tbl))
  (:get-initial-angle (horn-name) (gethash horn-name angle-tbl))
  (:get-actuator-axis (horn-name) (gethash horn-name axis-tbl))
  (:read-assemble-config (config &optional (no-check t))
   (let ((ret (cadr (assoc :actuator-name config))))
     (dolist (cf ret)
       (send self :add-actuator-name (car cf) (cadr cf) no-check))
     )
   (let ((ret (cadr (assoc :initial-angle config))))
     (dolist (cf ret)
       (send self :add-initial-angle (car cf) (cadr cf) no-check))
     )
   (let ((ret (cadr (assoc :actuator-axis config))))
     (dolist (cf ret)
       (send self :add-actuator-axis (car cf) (cadr cf) no-check))
     )
   (let ((ret (cadr (assoc :robot-name config))))
     (when ret
       (send self :name ret)
       (send self :set :assemble-name ret)
       ))
   (let ((ret (cadr (assoc :initial-coords config))))
     (cond
      ((listp ret)
       (setq ret (eval ret)))
      ((stringp ret)
       (setq ret (eval (read-from-string ret))))
      (t (warn ";; unknown type ~A~%" ret)))
     (when ret
       (send self :set-initial-coords ret))
     )
   )
  (:assemble-config ()
   (let ((anm (send self :get :assemble-name))
         (cds (send self :transformation (car (send self :descendants))))
         nm ag ax)
     (send name-tbl  :map #'(lambda (k v) (push (list k v) nm)))
     (send angle-tbl :map #'(lambda (k v) (push (list k v) ag)))
     (send axis-tbl  :map #'(lambda (k v) (push (list k v) ax)))
     (if (eps-coords= cds (make-coords) 1e-9)
         (setq cds nil)
       (setq cds (write-coords-as-string cds)))
     (if (or nm ag ax cds)
         (remove nil
                 (list
                  :assemble-config
                  (if cds (list :initial-coords cds))
                  (if anm (list :robot-name anm))
                  (if nm (list :actuator-name nm))
                  (if ag (list :initial-angle ag))
                  (if ax (list :actuator-axis ax))
                  )))
     ))
  )

(defun search-match-candidates (parts-point-types robot-point-types)
  (let ( ret )

    (dolist (pp-t parts-point-types)
      (dolist (rp-t robot-point-types)
        (let ((r (gethash (list pp-t rp-t) *fixed-point-match-htable*)))
          (if r (push r ret)))
        ))

    (unless ret
      (return-from search-match-candidates))

    ret
    ))
(defun unique-list (lst)
  (let (cur)
    (mapcan #'(lambda (x) (unless (memq x cur) (push x cur) (list x))) lst)))
(defun search-allowed-configuration (parts-point-types robot-point-types)
  (let ((match-candidates
         (search-match-candidates parts-point-types robot-point-types))
        ret-attach)

    (dolist (mt match-candidates)
      (setq ret-attach (append ret-attach (mt . allowed-configuration)))
      )

    (unique-list ret-attach)
    ))
(defun search-target-types (point-types)
  (let (ret)
    (dolist (p-t point-types)
      (send *fixed-point-match-htable*
            :map #'(lambda (k v) (cond ((eq p-t (car  (v . pair))) (push (cadr (v . pair)) ret))
                                       ((eq p-t (cadr (v . pair))) (push (car  (v . pair)) ret)))))
      )
    (unique-list ret)
    )))
;;
;;
;;
(defun make-robot-assembler-robot (initial-parts &key (coords (make-coords)) (name "roboasm"))
  (instance roboasm-robot :init (make-cascoords :coords coords)
            :name name
            :bodies (list initial-parts))
  )

;;
;; parse configuration.yaml
;;
(defun robot-assembler-initialize (fname &key (project-dir))
  (when project-dir
    (setq *project-dir* project-dir))
  (_parse-roboasm-configuration
   (_load-yaml-file fname :project-dir project-dir)
   :project-dir project-dir)
  )

#| ;; use call python script
(defun _load-yaml-file (fname &key (project-dir) (pyfile "dump_yaml_to_sexp.py"))
  (when project-dir
    (setq pyfile (format nil "~A/scripts/~A" project-dir pyfile)))
  (let (res)
    ;; read yaml -> list
    (let ((p (piped-fork (format nil "python ~A ~A" pyfile fname))))
      (setq res (read p))
      (close p))
    ;; read yaml as string
    (let (lines line)
      (with-open-file
       (f fname)
       (while (setq line (read-line f nil nil))
         (push line lines)))
      (setq *current-setting-yaml* (reverse lines)))
    res))
|#
;; new version / use euslisp foreign function
(defun _load-yaml-file (fname &key (project-dir) (pyfile "dump_yaml_to_sexp.py"))
  (when project-dir
    (setq pyfile (format nil "~A/scripts/~A" project-dir pyfile)))
  (let (res)
    ;; read yaml as string
    #|
    (let (lines line)
      (with-open-file
       (f fname)
       (while (setq line (read-line f nil nil))
         (push line lines)))
      (setq *current-setting-yaml* (reverse lines)))
    |#
    (c-parse-yaml-file fname)
    ))
(defun _parse-roboasm-configuration (read-from-yaml &key (project-dir))
  (let ((fm (cadr (assoc :format read-from-yaml)))
        (fv (cadr (assoc :formatversion read-from-yaml)))
        (ag-u (cadr (assoc :angleunit read-from-yaml)))
        (lg-u (cadr (assoc :lengthunit read-from-yaml)))
        (_fixed-point-lst
         (cadr (assoc :fixed-point-type-list read-from-yaml)))
        (_fixed-point-match-lst
         (cadr (assoc :fixed-point-type-match-list read-from-yaml)))
        (_pre-defined-config-lst
         (cadr (assoc :pre-defined-configuration-list read-from-yaml)))
        (actuators (cadr (assoc :actuators read-from-yaml)))
        (parts-lst (cadr (assoc :parts read-from-yaml)))
        )

    ;; *fixed-point-list* -> *fixed-point-htable*
    (setq *fixed-point-htable* (make-hash-table))
    (dolist (fp _fixed-point-lst)
      (let ((ky (intern (string-upcase fp) *keyword-package*)))
        (setf (gethash ky *fixed-point-htable*) t)))
    ;;
    (setq *pre-defined-configuration-htable* (make-hash-table))
    (dolist (pc _pre-defined-config-lst)
      (let* ((tp (cadr (assoc :type pc)))
             (rot (cadr (assoc :rotation pc)))
             (desc (cadr (assoc :description pc)))
             (ky (intern (string-upcase tp) *keyword-package*))
             (fc (instance fix-configuration :init))
             )
        (setq (fc . description) desc)
        (setq (fc . type) ky)
        (send fc :name ky)
        (when rot
          (send fc :transform
                (make-coords :rot
                             (rotation-matrix (deg2rad (car (last rot)))
                                              (subseq (coerce rot float-vector) 0 3)))))
        (setf (gethash ky *pre-defined-configuration-htable*) fc)
        ))
    ;; create hash-table for point-match
    (setq *fixed-point-match-htable*
          (make-hash-table :test 'pair-test))
    (send *fixed-point-match-htable* :hash-function 'pair-sxhash)
    (dolist (fpm _fixed-point-match-lst)
      (let ((pair (mapcar #'(lambda (x) (intern (string-upcase x) *keyword-package*))
                          (cadr (assoc :pair fpm))))
            (ac-list (cadr (assoc :allowed-configuration fpm)))
            inst-ac-list
            ret)
        ;; pair-check
        (dolist (ac ac-list)
          (let ((ky (intern (string-upcase ac) *keyword-package*))
                ret)
            (setq ret (gethash ky *pre-defined-configuration-htable*))
            (cond
             (ret (push ret inst-ac-list))
             (t   (warn ";; ~A does not exist in pre-defined-configurations~%" ky)))))
        ;;
        (setq ret (gethash pair *fixed-point-match-htable*))
        (cond
         (ret (warn ";; ~%" pair))
         (t   (let ((inst (instance fixed-point-match :name pair)))
                (setq (inst . pair) pair)
                (setq (inst . allowed-configuration) (reverse inst-ac-list))
                (setf (gethash pair *fixed-point-match-htable*) inst)))
         )
        ))
    ;;
    (setq *robot-assembler-functions-htable* (make-hash-table))
    ;;
    ;; make function for actuator
    (dolist (actuator actuators)
      (_parse-parts-and-actuator actuator :parts nil)
      )
    ;; make function for parts
    (dolist (parts parts-lst)
      (_parse-parts-and-actuator parts :parts t)
      )
    ;;
    ))
(defun _parse-parts-and-actuator (slst &key (parts nil))
  (let* ((type      (cadr (assoc :type       slst)))
         (geometry  (cadr (assoc :geometry   slst)))
         (massparam (cadr (assoc :mass-param slst)))
         (horns     (cadr (assoc :horns      slst)))
         (extra-d   (cadr (assoc :extra-data slst)))
         (fixed-points  (cadr (assoc :fixed-points slst)))
         (fname
          (if parts
              (intern (string-upcase (format nil "make-robot-assembler-parts-~A" type)))
            (intern (string-upcase (format nil "make-robot-assembler-actuator-~A" type)))))
         (mass (cadr (assoc :mass massparam)))
         (com  (cadr (assoc :center-of-mass massparam)))
         (iner (cadr (assoc :inertia-tensor massparam)))
         )
    (when (and parts horns)
      (warn ";; definition of ~A has wrong setting of horns / ~A~%" type horns))
    (unless massparam
      (warn ";; no massparam setting for ~A~%" type))
    (setq type (intern (string-upcase type) *keyword-package*))
    (when (gethash type *robot-assembler-functions-htable*)
      (warn ";; function for type ~A is already defined!~%" type))
    ;;
    (eval
     `(defun ,fname
          (&key (name ,type) (namespace))
        (let ((bds (_parse-geometry ',geometry))
              parts-instance)
          (dolist (c (cdr bds)) (send (car bds) :assoc c))
          ,(if parts
               `(setq parts-instance (instance roboasm-parts :init (make-cascoords) :name name :bodies bds))
             `(setq parts-instance (instance roboasm-actuator :init (make-cascoords) :name name :bodies bds))
             )
          (setq (parts-instance . type) ,type)
          ,(if mass
               `(setq (parts-instance . mass) ,(* 1000 mass))
             `(setq (parts-instance . mass) 100)
             )
          ,(if com
               `(setq (parts-instance . center-of-mass) ,(scale 1000 (coerce com float-vector)))
             `(setq (parts-instance . center-of-mass) #f(0 0 0))
             )
          ,(if iner
               `(setq (parts-instance . inertia-tensor)
                      ,(let ((m (make-matrix 3 3)))
                         (setf (array-entity m) (scale 1e9 (coerce iner float-vector)))
                         m))
             `(setq (parts-instance . inertia-tensor) #2f((0.1 0 0) (0 0.1 0) (0 0 0.1)))
             )
          (let ((parsed-ex-data))
            (dolist (e-d ',extra-d)
              (push (_parse-extra-data e-d) parsed-ex-data))
            (setq (parts-instance . extra-data) parsed-ex-data))
          (send parts-instance :set-bounding-box)
          (if (eq t namespace) (setq namespace (format nil "~A" (string name))))
          ,(if (not parts)
             `(let ((hns (_parse-fixed-points ',horns :namespace namespace)))
                (dolist (hn hns) (send parts-instance :assoc hn))
                (setq (parts-instance . horns) hns))
             )
          (let ((fxd-pts (_parse-fixed-points ',fixed-points :namespace namespace)))
            (dolist (pt fxd-pts) (send parts-instance :assoc pt))
            (setq (parts-instance . fixed-points) fxd-pts))
          parts-instance))
     )
    (setf (gethash type *robot-assembler-functions-htable*) fname)
    ))
(defun _parse-geometry (slst)
  (labels ((parse-element
            (elm)
            (let ((type (cadr (assoc :type elm)))
                  (trs    (cadr (assoc :translation elm)))
                  (rot-an (cadr (assoc :rotation elm)))
                  (collision (cadr (assoc :collision elm)))
                  obj)
              (setq type (intern (string-upcase type) *keyword-package*))
              (setq obj
                    (case type
                      (:mesh
                       (let* ((org-fname (cadr (assoc :url elm)))
                              (scl   (cadr (assoc :scale elm)))
                              (fname org-fname)
                              )
                         (unless scl (setq scl 1.0))
                         (setq fname
                               (let (temp)
                                 (cond
                                  ((probe-file fname)
                                   fname)
                                  ((and (boundp '*project-dir*)
                                        (probe-file (setq temp (format nil "~A/~A" *project-dir* fname))))
                                   temp)
                                  (t nil)
                                  )))
                         (when fname
                           (let ((glv (load-mesh-file fname :scale scl)))
                             (send glv :put :filename fname)
                             (send glv :put :original-filename org-fname)
                             (send glv :put :scale scl)
                             glv
                             ))
                         ))
                       (t
                        (warn ";; parse-config / unkown type(geometry) ~A~%" type)
                        nil
                        )
                       ))
              (let ((pos (if trs (coerce trs float-vector)))
                    (rot (if rot-an (rotation-matrix (deg2rad (car (last rot-an)))
                                                     (subseq (coerce rot-an float-vector) 0 3))))
                    args cds
                    )
                (when rot (push rot args) (push :rot args))
                (when pos (push pos args) (push :pos args))
                (setq cds (apply #'make-coords args))
                (when obj
                  (send obj :transform cds))
                (when (and obj collision)
                  (send obj :put :collision-model collision))
                )
              (list obj))
            ))
    (mapcan #'(lambda (e) (parse-element e)) slst)
    ))
(defun _parse-fixed-points (slst &key (namespace))
  (labels ((parse-element
            (elm)
            (let ((name  (yaml-get-value elm :name  :no-key nil))
                  (types (yaml-get-value elm :types :no-key nil))
                  (cds (yaml-parse-trans-and-rot elm))
                  obj)
              (cond
               (namespace
                (setq name (intern (string-upcase
                                    (format nil "~A/~A" (string namespace) (string name)))
                                   *keyword-package*))
                )
               (t (setq name (intern (string-upcase name) *keyword-package*)))
               )
              (setq types
                    (mapcar #'(lambda (x) (intern (string-upcase x) *keyword-package*))
                            types))
              (setq obj (_make-roboasm-fixed-point types :name name))
              (if (and cds obj) (send obj :transform cds))
              (dolist (key (list :actuator-type :axis :limit :vlimit :tqlimit))
                (send obj key (yaml-get-value elm key :no-key nil)))
              (list obj))
            ))
    (mapcan #'(lambda (e) (parse-element e)) slst)
    ))
(defun _parse-extra-data (yaml-data)
  (let ((cds (yaml-parse-trans-and-rot yaml-data)))
    (if (not cds) (setq cds (make-coords)))
    (dolist (k (list :name :type :parameters))
      (let ((val (yaml-get-value yaml-data k)))
        (if (not (eq val *yaml-not-exist*))
            (send cds :put k (if (eq k :type)
                                 (intern (string-upcase val) *keyword-package*)
                               val)))
        ))
    cds
    ))

(defun _make-roboasm-fixed-point (types &key (name) (coords (make-coords)))
  (let ((ret (instance roboasm-fixed-point
                       :init (make-cascoords :coords coords)
                       :name name)))
    (unless (listp types) (return-from _make-roboasm-fixed-point))
    (let (fail)
      (dolist (tt types)
        (unless (gethash tt *fixed-point-htable*)
          (warn ";; _make-roboasm-fixed-point / type mismatch ~A~%" tt)
          (setq fail t)
          (return)))
      (when fail
        (return-from _make-roboasm-fixed-point))
      )
    (setq (ret . types) types)
    ;; geom / configuration...?
    ret
    ))

(defun move-inertia
    (weight inertia centroid new-rot new-centroid
            &key (tmp-ma (make-matrix 3 3))
            (tmp-mb (make-matrix 3 3))
            (tmp-mc (make-matrix 3 3))
            (tmp-md (make-matrix 3 3))
            (tmp-va (float-vector 0 0 0))
            )
  (labels
      ((DD (r)
           (let ((r2 (outer-product-matrix r tmp-ma)))
             (m* (transpose r2 tmp-mb) r2 tmp-mc))))
    ;; rotate inertia matrix
    (let ((ret (m* (m* new-rot inertia tmp-ma)
                   (transpose new-rot tmp-mb)
                   tmp-md)))
      ;; move weight
      (m+ (scale-matrix
           weight
           (DD (v- new-centroid centroid tmp-va)) tmp-ma)
          ret ret)
      ret)))

(defun parse-roboasm-list (a-hist &key (use-current-setting t))
  (let ((aconfig (assoc :assemble-config a-hist)))
    (if aconfig (setq a-hist (remove aconfig a-hist)))
    (unless (cadr (assoc :initial-parts (car a-hist)))
      (warn ";; history do not start-with :initial-parts~%")
      (return-from parse-roboasm-list))
    (unless use-current-setting
      ;; pase yaml-string
      )
    (labels ((make-parts-from-key
              (p-type p-name)
              (let ((fsymbol (gethash p-type *robot-assembler-functions-htable*)))
                (cond
                 ((fboundp fsymbol)
                  (funcall fsymbol :name p-name :namespace t))
                 (t (warn ";; do not find function for ~A~%" p-type)))
                )))
      (let (arobot ret)
        (let* ((p-type (cadr (assoc :parts-type (car a-hist))))
               (p-name (cadr (assoc :parts-name (car a-hist))))
               (p (make-parts-from-key p-type p-name)))
          (unless p (return-from parse-roboasm-list))
          (setq arobot (make-robot-assembler-robot p :name "parsed_from_ahist"))
          )
        (dolist (plist (cdr a-hist))
          (let* ((p-type (cadr (assoc :parts-type plist)))
                 (p-name (cadr (assoc :parts-name plist)))
                 (p-point (cadr (assoc :parts-point plist)))
                 (attach-point (cadr (assoc :robot-parts-point plist)))
                 (config (cadr (assoc :configuration plist)))
                 (p (make-parts-from-key p-type p-name))
                 r-p p-p
                 )
            (if p
                (progn
                  (setq r-p (find-if #'(lambda (x) (eq attach-point (send x :name))) (send arobot :active-fixed-points)))
                  (setq p-p (find-if #'(lambda (x) (eq p-point (send x :name))) (send p :active-fixed-points)))
                  (cond
                   ((and r-p p-p)
                    (cond
                     ((listp config)
                      (setq config (eval config)))
                     ((stringp config)
                      (setq config (eval (read-from-string config))))
                     )
                    (setq ret
                          (send arobot :attach p :parts-point p-p :robot-point r-p
                                :attach-configuration (if (eq config :default) nil config)))
                    (unless ret (warn ":attach error / ~A ~A ~A ~A~%" p p-p r-p config))
                    ) ;; (and r-p p-p)
                   (t
                    (warn ":fixed-point error / ~A ~A ~A ~A~%" attach-point p-point r-p p-p))
                   )
                  ) ;; progn
              (warn ":parts error / ~A ~A ~A~%" p-type p-name plist))
            ))
        (if aconfig (send arobot :read-assemble-config aconfig))
        arobot)
      ) ;; label
    ))
#|
(setq ht (parse-roboasm (with-open-file (f "filename.roboasm.l") (read f))))
(create-roboasm-from-parsed-table ht)
(create-roboasm-from-parsed-table (change-root-parts :new-root ht))

(setq ht-base (parse-roboasm (with-open-file (f "filename_base.roboasm.l") (read f))))
(setq ht-merged (parse-roboasm (with-open-file (f "filename_base.roboasm.l") (read f))))
(create-roboasm-from-parsed-table
 (merge-parsed-table ht-base ht-merged base-fixed-point merged-fixed-point))

|#
(defun point-key-to-body-and-point (key)
  (let* ((key_str (string key))
         (pos (position #\/ key_str)))
    (unless pos
      (return-from point-key-to-body-and-point nil))
    (cons
     (intern (subseq key_str 0 pos) *keyword-package*)
     (intern (subseq key_str (1+ pos)) *keyword-package*)
     )
    ))
(defun parse-roboasm (roboasm)
  (let ((parts-htbl (make-hash-table)))
    (dolist (p roboasm)
      (let* ((pname (cadr (assoc :parts-name p)))
             (rp (cadr (assoc :robot-parts-point p)))
             (p-fp (if rp (point-key-to-body-and-point rp)))
             )
        (cond
         (pname
          (when p-fp
            (push (list :parent (car p-fp)) p)
            (push (list :parents-point rp) p)
            )
          (setf (gethash pname parts-htbl) p))
         (t ;; :assemble-config
          (let ((key (car p)))
            (if (eq key :assemble-config)
                (send parts-htbl :enter key p))
            )))
        ))
    parts-htbl
    ))
(defun _to-root-list (parts parts-htbl)
  (labels ((parents-list
            (pt)
            (if (assoc :parent (gethash pt parts-htbl))
                (append (list pt) (parents-list (cadr (assoc :parent (gethash pt parts-htbl)))))
              (list pt))
             ))
     (parents-list parts)
     ))
(defun change-root-parts (new-root parts-htbl)
  (let ((to-root (_to-root-list new-root parts-htbl))
        from-root)
    (setq from-root (reverse to-root))
    (let ((org-parent (car from-root)))
      (dolist (org-child (cdr from-root))
        ;; swap parent-child
        (let* ((p-param (gethash org-parent parts-htbl))
               (c-param (gethash org-child parts-htbl))
               (c-point (cadr (assoc :parts-point c-param)))
               (p-point (cadr (assoc :parents-point c-param)))
               new-cp new-pp)
          (when (cadr (assoc :initial-parts p-param))
            (setf (gethash org-child parts-htbl)
                  (list (list :parts-name org-child)
                        (assoc :parts-type c-param)
                        (list :initial-parts t)))
            )
          (setf (gethash org-parent parts-htbl)
                (list (list :parts-name org-parent)
                      (assoc :parts-type p-param)
                      (list :parts-point p-point)
                      (list :robot-parts-point c-point)
                      (assoc :configuration c-param);;??
                      (list :parent org-child)
                      (list :parents-point c-point)
                      ))
          )
        (setq org-parent org-child)
        )
      parts-htbl
      )))
(defun create-roboasm-from-parsed-table (parts-htbl)
  (let ((initial
         (find-if #'(lambda (k) (assoc :initial-parts (gethash k parts-htbl))) (send parts-htbl :list-keys)))
        (a-config (send parts-htbl :get :assemble-config))
        keys cur-parts
        lst)
    (if a-config (send parts-htbl :delete :assemble-config))
    (push (gethash initial parts-htbl) lst)
    (remhash initial parts-htbl)
    (setq keys (send parts-htbl :list-keys))
    (while keys
      (setq cur-parts
            (find-if #'(lambda (k) (null (gethash (cadr (assoc :parent (gethash k parts-htbl))) parts-htbl)))
                     keys))
      #|(let* ((pk (cadr (assoc :parent (gethash cur-parts parts-htbl))))
             (p (gethash pk parts-htbl)))
        (pprint (list cur-parts pk p keys)))|#
      (let ((param (gethash cur-parts parts-htbl)))
        (setq param (remove-if #'(lambda (p) (or (eq :parent (car p))
                                                 (eq :parents-point (car p)))) param))
        (push param lst)
        )
      (remhash cur-parts parts-htbl)
      (setq keys (send parts-htbl :list-keys))
      )
    (if a-config (push a-config lst))
    (reverse lst)
    ;;
    ))
(defun merge-assemble-config (config-a config-b)
  (labels ((append-value-with-key (key)
            (let ((ka (cadr (assoc key config-a)))
                  (kb (cadr (assoc key config-b))))
              (if (or ka kb)
                  (list key (append ka kb)))))
           )
    (let (ret)
      (push (append-value-with-key :actuator-name) ret)
      (push (append-value-with-key :initial-angle) ret)
      (push (append-value-with-key :actuator-axis) ret)
      (push (assoc :robot-name config-a) ret)
      (push (assoc :initial-coords config-a) ret)
      (push :assemble-config ret)
      (remove nil ret)
      )))
(defun merge-parsed-table (parent-tbl child-tbl parent-point child-point &key (configuration))
  (let (;;(parent-initial (find-if #'(lambda (k) (assoc :initial-parts (gethash k parent-tbl))) (send parent-tbl :list-keys)))
        (child-initial (find-if #'(lambda (k) (assoc :initial-parts (gethash k child-tbl))) (send child-tbl :list-keys)))
        (parent-fp (point-key-to-body-and-point parent-point))
        (child-fp  (point-key-to-body-and-point child-point))
        )
    (unless (and parent-fp child-fp)
      (return-from merge-parsed-table))

    (unless (eq child-initial (car child-fp))
      ;; check child-point does not exist at root
      (setq child-tbl (change-root-parts (car child-fp) child-tbl))
      )

    (let ((ret-tbl (make-hash-table :size (+ (send child-tbl :size) (send parent-tbl :size))))
          (child-root-elem (gethash (car child-fp) child-tbl))
          (child-asm-config  (send child-tbl  :get :assemble-config))
          (parent-asm-config (send parent-tbl :get :assemble-config))
          )
      ;; merge parent-tbl and child-tbl -> ret-tbl
      (send parent-tbl :map #'(lambda (k v) (setf (gethash k ret-tbl) v))) ;; copy parent-tbl -> ret-tbl
      (send child-tbl  :map #'(lambda (k v) (setf (gethash k ret-tbl) v))) ;; copy child-tbl -> ret-tbl
      ;; add fixed-point to child's root-element
      (setq child-root-elem (remove (assoc :initial-parts child-root-elem) child-root-elem))
      (setq child-root-elem
            (append child-root-elem
                    (list (list :parent (car parent-fp))
                          ;;(:parents-point parent-point)
                          (list :robot-parts-point parent-point)
                          ;;(:parts-name )
                          ;;(:parts-type )
                          (list :parts-point child-point)
                          (list :configuration (if configuration configuration :default))
                          )))
      ;; overwrite ret-tbl
      (setf (gethash (car child-fp) ret-tbl) child-root-elem)

      (cond
       ((and child-asm-config parent-asm-config)
        (send ret-tbl :enter :assemble-config (merge-assemble-config parent-asm-config child-asm-config))
        )
       (parent-asm-config
        ;; do nothing, already in ret-tbl
        )
       (child-asm-config
        ;; do nothing, already in ret-tbl
        )
       )
      ret-tbl
      )
    ))
(defun rename-parts-name-on-parsed-table (old-tbl &key (suffix (string (gensym))) (start-counter 0))
  (let ((rename-tbl (make-hash-table))
        (new-tbl (make-hash-table))
        (cntr start-counter))
    (dolist (k (send old-tbl :list-keys))
      (let ((pt (cadr (assoc :parts-type (send old-tbl :get k)))))
        (if pt
            (let ((new-k
                   (intern (format nil "~A_~A_~D" (string pt) suffix cntr) *keyword-package*)))
              (send rename-tbl :enter k new-k)
              (incf cntr)))))
    (labels ((replace-assoc
              (in-list key val)
              (let ((ac (assoc key in-list)))
                (if ac (append (remove ac in-list) (list (list key val)))
                  in-list))
              )
             (rename-assemble-config
              (in)
              (let* ((parts-point (car in))
                     (val (cadr in))
                     (tmp (point-key-to-body-and-point parts-point))
                     (parts (car tmp))
                     (point (cdr tmp)))
                (list (intern (format nil "~A/~A" (string (send rename-tbl :get parts)) (string point)) *keyword-package*)
                      val)
                ))
             )
      (dolist (k (send old-tbl :list-keys))
        (cond
         ((eq k :assemble-config)
          (let* ((as-conf (cdr (send old-tbl :get k)))
                 (ac-name (cadr (assoc :actuator-name as-conf)))
                 (i-angle (cadr (assoc :initial-angle as-conf)))
                 (ac-axis (cadr (assoc :actuator-axis as-conf)))
                 new-conf)
            (if ac-name
                (push (list :actuator-name
                            (mapcar #'rename-assemble-config ac-name))
                      new-conf))
            (if i-angle
                (push (list :initial-angle
                            (mapcar #'rename-assemble-config i-angle))
                      new-conf))
            (if ac-axis
                (push (list :actuator-axis
                            (mapcar #'rename-assemble-config ac-axis))
                      new-conf))
            (if (assoc :robot-name as-conf)
                (push (assoc :robot-name as-conf) new-conf))
            (if (assoc :initial-coords as-conf)
                (push (assoc :initial-coords as-conf) new-conf))
            (push :assemble-config new-conf)
            (send new-tbl :enter k new-conf)
            ))
         (t
          (let* ((tbl-val (send old-tbl :get k))
                 (parts-name (cadr (assoc :parts-name tbl-val))) ;; name
                 (parent (cadr (assoc :parent tbl-val))) ;; name
                 (parents-p
                  (point-key-to-body-and-point (cadr (assoc :parents-point tbl-val)))) ;; name/point
                 (robot-pp
                  (point-key-to-body-and-point (cadr (assoc :robot-parts-point tbl-val)))) ;; name/point
                 (parts-point
                  (point-key-to-body-and-point (cadr (assoc :parts-point tbl-val)))) ;; name/point
                 )
            (setq tbl-val (replace-assoc tbl-val :parts-name (send rename-tbl :get parts-name)))
            (setq tbl-val (replace-assoc tbl-val :parent (send rename-tbl :get parent)))

            (setq tbl-val
                  (replace-assoc
                   tbl-val :parents-point
                   (intern (format nil "~A/~A" (string (send rename-tbl :get (car parents-p)))
                                   (string (cdr parents-p))) *keyword-package*)))
            (setq tbl-val
                  (replace-assoc
                   tbl-val :robot-parts-point
                   (intern (format nil "~A/~A" (string (send rename-tbl :get (car robot-pp)))
                                   (string (cdr robot-pp))) *keyword-package*)))
            (setq tbl-val
                  (replace-assoc
                   tbl-val :parts-point
                   (intern (format nil "~A/~A" (string (send rename-tbl :get (car parts-point)))
                                   (string (cdr parts-point))) *keyword-package*)))

            (send new-tbl :enter (send rename-tbl :get parts-name) tbl-val)
            ))
         )))
    new-tbl))
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

(defmethod coordinates
  (:translate-with-axis
   (distance axis &optional (wrt :local))
   (let ((fv
          (case axis
            (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
            (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
            (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
            (t nil))))
     (when (not fv)
       (if (float-vector-p axis)
           (setq axis (copy-seq axis))
         (if (listp axis)
             (setq axis (coerce axis float-vector))
           (error "")))
       (setq fv (normalize-vector axis axis)))
     (scale fv distance fv)
     (send self :translate fv wrt)
     ))
  )
