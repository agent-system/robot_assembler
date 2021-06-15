;;
(defmethod bodyset
  (:set-color
   (col &rest args)
   (send-all geo::bodies :set-color col)
   )
  (:faces-intersect-with-point-vector
   (pos v)
   (apply #'append (send-all geo::bodies :faces-intersect-with-point-vector pos v)))
  )
;;
(defclass pushed-hash-table
  :super hash-table
  :slots (set-test))
(defmethod pushed-hash-table
  (:push
   (key value)
   (let ((v (gethash key self)))
     (if (and v
              (or (null set-test)
                  (not (funcall set-test value v))))
         (sethash key self (push value v))
       (sethash key self (list value)))
     )
   )
  )
(defun make-pushed-hash-table (&key (size 10) (test #'eq) (rehash-size 1.7)
                                    (hash #'sxhash) (not-found nil) (set-test))
  (let ((i
         (instance pushed-hash-table :init :size size
                   :test test
                   :rehash-size rehash-size
                   :hash hash
                   :not-found not-found)))
    (if set-test (setq (i . set-test) set-test))
    i
    ))
;; string-pair hash-table
;;key : (cons "a" "b")
(defun string-pair-string (str)
  (let ((sa (string (car str)))
        (sb (string (cdr str))))
    (if (string< sa sb)
        (concatenate string sa sb)
      (concatenate string sb sa))))
(defun string-pair-test (keya keyb)
  (if (or (symbolp keya)
          (symbolp keyb)) nil
    (let ((a (string-pair-string keya))
          (b (string-pair-string keyb)))
      (string= a b))))
(defun string-pair-hash (str)
  (sxhash (string-pair-string str)))
(defun make-string-pair-hash-table (&key (size 10) (rehash-size 1.7) (not-found nil))
  (instance hash-table :init :size size
            :test #'string-pair-test
            :rehash-size rehash-size
            :hash #'string-pair-hash
            :not-found not-found))
;;
;; for ros-node
;;
(defun message-warn (&rest args)
  ;;(apply #'ros::ros-warn args)
  (apply #'format *error-output* args)
  (terpri)
  )
(defun message-info (&rest args)
  ;;(apply #'ros::ros-info args)
  (apply #'warn args)
  (terpri)
  )
;;
(defun make-small-axis (&key (radius 1.6) (length 16) (base-color 0))
  (let ((a-x (make-cylinder radius length))
        (a-y (make-cylinder radius length))
        (a-z (make-cylinder radius length))
        )
    (send a-x :set-color (float-vector 1 base-color base-color))
    (send a-y :set-color (float-vector base-color 1 base-color))
    (send a-z :set-color (float-vector base-color base-color 1))
    (send a-x :rotate pi/2 :y)
    (send a-y :rotate pi/2 :-x)

    (send a-z :assoc a-x)
    (send a-z :assoc a-y)
    (instance bodyset :init (make-cascoords)
              :bodies (list a-z a-x a-y))
    ))
;;
(defun axis-box (ax len p)
  (let ((cb (make-cube 1 1 len))
        (cc (make-cascoords)))
    (send cc :translate (float-vector 0 0 (- (* 0.5 len))))
    (send cb :assoc cc)

    (send cb :transform (orient-coords-to-axis (make-coords) ax))

    (send cb :translate (v- p (send cc :worldpos)) :world)
    cb
    ))
;;
(defun write-coords-as-string (cds)
  (let* ((cc (send cds :worldcoords))
         (pos (send cc :pos))
         (rpy (car (send cc :rpy-angle))))
    (format nil "(make-coords :pos (float-vector ~12,12F ~12,12F ~12,12F) :rpy (float-vector ~12,12F ~12,12F ~12,12F))"
            (elt pos 0) (elt pos 1) (elt pos 2)
            (elt rpy 0) (elt rpy 1) (elt rpy 2))))
(defun parse-coordinates-from-string (str)
  (let ((exp
         (if (= (elt str 0) #\()
             (read-from-string str)
           (read-from-string (format nil "(~A)" str))))
        (cds (make-coords)))
    (while exp
      (cond
       ((eq (car exp) :rotate)
        (let ((ang (cadr exp))
              (axs (caddr exp)))
          (send cds :rotate (eval ang) (eval axs)))
        (setq exp (cadddr exp))
        )
       ((eq (car exp) :rpy)
        (let ((rpy (cadr exp))
              (pos (send cds :pos)))
          (setq cds (make-coords :rpy (eval rpy)))
          (send cds :locate pos :world))
        (setq exp (caddr exp))
        )
       ((eq (car exp) :translate)
        (let ((trx (cadr exp)))
          (send cds :translate (eval trx)))
        (setq exp (caddr exp))
        )
       ((eq (car exp) :pos)
        (let ((pos (cadr exp)))
          (send cds :locate pos :world)
          )
        (setq exp (caddr exp))
        )
       (t
        (setq cds (eval exp))
        (setq exp nil)
        )
       ))
    cds
    ))
;;
(defun make-default-fixed-point-type-geometry
    (fixp &key (base 2.5) (x-len 12) (y-len 15) (z-len 10) ((:scale scl)))
  (when scl
    (setq base  (* scl base)
          x-len (* scl x-len)
          y-len (* scl y-len)
          z-len (* scl z-len)))
  (let* ((z-axis (make-cube base base (+ base z-len)))
         (y-axis (make-cube base y-len base))
         (x-axis (make-cube x-len base base))
         (lst (list z-axis y-axis x-axis))
         ret
         )
    (send z-axis :translate (float-vector 0 0 (* 0.5 z-len)))
    (send y-axis :translate (float-vector 0 (* 0.5 (+ base y-len)) 0))
    (send x-axis :translate (float-vector (* 0.5 (+ base x-len)) 0 0))
    (send z-axis :assoc x-axis)
    (send z-axis :assoc y-axis)
    (send-all lst :put :related-fixed-point fixp)
    (prog1
        (setq ret (instance bodyset :init (make-cascoords) :bodies lst))
      (send ret :put :related-fixed-point fixp)
      (send ret :transform (send fixp :worldcoords)))
    ))
;;
;;
;;
(defun make-horn12-fixed-point-type-geometry ()
  (let ((hr (make-cylinder 9 1.5 :segments 12))
        (sh (make-cylinder 2 1.0 :segments 32))
        (aa (make-cube 2 5 10))
        )
    (send aa :translate (float-vector 0 9 0))
    (send sh :translate (float-vector 0 0 1.0))
    (setq hr (body- hr sh))
    (setq hr (body- hr aa))
    (instance bodyset :init (make-cascoords) :bodies (list hr))
    ))
(defun make-horn12-hole-fixed-point-type-geometry ()
  (let ((hr (make-cylinder 9 1.5 :segments 12))
        (sh (make-cylinder 2 1.0 :segments 32))
        ;;(aa (make-cube 5 2 10))
        (aa (make-cube 2 5 10))
        )
    ;;(send aa :translate (float-vector 6 0 0))
    (send aa :translate (float-vector 0 9 0))
    (send sh :translate (float-vector 0 0 1.0))
    (setq hr (body+ hr sh))
    (setq hr (body- hr aa))
    (send hr :rotate pi :y)
    (instance bodyset :init (make-cascoords) :bodies (list hr))
    ))
(defun make-bolt12_0-tap-fixed-point-type-geometry ()
  (let ((bs (make-cube  10 20 1))
        (mz (make-cube 3.4 15 1)))
    (send mz :translate (float-vector 0 0 0.5))
    (setq bs (body- bs mz))
    (send bs :translate (float-vector 0 0 0.5))
    (instance bodyset :init (make-cascoords) :bodies (list bs))
    ))
(defun make-bolt12_0-hole-fixed-point-type-geometry ()
  (let ((bs (make-cube  10 20 1))
        (mz (make-cube 3.4 15 1)))
    (send mz :translate (float-vector 0 0 0.5))
    (setq bs (body+ bs mz))
    (send bs :translate (float-vector 0 0 0.5))
    (send bs :rotate pi :y)
    (instance bodyset :init (make-cascoords) :bodies (list bs))
    ))
;;
;; for debug
;;
(defun draw-coords (r)
  (let ((bds (list r)))
    (dolist (p (send r :all-parts roboasm-fixed-point))
      (when (eq roboasm-fixed-point (class p))
        (let ((ar (make-small-axis)))
          (send ar :transform (send p :worldcoords))
          (push ar bds))
        ))
    (objects bds)
    ))
(defun draw-fixed-coords (r)
  (let ((bds (list r)))
    (dolist (p (send r :all-parts roboasm-fixed-point))
      (when (eq roboasm-fixed-point (class p))
        (let ((tp-lst (p . types)))
          (dolist (tp tp-lst)
            (let ((ff (intern (string-upcase (format nil "make-~A-fixed-point-type-geometry" (string tp)))))
                  bd)
              (when t ;;(fboundp ff)
                ;;(setq bd (funcall ff))
                (setq bd (make-default-fixed-point-type-geometry))
                (send bd :newcoords (send p :worldcoords))
                (push bd bds)
                )
              ))
          ))
      )
    (objects bds)
    ))
;;;
(defun initial-mass-param (parts mass)
  (labels ((make-box-inertia
            (mass xx yy zz)
            (let ((ret (make-matrix 3 3)))
              (setf (aref ret 0 0) (/ (* mass (+ (* yy yy) (* zz zz))) 12))
              (setf (aref ret 1 1) (/ (* mass (+ (* xx xx) (* zz zz))) 12))
              (setf (aref ret 2 2) (/ (* mass (+ (* yy yy) (* xx xx))) 12))
              (setq ret (scale 1e-9 (array-entity ret)))

              (format t "inertia-tensor: [~A, ~A, ~A,  ~A, ~A, ~A,  ~A, ~A, ~A]~%"
                      (elt ret 0) (elt ret 1) (elt ret 2)
                      (elt ret 3) (elt ret 4) (elt ret 5)
                      (elt ret 6) (elt ret 7) (elt ret 8))
              )))
    (let* ((bb
            (make-bounding-box (send (car (send parts :bodies)) :vertices) 0.0))
           (size (send bb :diagonal))
           (cent (scale 0.001 (send bb :center)))
           (xx (elt size 0))
           (yy (elt size 1))
           (zz (elt size 2))
           )
      (format t "## ~A~%" (parts . type))
      (format t "mass: ~A~%" (* 0.001 mass))
      (format t "center-of-mass: [~A, ~A, ~A]~%" (elt cent 0) (elt cent 1) (elt cent 2))
      (make-box-inertia mass xx yy zz)
      )))
;;;
(defun make-bounding-cube-from-robot-link (*robot* &key ((:scale scl) 0.001))
  (mapcar #'(lambda (r)
              (let ((cds (send r :copy-worldcoords))
                    (pts (flatten (send-all (send r :bodies) :vertices))))
                (setq pts (mapcar #'(lambda (p) (send cds :inverse-transform-vector p)) pts))
                (let* ((bb (make-bounding-box pts 0.0))
                       (cb (apply #'make-cube (coerce (send bb :diagonal) cons)))
                       )
                  (format t "box: ~A / trans: ~A~%" (scale scl (send bb :diagonal)) (scale scl (send bb :center)))
                  (send cds :transform (make-coords :pos (send bb :center)))
                  (send cb :newcoords cds)

                  cb))) (send *robot* :links)))

;;;; should be commit
(in-package "GL")
(defmethod bounding-box
#|
draw 8 lines by #f(minx miny minz)/minpoint and #f(maxx maxy maxz)/minpoint
#f(minx miny minz)/min0 -> #f(minx maxy minz)/min1 -> #f(maxx maxy minz)/min3
#f(minx miny minz)/min0 -> #f(maxx miny minz)/min2 -> #f(maxx maxy minz)/min3

#f(maxx maxy maxz)/max0 -> #f(minx maxy maxz)/min1 -> #f(minx miny maxz)/min3
#f(maxx maxy maxz)/max0 -> #f(maxx miny maxz)/min2 -> #f(minx miny maxz)/min3

#f(minx miny minz)/min0 -> #f(minx miny maxz)/max3
#f(minx maxy minz)/min1 -> #f(minx maxy maxz)/max1
#f(maxx miny minz)/min2 -> #f(maxx miny maxz)/max2
#f(maxx maxy minz)/min3 -> #f(maxx maxy maxz)/max0
|#
  (:draw-on
   (&key ((:viewer vwer) *viewer*)
         flush (width 1) (color #f(1 1 1)))
   (if flush (send vwer :viewsurface :makecurrent))
   (let* ((pwidth (send vwer :viewsurface :line-width))
          (pcolor (send vwer :viewsurface :color))
          (v (float-vector 0 0 0))
          (minx (elt geometry::minpoint 0))
          (miny (elt geometry::minpoint 1))
          (minz (elt geometry::minpoint 2))
          (maxx (elt geometry::maxpoint 0))
          (maxy (elt geometry::maxpoint 1))
          (maxz (elt geometry::maxpoint 2))
          (min0 geometry::minpoint)
          (min1 (float-vector minx maxy minz))
          (min2 (float-vector maxx miny minz))
          (min3 (float-vector maxx maxy minz))
          (max0 geometry::maxpoint)
          (max1 (float-vector minx maxy maxz))
          (max2 (float-vector maxx miny maxz))
          (max3 (float-vector minx miny maxz))
          )
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     ;;
     (send vwer :viewsurface :3d-line min0 min1)
     (send vwer :viewsurface :3d-line min1 min3)
     (send vwer :viewsurface :3d-line min0 min2)
     (send vwer :viewsurface :3d-line min2 min3)
     ;;
     (send vwer :viewsurface :3d-line max0 max1)
     (send vwer :viewsurface :3d-line max1 max3)
     (send vwer :viewsurface :3d-line max0 max2)
     (send vwer :viewsurface :3d-line max2 max3)
     ;;
     (send vwer :viewsurface :3d-line min0 max3)
     (send vwer :viewsurface :3d-line min1 max1)
     (send vwer :viewsurface :3d-line min2 max2)
     (send vwer :viewsurface :3d-line min3 max0)
     ;;
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (if flush (send vwer :viewsurface :flush))
     ))
  )
(in-package "USER")
