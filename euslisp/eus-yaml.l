(eval-when (load)
;; compiled code
(require :eus-yaml   "package://robot_assembler/euslisp/eus_yaml_cpp.so") ;; loading c-parse-yaml-file
)
(eval-when (eval compile)
;; source code
(require :eus-yaml   "eus_yaml_cpp.so") ;; loading c-parse-yaml-file
)

(defvar *yaml-not-exist* :yaml-not-existing-key)

(defun yaml-map? (yaml-data)
  (every #'(lambda (x) (keywordp (car x))) yaml-data))
(defun yaml-list? (yaml-data)
  (and (listp yaml-data)
       (not (yaml-map? yaml-data))))
(defun yaml-scalar? (yaml-data)
  (not (listp yaml-data)))

(defun write-yaml (yaml-data &key (fmt t) (offset "") (inline t))
  (cond
   ((not (listp yaml-data)) ;; scalar
    (write-yaml-scalar yaml-data :fmt fmt :offset offset :inline inline)
    )
   ((every #'(lambda (x) (keywordp (car x))) yaml-data) ;; list
    (write-yaml-map yaml-data :fmt fmt :offset offset :inline inline)
    )
   (t ;; list
    (write-yaml-list yaml-data :fmt fmt :offset offset :inline inline)
    )
   ))
(defun write-yaml-map (yaml-data &key (fmt) (offset "") (inline t))
  (if inline
      (progn
        (format fmt "~A{" offset)
        (dolist (elm yaml-data)
          (format fmt "~A: " (string (car elm)))
          (write-yaml (cadr elm) :fmt fmt :inline t)
          (format fmt ", ")
          )
        (format fmt "}")
        )
    (dolist (elm yaml-data)
      (format fmt "~A~A: " offset (car elm))
      (write-yaml (cadr elm) :fmt fmt)
      (format fmt "~%")
      )
    ))
(defun write-yaml-list (yaml-data &key (fmt) (offset "") (inline t))
  (if inline
      (progn
        (format fmt "~A[" offset)
        (dolist (elm yaml-data)
          (write-yaml elm :fmt fmt :inline t)
          (format fmt ", ")
          )
        (format fmt "]")
        )
    (dolist (elm yaml-data)
      (format fmt "~A-~%" offset)
      (write-yaml elm :fmt fmt :offset (format nil "~A  " offset))
      (format fmt "~%")
      )
    ))
(defun write-yaml-scalar (yaml-data &key (fmt) (offset "") (inline t))
  (cond
   ((stringp yaml-data)
    (format fmt "~A\"~A\"" offset yaml-data)
    )
   (t
    (format fmt "~A~A" offset yaml-data))
   )
  )

(defun yaml-list-keys (yaml-data)
  (if (not (yaml-map? yaml-data)) (return-from yaml-list-keys nil))
  (mapcar #'(lambda (x) (car x)) yaml-data))

(defun yaml-key-exist? (yaml-data key)
  (assoc key yaml-data))

(defun yaml-get-value (yaml-data key &key (no-key *yaml-not-exist*))
  (let ((data (yaml-key-exist? yaml-data key)))
    (if data (cadr data) no-key)))

(defun yaml-parse-trans-and-rot (yaml-data)
  (let ((trs (yaml-get-value yaml-data :translation :no-key nil))
        (rot (yaml-get-value yaml-data :rotation    :no-key nil))
        args)
    ;;
    (if trs (setq trs (coerce trs float-vector)))
    (if rot (setq rot (rotation-matrix (deg2rad (car (last rot)))
                                       (subseq (coerce rot float-vector) 0 3))))
    (when (or trs rot)
      (when trs (push trs args) (push :pos args))
      (when rot (push rot args) (push :rot args))
      (apply #'make-coords args)
      )
    ))
