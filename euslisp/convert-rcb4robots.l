;;;;
;;;; code for rcb4eus
;;;;
(load "package://robot_assembler/euslisp/robot-assembler-utils.l")

;; roseus eus2wrl.l
;;(load "package://robot_assembler/euslisp/convert-rcb4robots.l")
;;(load-parts-association-file "/tmp/ra_parts_attach_list.l")
;;(setq *robot* (rename-and-add-bbox (kxr-make-robot "kxrl2l6a6h2")))
;;(setq ht (convert-robot-to-roboasm *robot*))
;;(dump-hash "/tmp/roboasm.l" ht)
;; -> (setq ht (load-hash "/tmp/roboasm.l"))
;; -> (setq roboasm (create-roboasm-from-parsed-table ht))
;; -> (setq rbt (make-robot-from-roboasm roboasm))
;; load file which was created by make-parts-association-file
(defun load-parts-association-file
    (&optional (fname "/tmp/ra_parts_attach_list.l"))
  ;;
  (setq *ra-parts-attach-list* (with-open-file (f fname) (read f)))
  ;;
  (setq *ra-parts-tbl* (make-hash-table :test #'equal))
  ;;
  (dolist (p *ra-parts-attach-list*)
    (let ((key
           (list (car  p)
                 (cadr p))))
      (setf (gethash key *ra-parts-tbl*) (caddr p))
      ))

  (setq *ra-actuator-tbl* (make-hash-table :test #'equal))
  (send *ra-parts-tbl* :map #'(lambda (k v)
                                (when (eq (car k) :s3301)
                                  (let ((res (remove-if-not #'(lambda (x) (eq (cadr x) :horn)) v)))
                                    (when res
                                      (setf (gethash k *ra-actuator-tbl*) res))))))
  ;;
  (setq *actuator-attach-list*
        (list
         (list :s3301 :joint-base
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 0 0 -2.5)))
         (list :s3301 :wheel
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 0 0 -2.0)))
         (list :s3301 :upper-arm20
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 17) :rpy (list 0 pi/2 0)))
         (list :s3301 :upper-arm26
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 23) :rpy (list 0 pi/2 0)))
         (list :s3301 :upper-arm38
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 35) :rpy (list 0 pi/2 0)))
         (list :s3301 :cross-upper-arm28
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 25) :rpy (list 0 pi/2 pi)))
         (list :s3301 :bottom-arm20 ;; upper<->bottom間違い/サーボ反転
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 17) :rpy (list 0 pi/2 0)))
         (list :s3301 :bottom-arm26
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 23) :rpy (list 0 pi/2 0)))
         (list :s3301 :bottom-arm38
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 35) :rpy (list 0 pi/2 0)))
         (list :s3301 :cross-bottom-arm28
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 4.5 0 25) :rpy (list 0 pi/2 pi)))
         (list :s3301 :finger
               (make-coords :pos (float-vector 0 0 18) :rpy (list pi 0 0))
               ;;(make-coords :pos (float-vector 21 10.5 18) :rpy (list pi 0 0))
               ;;(make-coords :pos (float-vector 21 -10.5 18) :rpy (list pi 0 0))
               (make-coords :pos (float-vector 2.5 0 0) :rpy (list pi/2 0 pi/2))
               ;;(make-coords :pos (float-vector -33.5 0 0) :rpy (list pi/2 0 -pi/2))
               )
         ))
  (setq *actuator-attach-table* (make-hash-table :test #'equal))
  (dolist (a *actuator-attach-list*)
    (setf (gethash (list (car a) (cadr a)) *actuator-attach-table*) a))
  t
  )

(defun fix-parts (pt func)
  (let ((p (send pt :parent))
        (ds (send pt :descendants)))
    (dolist (d ds)
      (send pt :dissoc d))
    (if p (send p :dissoc pt))
    ;;
    (funcall func pt)
    ;;
    (dolist (d ds)
      (send pt :assoc d))
    (if p (send p :assoc pt))
    pt
    ))

(defun adjacent? (a b)
  (send (send a :get :bbox) :intersection-p (send b :get :bbox))
  )
(defun estimate-coords-equal (aa bb &key (pos-thre 2) (rot-thre 0.4))
  (let ((res (coordinates-distance aa bb)))
    (and (< (abs (car res)) pos-thre)
         (or (not (cadr res))
             (< (abs (cadr res)) rot-thre)))
    ))
(defun can-attach (a b &key (pre-test) (table *ra-parts-tbl*) (pos-thre 2) (rot-thre 0.07) (debug nil))
  (when (or (not pre-test) (funcall pre-test a b))
    (let ((at-lst (gethash (list (send a :name) (send b :name)) table)))
      (if debug (pprint (list at-lst a b)))
      (if (not at-lst)
          (return-from can-attach nil))
      (let* ((cc (send a :transformation b))
             (res (find-if #'(lambda (at) (estimate-coords-equal
                                           cc (car at) :pos-thre pos-thre :rot-thre rot-thre)) at-lst)))
        (if debug (unless res (pprint (list :fail cc a b))))
        res)
      )))
(defun ra-name (name)
  (cond
   ((string= name "upper-arm28")
    "cross-upper-arm28")
   ((string= name "bottom-arm28")
    "cross-bottom-arm28")
   ((string= name "finger-a")
    "finger")
   ((string= name "head")
    "euclid-head")
   (t
    name)))
(defun bottom-to-upper (key)
  (case key
    (:bottom-arm20
     :upper-arm20)
    (:bottom-arm26
     :upper-arm26)
    (:bottom-arm38
     :upper-arm38)
    (:cross-bottom-arm28
     :cross-upper-arm28)
    (t key)))
(defun upper-to-bottom (key)
  (case key
    (:upper-arm20
     :bottom-arm20)
    (:upper-arm26
     :bottom-arm26)
    (:upper-arm38
     :bottom-arm38)
    (:cross-upper-arm28
     :cross-bottom-arm28)
    (t key)))
(defun rename-and-add-bbox (robot &key (fat 0.015) (bottom-to-upper t))
  (let ((name-counter 0))
    (dolist (lk (send robot :links))
      (dolist (bd (send lk :bodies))
        (let* ((bx (make-bounding-box
                    ;;(send bd :vertices)
                    (mapcar #'(lambda (v)
                                (send (send bd :worldcoords) :transform-vector v))
                            (send (bd . gl::aglvertices) :vertices))
                    fat))
               (kn (intern (string-upcase (ra-name (send bd :name))) *keyword-package*))
               (id (intern (string-upcase (format nil "~A-~D" (string kn) name-counter))
                           *keyword-package*))
               )
          (incf name-counter)
          (if bottom-to-upper
              (send bd :name (bottom-to-upper kn))
            (send bd :name kn))
          (send bd :put :bbox bx)
          (send bd :put :id id)
          )))
    (send robot :init-pose)
    robot))

;; (link-adjacent-list (find-if #'(lambda (x) (derivedp x kxr-body)) (send lk :descendants)) (send lk :bodies))
(defun link-adjacent-list (root-parts parts &key
                                      (pre-test #'adjacent?)
                                      (test #'can-attach)
                                      (debug)
                                      )
  (let* ((rt root-parts)
         (childs (remove rt parts))
         (poped (list rt))
         res)
    (while (and childs
                (block :hoge
                  (dolist (p poped)
                    (dolist (c childs)
                      (when (setq testres
                                  (and (funcall pre-test p c)
                                       (funcall test p c)))
                        (push (list p c testres) res)
                        (push c poped)
                        (setq childs (remove c childs))
                        (return-from :hoge t))))
                  nil
                  )))
    (while (and childs
                (block :hoge
                  (dolist (p poped)
                    (dolist (c childs)
                      (when (setq testres (funcall test p c))
                        (push (list p c testres) res)
                        (push c poped)
                        (setq childs (remove c childs))
                        (return-from :hoge t))))
                  nil
                  )))
    (list childs (reverse res))
    ))

(defun parts-offset (a-name b-name transformation)
  (let ((att (gethash (list a-name b-name) *actuator-attach-table*)))
    (unless att
      (return-from parts-offset nil))
    (let ((a-name (car att))
          (b-name (cadr att))
          (a-point (caddr att))
          (b-point (cadddr att))
          cds)
      (setq cds
            (send (send (send a-point :inverse-transformation)
                        :transform transformation)
                  :transform b-point))
      cds)))

(defun search-joint-attach (lk &key (target-parts (list :upper-arm20 :upper-arm26 :upper-arm38 :cross-upper-arm28
                                                        :joint-base :wheel :finger
                                                        :bottom-arm20 :bottom-arm26 :bottom-arm38 :cross-bottom-arm28
                                                        ))
                               (debug nil))
  (let ((adjacent-links
         (remove-if-not #'(lambda (d) (derivedp d bodyset-link)) (send lk :descendants)))
        res-lst)
    (when (and (send lk :parent) (derivedp (send lk :parent) bodyset-link))
      (push (send lk :parent) adjacent-links))
    (unless adjacent-links ;; tip-link??
      (return-from search-joint-attach nil))
    (dolist (act (remove-if-not #'(lambda (b) (eq (send b :name) :s3301)) (send lk :bodies)))
      (let (tmp-res)
        (dolist (al adjacent-links)
          (let ((bds (remove-if-not #'(lambda (b) (memq (send b :name) target-parts)) (send al :bodies)))
                res)
            (dolist (bd bds)
              (when (adjacent? act bd)
                (let ((cds (send (send act :worldcoords) :transformation (send bd :worldcoords)))
                      offset-res)
                  (setq offset-res
                        (parts-offset (send act :name) (send bd :name) cds))
                  (if debug
                      (pprint (list act bd (if (eq (send lk :parent) al)
                                               (send lk :joint)
                                             (send al :joint)) offset-res)))
                  (when (and offset-res (estimate-coords-equal offset-res (make-coords) :pos-thre 1.0 :rot-thre 10))
                    (push (list act bd (if (eq (send lk :parent) al)
                                           (send lk :joint)
                                         (send al :joint))
                                offset-res
                                (eq (send lk :parent) al)
                                )
                          tmp-res))
                  )))))
        (when tmp-res
          (dolist (r tmp-res)
            (push r res-lst)))
        ;;
        (unless tmp-res
          (warn ";; actuator not found for ~A (~A)~%" act (send act :get :id))
          #|
          (dolist (al adjacent-links)
            (let ((bds (remove-if-not #'(lambda (b) (memq (send b :name) target-parts)) (send al :bodies)))
                  res)
              (dolist (bd bds)
                (let ((cds (send (send act :worldcoords) :transformation (send bd :worldcoords)))
                      offset-res)
                  (setq offset-res
                        (parts-offset (send act :name) (send bd :name) cds))
                  (push (list act bd (if (eq (send lk :parent) al)
                                         (send lk :joint)
                                       (send al :joint))
                              offset-res
                              (eq (send lk :parent) al)
                              )
                        tmp-res)))))
          ;;(pprint tmp-res)
          (let ((res (find-extream tmp-res #'(lambda (x) (norm (send (cadddr x) :pos))) #'<)))
            ;;(pprint res)
            (warn ";; actuator (~A) was assumed to ~A~%" act res)
            (if res (push res res-lst)))
          |#
          )
        ;;
        ))
    res-lst
    ))

(defun parse-joint-attach-result (lk res &key (table) (debug))
  (let ((bd0 (car  res))
        (bd1 (cadr res))
        (jt  (caddr res))
        (cds (cadddr res))
        (child? (caddddr res))
        ret
        )
    (if child? ;; swap
        (let ((tmp bd0))
          (setq bd0 bd1
                bd1 tmp)))
    (cond
     ((eq (send jt :child-link) lk)
      ;; nothing to do
      ;; parent-link already processed
      ;;(send jt :parent-link)
      (if (send lk :get :actuated-parts)
          (warn ";; :actuated-parts already set (~A)~%" (send lk :get :actuated-parts)))
      (send lk :put :actuated-parts bd1)
      )
     ((eq (send jt :parent-link) lk)
      (if child?
          (warn ";; illegal childs? (~A)~%" lk))
      (if (send (send jt :child-link) :get :actuated-parts)
          (warn ";; :actuated-parts already set (~A)~%" (send (send jt :child-link) :get :actuated-parts)))
      (send (send jt :child-link) :put :actuated-parts bd1)
      ))

    (let ((res (can-attach bd0 bd1 :pos-thre 2.0 :rot-thre 0.01 :debug debug))
          fx0 fx1 config)
      (cond
       (res
        (setq fx0 (cadr res))
        (setq fx1 (caddr res))
        (setq config (cadddr res))
        )
       (t
        (cond
         ((eq (send bd0 :name) :s3301)
          (setq fx0 :horn fx1 :horn-hole)
          ;; invert cds ??
          (setq cds (send cds :inverse-transformation))
          )
         (t ;; s3301 is child-parts
          (setq fx1 :horn fx0 :horn-hole)
          ))
        ;;
        (setq config (write-coords-as-string (make-coords :rpy (list (caar (send cds :rpy-angle)) 0 0))))
        ))
      (when table
        (let ((pt (intern (string-upcase (format nil "~A/~A" (string (send bd1 :get :id)) (string fx1))) *keyword-package*))
              (rt (intern (string-upcase (format nil "~A/~A" (string (send bd0 :get :id)) (string fx0))) *keyword-package*))
              )
          #|
          (pprint
                (list
                 (list :parts-name (send bd1 :get :id))
                 (list :parts-type (send bd1 :name))
                 (list :parts-point pt)
                 (list :robot-parts-point pt)
                 (list :configuration config)
                 (list :parent (send bd0 :get :id))
                 (list :parents-point rt)))
          |#
          (when (gethash (send bd1 :get :id) table)
            (warn ";; ~A (~A) already in table/joint ~%" (send bd1 :get :id) bd1))
          (if debug (warn ";## push(joint) ~A~%" (send bd1 :get :id)))
          (cond
           ((eq (send bd0 :name) :s3301)
            ;;(list rt (send jt :name))
            (send jt :put :actuator-point rt)
            (send jt :put :actuator-parts (send bd0 :get :id))
            )
           (t
            ;;(list pt (send jt :name))
            (send jt :put :actuator-point pt)
            (send jt :put :actuator-parts (send bd1 :get :id))
            ))
          (setf (gethash (send bd1 :get :id) table)
                (list
                 (list :parts-name (send bd1 :get :id))
                 (list :parts-type (send bd1 :name))
                 (list :parts-point pt)
                 (list :configuration config)
                 (list :parent (send bd0 :get :id))
                 (list :parents-point rt)
                 (list :robot-parts-point rt)
                 ))
          ))
      )))

(defun sorted-links (robot)
  ;; result should be parent-link first
  (let ((lk-lst (cdr (send robot :links)))
        (rt (car (send robot :links)))
        (tmp-hash (make-hash-table))
        (cntr 0) result
        )
    (setf (gethash rt tmp-hash) cntr)
    (push rt result)
    (incf cntr)
    (while lk-lst
      (dolist (lk lk-lst)
        (unless (gethash lk tmp-hash)
          (setf (gethash lk tmp-hash) cntr)
          (incf cntr)
          (push lk result)
          (return)))
      (pop lk-lst)
      )
    (nreverse result)
    ))

(defun convert-robot-to-roboasm (robot &key (debug nil))
  (dolist (lk (send robot :links))
    (send lk :put :actuated-parts nil))

  (let ((lk-lst (sorted-links *robot*))
        (table (make-hash-table)))
    (dolist (lk lk-lst)
      (if debug (warn ";## start with link (~A)~%" lk))
      (let ((res (search-joint-attach lk)))
        ;;
        (dolist (r res)
          (parse-joint-attach-result lk r :table table :debug debug))
        (let ((ac-pt (send lk :get :actuated-parts))
              (cur-bds (send lk :bodies))
              bds
              )
          (setq cur-bds (remove ac-pt cur-bds))
          (dolist (bd cur-bds)
            (unless (gethash (send bd :get :id) table)
              (push bd bds)))
          (setq bds (nreverse bds))
          (unless ac-pt
            (setq ac-pt (car bds)))
          (let ((ret (link-adjacent-list ac-pt bds))
                (root? (eq lk (car lk-lst)))
                )
            (when (car ret)
              (warn ";; These parts (~A) can't fixed~%"
                    (mapcar #'(lambda (c) (list c (send c :get :id))) (car ret))
                    ))
            (dolist (r (cadr ret))
              (let ((bd0 (car r))
                    (bd1 (cadr r))
                    (fx0 (cadr (caddr r)))
                    (fx1 (caddr (caddr r)))
                    (config (cadddr (caddr r))))
                (let ((pt (intern (string-upcase (format nil "~A/~A" (string (send bd1 :get :id)) (string fx1))) *keyword-package*))
                      (rt (intern (string-upcase (format nil "~A/~A" (string (send bd0 :get :id)) (string fx0))) *keyword-package*))
                      )
                  (when (gethash (send bd1 :get :id) table)
                    (warn ";; ~A (~A) already in table/parts ~%" (send bd1 :get :id) bd1))
                  (cond
                   (root?
                    (when (gethash (send bd0 :get :id) table)
                      (warn ";; ~A (~A) already in table/parts ~%" (send bd0 :get :id) bd0))
                    (if debug (warn ";## push(parts) ~A~%" (send bd0 :get :id)))
                    (setf (gethash (send bd0 :get :id) table)
                          (list
                           (list :initial-parts t)
                           (list :parts-name (send bd0 :get :id))
                           (list :parts-type (send bd0 :name))
                           ))
                    (if debug (warn ";## push(parts) ~A~%" (send bd1 :get :id)))
                    (setf (gethash (send bd1 :get :id) table)
                          (list
                           (list :parts-name (send bd1 :get :id))
                           (list :parts-type (send bd1 :name))
                           (list :parts-point pt)
                           (list :configuration config)
                           (list :parent (send bd0 :get :id))
                           (list :parents-point rt)
                           (list :robot-parts-point rt)
                           ))
                    (setq root? nil)
                    )
                   (t
                    (if debug (warn ";## push(parts) ~A~%" (send bd1 :get :id)))
                    (setf (gethash (send bd1 :get :id) table)
                          (list
                           (list :parts-name (send bd1 :get :id))
                           (list :parts-type (send bd1 :name))
                           (list :parts-point pt)
                           (list :configuration config)
                           (list :parent (send bd0 :get :id))
                           (list :parents-point rt)
                           (list :robot-parts-point rt)
                           ))
                    ))
                  )))
            ))))
    (let (act-conf axis-conf nm)
      (dolist (jt (send robot :joint-list))
        (when (send jt :get :actuator-point)
          (setq nm (send jt :name))
          (if (keywordp nm)
              (setq nm (string-downcase (string nm))))
          (push
           (list (send jt :get :actuator-point) nm)
           act-conf)
          (let ((pt (send jt :get :actuator-parts)))
            (when pt
              (let ((bd (find-if #'(lambda (x) (eq (send x :get :id) pt)) (send robot :bodies)))
                    ax)
                (setq ax (send (send bd :worldcoords) :inverse-rotate-vector (send jt :axis)))
                (cond
                 ((eps-v= ax #f(0 0  1))
                  ;;(push (list (send jt :get :actuator-point) :z) axis-conf)
                  ;; do nothing
                  )
                 ((eps-v= ax #f(0 0 -1))
                  (push (list (send jt :get :actuator-point) :-z) axis-conf)
                  )
                 (t
                  (warn ";; joint(~A) has incorrect axis ~A~%" jt ax)
                  ))
                )))
          ))
      (when (or act-conf axis-conf)
        (let ((conf (list (list :robot-name (send robot :name))
                          :assemble-config)))
          (if act-conf
              (push (list :actuator-name act-conf) conf))
          (if axis-conf
              (push (list :actuator-axis axis-conf) conf))
          (setf (gethash :assemble-config table) (nreverse conf)))
        ))
    (switch-leaf-upper-parts-to-bottom-parts table)
    ))

(defun switch-leaf-upper-parts-to-bottom-parts (table)
  (let ((up (list :upper-arm20 :upper-arm26 :upper-arm38 :cross-upper-arm28))
        pts)
    (dolist (v (send table :list-values))
      (let ((tp (cadr (assoc :parts-type v)))
            (nm (cadr (assoc :parts-name v))))
        (when (and (memq tp up)
                   (is-leaf? nm table))
          (let ((new-v (remove (assoc :parts-type v) v)))
            (push (list :parts-type
                        (upper-to-bottom tp)) new-v)
            (setf (gethash nm table) new-v)
            ))))
    table
    ))

(defun is-leaf? (key table)
  (let ((res t))
    (dolist (v (send table :list-values))
      (when (eq (cadr (assoc :parent v)) key)
        (setq res nil)
        (return)))
    res))

(defun dump-hash (fname table)
  (let ((hfunc (table . lisp::hash-function))
        (tfunc (table . lisp::test-function)))
    (setq (table . lisp::hash-function) nil)
    (setq (table . lisp::test-function) nil)
    (dump-structure fname table)
    (setq (table . lisp::hash-function) hfunc)
    (setq (table . lisp::test-function) tfunc)
    table))
(defun load-hash (fname &key (test #'eq) (hash #'sxhash))
  (let ((table (with-open-file (f fname) (read f))))
    (setq (table . lisp::hash-function) hash)
    (setq (table . lisp::test-function) test)
    table))
;;;;
;;;; code for robot_assembler
;;;;

(defun create-robot-assembler-parts-from-name (name)
  (if (stringp name)
      (setq name (intern (string-upcase name) *keyword-package*)))
  (let ((fc (gethash name *robot-assembler-functions-htable*)))
    (if fc (funcall fc))))
;;

(defun attach-parts-by-point (a-point b-point &optional (config))
  (let (lst match)
    (dolist (a-p-t (roboasm-fixed-point-types a-point))
      (dolist (b-p-t (roboasm-fixed-point-types b-point))
        (let ((fmatch (gethash (list a-p-t b-p-t) *fixed-point-match-htable*)))
          (push fmatch lst))))
    (setq lst (nreverse lst))
    (setq lst (flatten (mapcar #'(lambda (x) (fixed-point-match-allowed-configuration x)) lst)))
    (cond
     (config
      (setq match (find-if #'(lambda (fm) (eq config (send fm :name))) lst))
      )
     (t
      (setq match (car lst)))
     )
    (when match
      ;;
      (let ((target-point (send a-point :worldcoords))
            (to-parts
             (send (send (send b-point :parent) :worldcoords) :transformation
                   (send (send b-point :copy-worldcoords) :transform match)))
            cds)
        (send (send b-point :parent) :newcoords (send to-parts :transformation target-point :world))
        )
      t
      )
    ))

(defun search-parts-fixed-candidates (a-parts b-parts)
  (let ((pair))
    (dolist (a-p (send a-parts :fixed-points))
      (dolist (b-p (send b-parts :fixed-points))
        (dolist (a-p-t (roboasm-fixed-point-types a-p))
          (dolist (b-p-t (roboasm-fixed-point-types b-p))
            (let ((fmatch (gethash (list a-p-t b-p-t) *fixed-point-match-htable*)))
              (if fmatch (push (list a-p b-p fmatch) pair)))))))
    pair))

(defun search-parts-fixed-list (a-parts b-parts)
  (let (lst)
    (dolist (a-p (send a-parts :fixed-points))
      (send a-parts :reset-coords)
      (send a-parts :worldcoords)
      (dolist (b-p (send b-parts :fixed-points))
        (send b-parts :reset-coords)
        (send b-parts :worldcoords)
        (dolist (a-p-t (roboasm-fixed-point-types a-p))
          (dolist (b-p-t (roboasm-fixed-point-types b-p))
            (let ((fmatch (gethash (list a-p-t b-p-t) *fixed-point-match-htable*)))
              (when fmatch
                (dolist (cc (fixed-point-match-allowed-configuration fmatch))
                  (let ((target-point (send a-p :worldcoords))
                        (to-parts
                         (send (send b-parts :worldcoords) :transformation
                               (send (send b-p :copy-worldcoords) :transform cc)))
                        cds)
                    (send b-parts :newcoords (send to-parts :transformation target-point :world))
                    (setq cds (send a-parts :transformation b-parts))
                    (send b-parts :reset-coords)
                    (send b-parts :worldcoords)
                    ;;(unless (eq a-p-t (car (fixed-point-match-pair fmatch)))
                    ;;(setq cds (send cds :inverse-transformation)))
                    (push (list cds
                                (send a-p :name)
                                (send b-p :name)
                                (send cc :name)) lst)
                    )))
              )))))
    lst))

(defun possible-parts-combination-list (parts-list)
  (let (res-lst)
    (dolist (cb (combination parts-list 2))
      (let ((parts-a (create-robot-assembler-parts-from-name  (car cb)))
            (parts-b (create-robot-assembler-parts-from-name (cadr cb)))
            )
        (when (and parts-a parts-b)
          (let (tmp-lst)
            (dolist (res (search-parts-fixed-list parts-a parts-b))
              (when res
                (push res tmp-lst)))
            (if tmp-lst
                (push (list (send parts-a :name)
                            (send parts-b :name)
                            tmp-lst) res-lst))
            )
          (let (tmp-lst)
            (dolist (res (search-parts-fixed-list parts-b parts-a))
              (when res
                (push res tmp-lst)))
            (if tmp-lst
                (push (list (send parts-b :name)
                            (send parts-a :name)
                            tmp-lst) res-lst))
            )
          )
        ))
    (dolist (p parts-list)
      (let ((parts-a (create-robot-assembler-parts-from-name p))
            (parts-b (create-robot-assembler-parts-from-name p))
            )
        (when parts-a
          (let (tmp-lst)
            (dolist (res (search-parts-fixed-list parts-a parts-b))
              (when res
                (push res tmp-lst)))
            (if tmp-lst
                (push (list (send parts-a :name)
                            (send parts-b :name)
                            tmp-lst) res-lst))
            ))))
    res-lst))

;; for making "/tmp/ra_parts_attach_list.l"
;; (load "package://robot_assembler/euslisp/robot-assembler-viewer.l")
;; (load "package://robot_assembler/euslisp/convert-rcb4robots.l")
;; (make-parts-association-file)
(defun make-parts-association-file
    (&optional (fname "/tmp/ra_parts_attach_list.l"))
  ;;
  (require :eus-assimp "package://eus_assimp/euslisp/eus-assimp.l")
  ;;
  (robot-assembler-initialize
   (ros::resolve-ros-path  "package://robot_assembler/config/robot_assembler_kxr_settings.yaml")
   :project-dir (ros::resolve-ros-path  "package://robot_assembler"))
  ;;
  ;; add name to robot_assembler
  (setf (gethash :upper-arm28 *robot-assembler-functions-htable*)
        (gethash :cross-upper-arm28 *robot-assembler-functions-htable*))
  (setf (gethash :bottom-arm28 *robot-assembler-functions-htable*)
        (gethash :cross-bottom-arm28 *robot-assembler-functions-htable*))
  ;;
  (setq *parts-list* '(:ducted-fan-body :ducted-fan-body-mirror :ducted-fan-motor :duct-mount :main-shaft :dyp-chest-shoulder-arm :dyp-chest-side-plate :small-back-pack :dyp-chest-base-plate :dual-servo-plate2 :flat-arm4242 :dyp-base :sensor-joint-base :sensor-sole :dual-servo-plate5 :m5stickv-u+ :m5stickv-u :m5stickv-g+ :m5stickv-w :m5stickv-c :m5stickv-b+ :m5stickv-b :m5stackfire-pb :m5stackfire-b3 :m5stackfire-b2 :m5stackfire-b1 :m5stackfire-bb :m5stackfire-bf- :m5stackfire-bf :dummy-half-servo :battery-leg-plate :body-plate-spacer :bottom-spacer2 :sole :angle-bracket :cross-servo-frame-b :cross-servo-frame-a :bottom-arm28 :upper-arm28 :arm-supporter-a :body-cut-plate :head :bottom-arm26 :upper-arm26 :joint-frame-a :finger-sensor-palm :finger-b :finger-a :finger-palm :joint-frame-b-b :joint-frame-b-a :flat-frame :upper-arm38 :bottom-arm38 :wheel :bottom-arm20 :upper-arm20 :arm-supporter-b :bottom-spacer :joint-base :s3301 :lock-ring-cap :lock-ring :back-pack :body-plate :battery-box :euclid-head))
  ;;
  (let ((ret (possible-parts-combination-list *parts-list*)))
    (dump-structure fname ret))
  )

#|
(load "package://robot_assembler/euslisp/convert-rcb4robots.l")
(load-parts-association-file "/tmp/ra_parts_attach_list.l")
;;;; at rcb4eus
(setq tbl-lst nil)
(dolist (nm
         (list "kxrl2l5a3h2"
          ))
  (pprint (list ':start-robot nm))
  (setq *robot* (rename-and-add-bbox (kxr-make-robot nm)))
  (objects (list *robot*))
  ;;(do-until-key (x::window-main-one))
  (setq ht (convert-robot-to-roboasm *robot*))
  (push ht tbl-lst)
  (let* ((as (gethash :assemble-config ht))
         (nm (cadr (assoc :robot-name as))))
    (if nm (dump-hash (format nil "/tmp/~A.roboasm.htable" nm) ht)))
  )
|#

#| ;; at robot_assembler
(load "package://robot_assembler/euslisp/robot-assembler-viewer.l")
(load "package://robot_assembler/euslisp/convert-rcb4robots.l")
(make-parts-association-file)
;;
(dolist (nm (list "kxrl2l5a3h2"

                  ))
  (let ((ht (load-hash (format nil "/tmp/~A.roboasm.htable" nm)))
        roboasm rbt)
    (setq roboasm (create-roboasm-from-parsed-table ht))
    (with-open-file
     (f (format nil "/tmp/~A.roboasm.l" nm) :direction :io)
     (pprint roboasm f))
    (setq rbt (make-robot-from-roboasm roboasm))
    (objects (list rbt))
    (send rbt :write-urdf (format nil "/tmp/~A.urdf" nm) :robotname nm)
    (do-until-key (x::window-main-one))
    ))
|#
