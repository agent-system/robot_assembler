(require :robot-assembler-utils "robot-assembler-utils.l")
(require :eus-assimp "package://eus_assimp/euslisp/eus-assimp.l")
(require :eus-yaml "package://robot_assembler/euslisp/eus_yaml_cpp.so") ;; loading c-parse-yaml-file

(defvar *robot-assembler-debug* nil)

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
(defun pair-test (x y)
  (if (or (atom x) (atom y))
      nil
    (or (and (eq (car x) (car y)) (eq (cadr x) (cadr y)))
        (and (eq (car x) (cadr y)) (eq (car x) (cadr y))))))
(defun pair-sxhash (x)
  (if (string<= (string (car x)) (string (cadr x)))
      (sxhash x)
    (sxhash (list (cadr x) (car x)))))
(defclass roboasm-coords
  :super bodyset
  )
(defmethod roboasm-coords
  (:dissoc-parent
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
  (:to-next-link
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
  )
(defclass roboasm-fixed-point
  :super roboasm-coords
  :slots (types ;; in *fixed-point-type-list*
          configuration
          type-geometry-list
          )
  )
(defclass roboasm-parts
  :super roboasm-coords
  :slots (type
          fixed-points ;; list of roboasm-fixed-point
          mass
          center-of-mass
          inertia-tensor
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
  (:active-fixed-points ()
   (remove-if #'(lambda (x) (send x :descendants)) fixed-points))
  (:all-parts (&rest args) fixed-points)
  (:fixed-points (&optional (type :free)) fixed-points)
  )
(defclass roboasm-actuator
  :super roboasm-parts
  :slots (horns ;; list of roboasm-fixed-point
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
  (:all-parts (&rest args) (append horns fixed-points))
  (:active-fixed-points ()
   (remove-if #'(lambda (x) (send x :descendants)) (append horns fixed-points)))
  (:horns (&optional (type :all)) horns)
  )

(defclass roboasm-robot
  :super roboasm-coords
  )
(defmethod roboasm-robot
  ;;
  (:attach
   (dx-obj &rest args &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   (when (send dx-obj :parent)
     (warn ";; :attach / ~A already attached~%" dx-obj)
     (return-from :attach nil))
   (cond
    ((derivedp dx-obj roboasm-robot)
     (send* self :attach-robot dx-obj args));;TODO:
    ((derivedp dx-obj roboasm-parts)
     (send* self :attach-parts dx-obj args))
    (t
     ;; err
     nil
     ))
   )
  (:attach-parts
   (aparts &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   (unless (derivedp robot-point roboasm-fixed-point)
     (warn ";;mismatch(:attach-parts) :robot-point(~A) is not fixed-point~%"
           robot-point)
     (return-from :attach-parts nil)
     )
   (unless (derivedp parts-point roboasm-fixed-point)
     (warn ";;mismatch(:attach-parts) :parts-point(~A) is not fixed-point~%"
           parts-point)
     (return-from :attach-parts nil)
     )
   (unless (eq aparts (send parts-point :parent))  ;; parts-point jsut child of parts
     (warn ";;mismatch(:attach-parts) :parts-point(~A) is not child of aparts~%"
           parts-point aparts)
     (return-from :attach-parts nil)
     )
   (unless (memq robot-point (send self :all-parts roboasm-coords)) ;; robot-point is descendants of self
     (warn ";;mismatch(:attach-parts) :robot-point(~A) is not descendants of ~A~%"
           robot-point self)
     (return-from :attach-parts nil)
     )
   (when (remove-if #'(lambda (x) (derivedp x roboasm-coords)) (send robot-point :descendants))
     (warn ";;mismatch(:attach-parts) :robot-point(~A) is already used for (~A)~%"
           robot-point (remove-if #'(lambda (x) (derivedp x roboasm-coords)) (send robot-point :descendants)))
     (return-from :attach-parts nil)
     )
   ;;
   (let ((ret
          (search-match-candidates (parts-point . types) (robot-point . types))))

     (unless ret
       (warn ";; type-mismatch(:attach-parts) :robot-point(~A) and :parts-point(~A)~% is not a pair~%"
             robot-point parts-point)
       (return-from :attach-parts nil))
     (setq ret (reverse ret))

     (cond
      ((null attach-configuration)
       (setq ret-attach
             (car ((car ret) . allowed-configuration))) ;; use default-one
       ;; warning
       )
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

     (unless ret-attach
       (warn ";; type-mismatch(:attach-parts) :robot-point(~A ~A) and :parts-point(~A ~A)~% does not has allowd-configuration(~A)~%"
             robot-point rp-t-lst
             parts-point pp-t-lst attach-configuration)
       (return-from :attach-parts nil))
     ;;
     (when *robot-assembler-debug*
       (warn ";; attach robot: ~A, robot-point: ~A, "
             self robot-point)
       (warn "parts: ~A, parts-point: ~A, :pair ~A, :attach-config ~A, "
             aparts parts-point (car ret) ret-attach)
       (warn "pair-candidates: ~A~%"
             ret))
     (setq attach-configuration ret-attach)
     )
   ;;move aparts
   ;;parts-point == robot-point
   (let ((target-point (send robot-point :worldcoords))
         (to-parts
          (send (send aparts :worldcoords) :transformation
                (send (send parts-point :copy-worldcoords) :transform attach-configuration)
                ))
         )
     ;; target-point may be changed
     (send aparts :newcoords (send to-parts
                                   :transformation target-point :world))
     ;;(send aparts :transform attach-configuration)
     )
   (unless just-align
     ;; swap parent-children
     (send aparts :dissoc parts-point)
     (send parts-point :assoc aparts)
     ;;
     (send robot-point :assoc parts-point)
     )
   self
   )
  (:attach-robot ;; do not use attached robot after this method
   (obj &key (parts-point) (robot-point) (attach-configuration) (just-align) &allow-other-keys)
   ;; do not use yet

   ;;move obj
   ;;parts-point == robot-point
   (let ((target-point (send robot-point :worldcoords)))
     ;; target-point may be changed
     (send obj :newcoords (send (send obj :transformation (send parts-point :worldcoords))
                                :transformation target-point :world))
     )

   (unless just-align
     ;; invert child-parent
     (let* ((r-lst (send parts-point :to-root-list))
            (prev (car r-lst)))
       (dolist (cur (cdr r-lst))
         (send cur :dissoc prev)
         (setq prev cur))
       ;;
       (setq r-lst (cdr (reverse r-lst)))
       (setq prev (car r-lst))
       (dolist (cur (cdr car-lst))
         (send prev :assoc cur))
       )
     ;; assoc
     (send robot-point :assoc parts-point)
     )
   )
  ;;
  (:horns (&optional (type :all))
   (let ((obj-lst (send self :all-descendants roboasm-actuator))
         ret)
     (setq ret (mapcan #'(lambda (obj)
                           (remove-if #'(lambda (x) (send x :descendants)) (obj . horns))) obj-lst))
     ))
  (:fixed-points (&optional (type :free))
   (let ((obj-lst (send self :all-descendants roboasm-parts))
         ret)
     (setq ret (mapcan #'(lambda (obj)
                           (remove-if #'(lambda (x) (send x :descendants)) (obj . fixed-points))) obj-lst))
     ))
  (:active-fixed-points ()
   (let ((obj-lst (send self :all-descendants roboasm-parts))
         ret)
     (setq ret (mapcan #'(lambda (obj) (send obj :active-fixed-points)) obj-lst))
     ret
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
              (col   (send bd :get :collision-geom))
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
           ;; parse collision / not implemented
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
       (format strm "~A<inertial>~%" offset)
       (format strm "~A   <mass value=\"~A\"/>~%" offset (* 0.001 t-mass))
       (format strm "~A   <origin rpy=\"0 0 0\" xyz=\"~A ~A ~A\"/>~%" offset
               (elt n-cent 0) (elt n-cent 1) (elt n-cent 2))
       (format strm "~A   <inertia ixx=\"~A\" ixy=\"~A\" ixz=\"~A\" iyy=\"~A\" iyz=\"~A\" izz=\"~A\"/>~%" offset
               (aref n-iner 0 0) (aref n-iner 1 0) (aref n-iner 2 0)
               (aref n-iner 1 1) (aref n-iner 1 2)
               (aref n-iner 2 2)
               )
       (format strm "~A</inertial>~%" offset)
       )
     ;;;
     (send self :_write-urdf-geometry strm parts-list origin :offset offset :package-prefix package-prefix)
     (format strm "~A</link>~%" offset)
     ))
  (:_write-urdf-joint ;;
   (strm parent-name child-name coords &key (name) (limit) (offset ""))
   (format strm "~A<xacro:gazebo_transmission joint_name=\"~A\" />~%" offset name)
   (format strm "~A<joint name=\"~A\" type=\"revolute\">~%" offset name);; uniq name
   (format strm "~A   <parent link=\"~A\"/>~%" offset parent-name)
   (format strm "~A   <child link=\"~A\"/>~%"  offset child-name)
   (send self :_write-urdf-coords strm coords (format nil "~A   " offset))
   ;;(format strm "   <origin rpy=\"0 -0 0\" xyz=\"0 0.09 0\"/>~%" offset)
   (format strm "~A   <axis xyz=\"0 0 1\"/>~%" offset) ;; horn may turn arout z-axis
   (format strm "~A   <limit effort=\"200\" lower=\"-3.14159\" upper=\"3.14159\" velocity=\"40\"/>~%" offset)
   (format strm "~A</joint>~%" offset)
   )
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
   (let ((start-coords-list
          (mapcan #'(lambda (x) (remove-if-not #'(lambda (x) (send x :descendants)) (send x :horns)))
                  (send self :all-descendants roboasm-actuator)))
         root-link
         link-list-list
         parent-list
         joint-names
         )
     ;; start-coords-list (list of link-origin) ;; should be horn(joint)
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
      (strm fname :direction :output)
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
      (format strm "      </joint>~%")
      (format strm "      <actuator name=\"${joint_name}_motor\">~%")
      (format strm "        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>~%")
      (format strm "        <mechanicalReduction>1</mechanicalReduction>~%")
      (format strm "      </actuator>~%")
      (format strm "    </transmission>~%")
      (format strm "  </xacro:macro>~%~%")
      (let ((cntr 0))
        ;; write root link
        (send self :_write-urdf-link strm root-link (send self :copy-worldcoords)
              :name "BASE" :package-prefix package-prefix :offset "  ")
        ;; write links
        (mapcar #'(lambda (link-list start-coords)
                    (send self :_write-urdf-link strm link-list
                          (send start-coords :copy-worldcoords)
                          :name (format nil "LINK~D" cntr)
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
                      (send self :_write-urdf-joint strm
                            (case parent-pos
                              (:root-link "BASE")
                              (t (format nil "LINK~D" parent-pos)))
                            (format nil "LINK~D" cntr)
                            p-c-coords
                            :name (format nil "JOINT~D" cntr)
                            :offset "  ")
                      (incf cntr)))
                parent-list start-coords-list)
        (dotimes (i cntr)
          (push (format nil "JOINT~D" i) joint-names))
        (setq joint-names (reverse joint-names))
        ;;
        )
      (format strm "~%  <!-- Gazebo plugin for ROS Control -->~%")
      (format strm "  <gazebo>~%")
      (format strm "    <plugin name=\"gazebo_ros_control\" filename=\"libgazebo_ros_control.so\">~%")
      (format strm "      <!--robotNamespace>/</robotNamespace-->~%")
      (format strm "      <legacyModeNS>false</legacyModeNS>~%")
      (format strm "      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>~%")
      (format strm "    </plugin>~%")
      (format strm "  </gazebo>~%")
      (format strm "</robot>~%")
      );; with-open-file
     (send self :_write-controller-config
           (format nil "~A.gz_controller.yaml" fname)  joint-names)
     (send self :_write-euscollada-yaml
           (format nil "~A.euscollada.yaml" fname)  joint-names)
     t
     ))
  (:write-body ());; choreonoid
  (:write-robot-assembler-yaml ())
  ;;
  (:all-parts (&optional (target-class roboasm-parts))
   (send self :all-descendants target-class))
  (:bodies () ;; for display at irtviewer
   (flatten (send-all (remove self (send self :all-parts roboasm-coords) :count 1) :bodies))
   )
  )

(defun unique-list (lst)
  (let (cur)
    (mapcan #'(lambda (x) (unless (memq x cur) (push x cur) (list x))) lst)))

(defun search-match-candidates (parts-point-types robot-point-types)
  (let ( ret )

    (dolist (pp-t parts-point-types)
      (dolist (rp-t robot-point-types)
        (let ((r (gethash (list pp-t rp-t) *fixed-point-match-htable*)))
          (when r (push r ret)))
        ))

    (unless ret
      (return-from search-match-candidates))

    (setq ret (reverse ret))
    ))

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
    ;;
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
  (let* ((type      (cadr (assoc :type      slst)))
         (geometry  (cadr (assoc :geometry  slst)))
         (massparam (cadr (assoc :mass-param slst)))
         (horns     (cadr (assoc :horns     slst)))
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
          (when (eq t namespace)
            (setq namespace (format nil "~A" (string name))))
          ,(unless parts
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
                )
              (list obj))
            ))
    (mapcan #'(lambda (e) (parse-element e)) slst)
    ))
(defun _parse-fixed-points (slst &key (namespace))
  (labels ((parse-element
            (elm)
            (let ((name  (cadr (assoc :name  elm)))
                  (types (cadr (assoc :types elm)))
                  (trs    (cadr (assoc :translation elm)))
                  (rot-an (cadr (assoc :rotation elm)))
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
                )
              (list obj))
            ))
    (mapcan #'(lambda (e) (parse-element e)) slst)
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

(defun parse-attach-history (a-hist &key (use-current-setting t))
  (unless (cadr (assoc :initial-parts (car a-hist)))
    (warn ";; history do not start-with :initial-parts~%")
    (return-from parse-attachi-history))
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
    (let (arobot)
      (let* ((p-type (cadr (assoc :parts-type (car a-hist))))
             (p-name (cadr (assoc :parts-name (car a-hist))))
             (p (make-parts-from-key p-type p-name)))
        (unless p (return-from parse-attach-history))
        (setq arobot (make-robot-assembler-robot p :name "pased_from_ahist"))
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
          (unless p (return-from parse-attach-history))
          (setq r-p (find-if #'(lambda (x) (eq attach-point (send x :name))) (send arobot :active-fixed-points)))
          (setq p-p (find-if #'(lambda (x) (eq p-point (send x :name))) (send p :active-fixed-points)))
          (unless (and r-p p-p)
            (return-from parse-attach-history))
          (setq ret
                (send arobot :attach p :parts-point p-p :robot-point r-p
                      :attach-configuration (if (eq config :default) nil config)))
          (unless ret
            (return-from parse-attach-history))
          ))
      arobot)
    ))

;; (setq ht (parse-roboasm "filename.roboasm.l"))
;; (create-roboasm-from-parsed-table ht)
;; (setq ht (parse-roboasm "filename.roboasm.l"))
;; (create-roboasm-from-parsed-table (change-root-parts :new-root ht))
(defun parse-roboasm (roboasm)
  (let ((parts-htbl (make-hash-table)))
    (dolist (p roboasm)
      (let* ((pname (cadr (assoc :parts-name p)))
             (rp (cadr (assoc :robot-parts-point p)))
             (str_rp (if rp (string rp)))
             (pos (if rp (position #\/ str_rp)))
             )
        (when pos
          (push (list :parent
                      (intern (subseq str_rp 0 pos) *keyword-package*))
                p)
          (push (list :parents-point rp) p)
          ;;(intern (subseq str_rp (1+ pos)) *keyword-package*)) p)
          )
        (setf (gethash pname parts-htbl) p)
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
        keys cur-parts
        lst)
    ;;(pprint (gethash initial parts-htbl))
    (push (gethash initial parts-htbl) lst)
    (remhash initial parts-htbl)
    (setq keys (send parts-htbl :list-keys))
    (while keys
      (setq cur-parts
            (find-if #'(lambda (k) (null (gethash (cadr (assoc :parent (gethash k parts-htbl))) parts-htbl)))
                     keys))
      (let ((param (gethash cur-parts parts-htbl)))
        (setq param (remove-if #'(lambda (p) (or (eq :parent (car p))
                                                 (eq :parents-point (car p)))) param))
        ;;(pprint param)
        (push param lst)
        )
      (remhash cur-parts parts-htbl)
      (setq keys (send parts-htbl :list-keys))
      )
    (reverse lst)
    ;;
    ))


;; compatibility with body:vertices
(defmethod gl::glvertices
  (:vertices-compatible ()
   (unless gl::bbox
     (send self :calc-bounding-box))
   (mapcar #'(lambda (p) (send self :transform-vector p))
           (send (send gl::bbox :body) :vertices))
   ))
