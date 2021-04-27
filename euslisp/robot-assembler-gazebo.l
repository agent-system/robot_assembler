(defun gazebo-xyz-rpy (in-coords)
  (let ((pos (scale 0.001 (send in-coords :worldpos)))
        (rpy (car (send in-coords :rpy-angle))))
    (cons
     (format nil "~12,12F ~12,12F ~12,12F" (elt pos 0) (elt pos 1) (elt pos 2))
     (format nil "~12,12F ~12,12F ~12,12F" (elt rpy 2) (elt rpy 1) (elt rpy 0)))
    ))
(defun gazebo-control
    (strm &key (offset ""))
  (format strm "~A<!-- Gazebo plugin for ROS Control -->~%" offset)
  (format strm "~A<gazebo>~%" offset)
  (format strm "~A  <plugin name=\"gazebo_ros_control\" filename=\"libgazebo_ros_control.so\">~%" offset)
  (format strm "~A    <!--robotNamespace>/</robotNamespace-->~%" offset)
  (format strm "~A    <legacyModeNS>false</legacyModeNS>~%" offset)
  (format strm "~A    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>~%" offset)
  (format strm "~A  </plugin>~%" offset)
  (format strm "~A</gazebo>~%" offset)
  )
(defun gazebo-imu-sensor
    (strm &key (offset "") (name) (parent-link) (xyz) (rpy) (rate 100) (topic))
  (let (pose)
    (if (and xyz rpy)
        (setq pose (format nil "~A ~A" xyz rpy))
      (if xyz
          (setq pose (format nil "~A   0 0 0" xyz))
        (if rpy
            (setq pose (format nil "0 0 0   ~A" rpy)))))
    (format strm "~A<gazebo reference=\"~A\">~%" offset parent-link)
    (format strm "~A  <sensor name=\"~A\" type=\"imu\">~%" offset name)
    ;;(format strm "~A    <gravity>true</gravity>~%" offset)
    (format strm "~A    <always_on>true</always_on>~%" offset)
    (format strm "~A    <update_rate>~A</update_rate>~%" offset rate)
    (format strm "~A    <!-- visualize>true</visualize -->~%" offset)
    (format strm "~A    <topic>~A</topic>~%" offset topic)
    (if pose (format strm "~A    <pose>~A</pose>~%" offset pose))
    (format strm "~A    <plugin filename=\"libgazebo_ros_imu_sensor.so\" name=\"imu_plugin\">~%" offset)
    (format strm "~A      <topicName>~A</topicName>~%" offset topic)
    (format strm "~A      <bodyName>~A</bodyName>~%" offset parent-link)
    (format strm "~A      <updateRateHZ>~A</updateRateHZ>~%" offset rate)
    (format strm "~A      <gaussianNoise>0.0</gaussianNoise>~%" offset)
    (if xyz
        (format strm "~A      <xyzOffset>~A</xyzOffset>~%" offset xyz))
    (if rpy
        (format strm "~A      <rpyOffset>~A</rpyOffset>~%" offset rpy))
    (format strm "~A      <frameName>~A</frameName>~%" offset parent-link)
    (format strm "~A      <initialOrientationAsReference>false</initialOrientationAsReference>~%" offset)
    (format strm "~A    </plugin>~%" offset)
    (format strm "~A  </sensor>~%" offset)
    (format strm "~A</gazebo>~%" offset)
    ))
(defun gazebo-touch-sensor ;; f3d
    (strm &key (offset "") (name) (parent-link) (rate 100) (topic))
  (format strm "~A<gazebo>~%" offset )
  (format strm "~A  <plugin name=\"~A_froce_sensor\" filename=\"libgazebo_ros_f3d.so\">~%" offset name)
  (format strm "~A    <alwaysOn>true</alwaysOn>~%" offset )
  (format strm "~A    <updateRate>~A</updateRate>~%" offset rate)
  (format strm "~A    <bodyName>~A</bodyName>~%" offset parent-link)
  (format strm "~A    <topicName>~A</topicName>~%" offset topic)
  (format strm "~A  </plugin>~%" offset )
  (format strm "~A</gazebo>~%" offset )
  )
(defun gazebo-range-sensor
    (strm &key (offset "") (name) (parent-link) (frame) (xyz) (rpy) (rate 25) (topic)
          (samples 720) (resolution 1) (min-angle (deg2rad -90)) (max-angle (deg2rad 90))
          (min-range 0.1) (max-range 30)
          )
  (if (null topic)
      (setq topic (format nil "~A_range_sensor" name)))
  (when frame
    (format strm "~A<link name=\"~A\" />~%" offset frame)
    (format strm "~A<joint name=\"~A_fixed_joint\" type=\"fixed\">~%" offset frame)
    (format strm "~A  <parent link=\"~A\"/>~%" offset parent-link)
    (format strm "~A  <child link=\"~A\"/>~%" offset frame)
    (if (and xyz rpy)
        (format strm "~A  <origin xyz=\"~A\" rpy=\"~A\"/>~%" offset xyz rpy)
      (if xyz
          (format strm "~A  <origin xyz=\"~A\"/>~%" offset xyz)
        (if rpy
            (format strm "~A  <origin rpy=\"~A\"/>~%" offset rpy))))
    (format strm "~A</joint>~%" offset))
  (if (null frame) (setq frame parent-link))
  (format strm "~A<gazebo reference=\"~A\">~%" offset frame)
  (format strm "~A  <sensor type=\"ray\" name=\"~A\">~%" offset name)
  (format strm "~A    <!-- visualize>true</visualize -->~%" offset)
  (format strm "~A    <update_rate>~A</update_rate>~%" offset rate)
  (format strm "~A    <ray>~%" offset)
  (format strm "~A      <scan>~%" offset)
  (format strm "~A        <horizontal>~%" offset)
  (format strm "~A          <samples>~A</samples>~%" offset samples)
  (format strm "~A          <resolution>~A</resolution>~%" offset resolution)
  (format strm "~A          <min_angle>~12,12F</min_angle>~%" offset min-angle)
  (format strm "~A          <max_angle>~12,12F</max_angle>~%" offset max-angle)
  (format strm "~A        </horizontal>~%" offset)
  (format strm "~A      </scan>~%" offset)
  (format strm "~A      <range>~%" offset)
  (format strm "~A        <min>~A</min>~%" offset min-range)
  (format strm "~A        <max>~A</max>~%" offset max-range)
  (format strm "~A        <resolution>0.01</resolution>~%" offset)
  (format strm "~A      </range>~%" offset)
  ;;(format strm "~A      <noise>~%" offset)
  ;;(format strm "~A        <type>gaussian</type>~%" offset)
  ;;(format strm "~A        <!-- Noise parameters based on published spec for Hokuyo laser~%" offset)
  ;;(format strm "~A             achieving \"+-30mm\" accuracy at range < 10m.  A mean of 0.0m and~%" offset)
  ;;(format strm "~A             stddev of 0.01m will put 99.7% of samples within 0.03m of the true~%" offset)
  ;;(format strm "~A             reading. -->~%" offset)
  ;;(format strm "~A        <mean>0.0</mean>~%" offset)
  ;;(format strm "~A        <stddev>0.01</stddev>~%" offset)
  ;;(format strm "~A      </noise>~%" offset)
  (format strm "~A    </ray>~%" offset)
  (format strm "~A    <plugin name=\"gazebo_ros_head_hokuyo_controller\" filename=\"libgazebo_ros_laser.so\">~%" offset)
  (format strm "~A      <topicName>~A</topicName>~%" offset topic)
  (format strm "~A      <frameName>~A</frameName>~%" offset frame)
  (format strm "~A    </plugin>~%" offset)
  (format strm "~A  </sensor>~%" offset)
  (format strm "~A</gazebo>~%" offset)
  )
(defun gazebo-camera
    (strm &key (offset "") (camera-coords) (name) (parent-link) (rate 25) (topic "camera")
          (fov 1.0) (width 800) (height 600) (near-clip 0.02) (far-clip 300))
  ;;
  (let (opt_xyz opt_rpy cam-pose)
    (setq cam-pose (gazebo-xyz-rpy camera-coords))
    (setq cam-pose (format nil "~A ~A" (car cam-pose) (cdr cam-pose)))
    (let ((opt (send camera-coords :copy-worldcoords)))
      (send opt :rotate pi/2 :y)
      (setq opt (gazebo-xyz-rpy opt))
      (setq opt_xyz (car opt)
            opt_rpy (cdr opt)))
    (format strm "~A<link name=\"~A_optical\" />~%" offset topic)
    (format strm "~A<joint name=\"~A_fixed_joint\" type=\"fixed\">~%" offset topic)
    (format strm "~A  <parent link=\"~A\"/>~%" offset parent-link)
    (format strm "~A  <child link=\"~A_optical\"/>~%" offset topic)
    (format strm "~A  <origin xyz=\"~A\" rpy=\"~A\"/>~%" offset opt_xyz opt_rpy)
    (format strm "~A</joint>~%" offset)
    ;;
    (format strm "~A<gazebo reference=\"~A\">~%" offset parent-link)
    (format strm "~A  <sensor name=\"~A_camera\" type=\"camera\" >~%" offset name)
    (format strm "~A    <pose>~A</pose>~%" offset cam-pose)
    (format strm "~A    <always_on>true</always_on>~%" offset )
    (format strm "~A    <update_rate>~A</update_rate>~%" offset rate)
    (format strm "~A    <!-- visualize>true</visualize -->~%" offset )
    (format strm "~A    <camera name=\"~A_camera\">~%" offset name)
    (format strm "~A      <horizontal_fov>~A</horizontal_fov>~%" offset fov)
    (format strm "~A      <image>~%" offset )
    (format strm "~A        <width>~A</width>~%" offset width)
    (format strm "~A        <height>~A</height>~%" offset height)
    (format strm "~A        <format>B8G8R8</format>~%" offset )
    (format strm "~A      </image>~%" offset )
    (format strm "~A      <clip>~%" offset )
    (format strm "~A        <near>~A</near>~%" offset near-clip)
    (format strm "~A        <far>~A</far>~%" offset far-clip)
    (format strm "~A      </clip>~%" offset )
    (format strm "~A      <noise>~%" offset )
    (format strm "~A        <type>gaussian</type>~%" offset )
    (format strm "~A        <!-- Noise is sampled independently per pixel on each frame.~%" offset )
    (format strm "~A             That pixel's noise value is added to each of its color~%" offset )
    (format strm "~A             channels, which at that point lie in the range [0,1]. -->~%" offset )
    (format strm "~A        <mean>0.0</mean>~%" offset )
    (format strm "~A        <stddev>0.007</stddev>~%" offset )
    (format strm "~A      </noise>~%" offset )
    (format strm "~A    </camera>~%" offset )
    (format strm "~A    <plugin name=\"camera_controller\" filename=\"libgazebo_ros_camera.so\">~%" offset )
    (format strm "~A      <alwaysOn>true</alwaysOn>~%" offset )
    (format strm "~A      <updateRate>0.0</updateRate><!-- ? -->~%" offset )
    (format strm "~A      <cameraName>~A</cameraName>~%" offset topic)
    (format strm "~A      <imageTopicName>image_raw</imageTopicName>~%" offset )
    (format strm "~A      <cameraInfoTopicName>camera_info</cameraInfoTopicName>~%" offset )
    (format strm "~A      <frameName>~A_optical</frameName>~%" offset topic)
    (format strm "~A      <hackBaseline>0.0</hackBaseline>~%" offset )
    (format strm "~A      <distortionK1>0.0</distortionK1>~%" offset )
    (format strm "~A      <distortionK2>0.0</distortionK2>~%" offset )
    (format strm "~A      <distortionK3>0.0</distortionK3>~%" offset )
    (format strm "~A      <distortionT1>0.0</distortionT1>~%" offset )
    (format strm "~A      <distortionT2>0.0</distortionT2>~%" offset )
    (format strm "~A    </plugin>~%" offset )
    (format strm "~A  </sensor>~%" offset )
    (format strm "~A</gazebo>~%" offset )
    ))