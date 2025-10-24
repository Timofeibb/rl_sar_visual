;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::RobotState)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'RobotState (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::ROBOTSTATE")
  (make-package "ROBOT_MSGS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html


(defclass robot_msgs::RobotState
  :super ros::object
  :slots (_imu _motor_state ))

(defmethod robot_msgs::RobotState
  (:init
   (&key
    ((:imu __imu) (instance robot_msgs::IMU :init))
    ((:motor_state __motor_state) ())
    )
   (send-super :init)
   (setq _imu __imu)
   (setq _motor_state __motor_state)
   self)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:motor_state
   (&rest __motor_state)
   (if (keywordp (car __motor_state))
       (send* _motor_state __motor_state)
     (progn
       (if __motor_state (setq _motor_state (car __motor_state)))
       _motor_state)))
  (:serialization-length
   ()
   (+
    ;; robot_msgs/IMU _imu
    (send _imu :serialization-length)
    ;; robot_msgs/MotorState[] _motor_state
    (apply #'+ (send-all _motor_state :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_msgs/IMU _imu
       (send _imu :serialize s)
     ;; robot_msgs/MotorState[] _motor_state
     (write-long (length _motor_state) s)
     (dolist (elem _motor_state)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_msgs/IMU _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; robot_msgs/MotorState[] _motor_state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motor_state (let (r) (dotimes (i n) (push (instance robot_msgs::MotorState :init) r)) r))
     (dolist (elem- _motor_state)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robot_msgs::RobotState :md5sum-) "c7d5f0c74618348347456ca207fda81e")
(setf (get robot_msgs::RobotState :datatype-) "robot_msgs/RobotState")
(setf (get robot_msgs::RobotState :definition-)
      "IMU imu
MotorState[] motor_state
================================================================================
MSG: robot_msgs/IMU
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
================================================================================
MSG: robot_msgs/MotorState
float32 q            # motor current position (rad)
float32 dq           # motor current speed (rad/s)
float32 ddq          # motor current speed (rad/s)
float32 tau_est      # current estimated output torque (N*m)
float32 cur          # current estimated output cur (N*m)
")



(provide :robot_msgs/RobotState "c7d5f0c74618348347456ca207fda81e")


