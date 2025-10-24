; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude RobotCommand.msg.html

(cl:defclass <RobotCommand> (roslisp-msg-protocol:ros-message)
  ((motor_command
    :reader motor_command
    :initarg :motor_command
    :type (cl:vector robot_msgs-msg:MotorCommand)
   :initform (cl:make-array 0 :element-type 'robot_msgs-msg:MotorCommand :initial-element (cl:make-instance 'robot_msgs-msg:MotorCommand))))
)

(cl:defclass RobotCommand (<RobotCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<RobotCommand> is deprecated: use robot_msgs-msg:RobotCommand instead.")))

(cl:ensure-generic-function 'motor_command-val :lambda-list '(m))
(cl:defmethod motor_command-val ((m <RobotCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:motor_command-val is deprecated.  Use robot_msgs-msg:motor_command instead.")
  (motor_command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotCommand>) ostream)
  "Serializes a message object of type '<RobotCommand>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor_command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotCommand>) istream)
  "Deserializes a message object of type '<RobotCommand>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_command) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_command)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robot_msgs-msg:MotorCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotCommand>)))
  "Returns string type for a message object of type '<RobotCommand>"
  "robot_msgs/RobotCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCommand)))
  "Returns string type for a message object of type 'RobotCommand"
  "robot_msgs/RobotCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotCommand>)))
  "Returns md5sum for a message object of type '<RobotCommand>"
  "ef1354637f40ee9d0985c79aafa5f40f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotCommand)))
  "Returns md5sum for a message object of type 'RobotCommand"
  "ef1354637f40ee9d0985c79aafa5f40f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotCommand>)))
  "Returns full string definition for message of type '<RobotCommand>"
  (cl:format cl:nil "MotorCommand[] motor_command~%================================================================================~%MSG: robot_msgs/MotorCommand~%float32 q            # motor target position~%float32 dq           # motor target velocity~%float32 tau          # motor target torque~%float32 kp           # motor spring stiffness coefficient~%float32 kd           # motor damper coefficient~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotCommand)))
  "Returns full string definition for message of type 'RobotCommand"
  (cl:format cl:nil "MotorCommand[] motor_command~%================================================================================~%MSG: robot_msgs/MotorCommand~%float32 q            # motor target position~%float32 dq           # motor target velocity~%float32 tau          # motor target torque~%float32 kp           # motor spring stiffness coefficient~%float32 kd           # motor damper coefficient~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotCommand>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_command) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotCommand
    (cl:cons ':motor_command (motor_command msg))
))
