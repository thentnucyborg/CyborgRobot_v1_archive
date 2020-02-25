; Auto-generated. Do not edit!


(cl:in-package speech_processing-msg)


;//! \htmlinclude Expression.msg.html

(cl:defclass <Expression> (roslisp-msg-protocol:ros-message)
  ((speech
    :reader speech
    :initarg :speech
    :type cl:string
    :initform "")
   (expression
    :reader expression
    :initarg :expression
    :type cl:string
    :initform ""))
)

(cl:defclass Expression (<Expression>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Expression>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Expression)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_processing-msg:<Expression> is deprecated: use speech_processing-msg:Expression instead.")))

(cl:ensure-generic-function 'speech-val :lambda-list '(m))
(cl:defmethod speech-val ((m <Expression>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_processing-msg:speech-val is deprecated.  Use speech_processing-msg:speech instead.")
  (speech m))

(cl:ensure-generic-function 'expression-val :lambda-list '(m))
(cl:defmethod expression-val ((m <Expression>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_processing-msg:expression-val is deprecated.  Use speech_processing-msg:expression instead.")
  (expression m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Expression>) ostream)
  "Serializes a message object of type '<Expression>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'speech))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'speech))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'expression))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'expression))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Expression>) istream)
  "Deserializes a message object of type '<Expression>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speech) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'speech) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'expression) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'expression) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Expression>)))
  "Returns string type for a message object of type '<Expression>"
  "speech_processing/Expression")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Expression)))
  "Returns string type for a message object of type 'Expression"
  "speech_processing/Expression")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Expression>)))
  "Returns md5sum for a message object of type '<Expression>"
  "64c25de9407ff13a659a42faaadb6ea1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Expression)))
  "Returns md5sum for a message object of type 'Expression"
  "64c25de9407ff13a659a42faaadb6ea1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Expression>)))
  "Returns full string definition for message of type '<Expression>"
  (cl:format cl:nil "string speech		#The troll will say this out loud. Write normal english.~%string expression 	#shortform for the expression, like angry, smile, etc. ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Expression)))
  "Returns full string definition for message of type 'Expression"
  (cl:format cl:nil "string speech		#The troll will say this out loud. Write normal english.~%string expression 	#shortform for the expression, like angry, smile, etc. ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Expression>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'speech))
     4 (cl:length (cl:slot-value msg 'expression))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Expression>))
  "Converts a ROS message object to a list"
  (cl:list 'Expression
    (cl:cons ':speech (speech msg))
    (cl:cons ':expression (expression msg))
))
