; Auto-generated. Do not edit!


(cl:in-package marsworks_vis_2-msg)


;//! \htmlinclude Tvec.msg.html

(cl:defclass <Tvec> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (z
    :reader z
    :initarg :z
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Tvec (<Tvec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tvec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tvec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name marsworks_vis_2-msg:<Tvec> is deprecated: use marsworks_vis_2-msg:Tvec instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Tvec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marsworks_vis_2-msg:x-val is deprecated.  Use marsworks_vis_2-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Tvec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marsworks_vis_2-msg:y-val is deprecated.  Use marsworks_vis_2-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Tvec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marsworks_vis_2-msg:z-val is deprecated.  Use marsworks_vis_2-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tvec>) ostream)
  "Serializes a message object of type '<Tvec>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tvec>) istream)
  "Deserializes a message object of type '<Tvec>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tvec>)))
  "Returns string type for a message object of type '<Tvec>"
  "marsworks_vis_2/Tvec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tvec)))
  "Returns string type for a message object of type 'Tvec"
  "marsworks_vis_2/Tvec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tvec>)))
  "Returns md5sum for a message object of type '<Tvec>"
  "85729383565f7e059d4a213b3db1317b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tvec)))
  "Returns md5sum for a message object of type 'Tvec"
  "85729383565f7e059d4a213b3db1317b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tvec>)))
  "Returns full string definition for message of type '<Tvec>"
  (cl:format cl:nil "int16 x~%int16 y~%int16 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tvec)))
  "Returns full string definition for message of type 'Tvec"
  (cl:format cl:nil "int16 x~%int16 y~%int16 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tvec>))
  (cl:+ 0
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tvec>))
  "Converts a ROS message object to a list"
  (cl:list 'Tvec
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
