; Auto-generated. Do not edit!


(cl:in-package local_pkg-msg)


;//! \htmlinclude NavPVTWithHeader.msg.html

(cl:defclass <NavPVTWithHeader> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (iTOW
    :reader iTOW
    :initarg :iTOW
    :type cl:integer
    :initform 0)
   (year
    :reader year
    :initarg :year
    :type cl:fixnum
    :initform 0)
   (month
    :reader month
    :initarg :month
    :type cl:fixnum
    :initform 0)
   (day
    :reader day
    :initarg :day
    :type cl:fixnum
    :initform 0)
   (hour
    :reader hour
    :initarg :hour
    :type cl:fixnum
    :initform 0)
   (min
    :reader min
    :initarg :min
    :type cl:fixnum
    :initform 0)
   (sec
    :reader sec
    :initarg :sec
    :type cl:fixnum
    :initform 0)
   (valid
    :reader valid
    :initarg :valid
    :type cl:fixnum
    :initform 0)
   (tAcc
    :reader tAcc
    :initarg :tAcc
    :type cl:integer
    :initform 0)
   (nano
    :reader nano
    :initarg :nano
    :type cl:integer
    :initform 0)
   (fixType
    :reader fixType
    :initarg :fixType
    :type cl:fixnum
    :initform 0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:fixnum
    :initform 0)
   (flags2
    :reader flags2
    :initarg :flags2
    :type cl:fixnum
    :initform 0)
   (numSV
    :reader numSV
    :initarg :numSV
    :type cl:fixnum
    :initform 0)
   (lon
    :reader lon
    :initarg :lon
    :type cl:integer
    :initform 0)
   (lat
    :reader lat
    :initarg :lat
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (hMSL
    :reader hMSL
    :initarg :hMSL
    :type cl:integer
    :initform 0)
   (hAcc
    :reader hAcc
    :initarg :hAcc
    :type cl:integer
    :initform 0)
   (vAcc
    :reader vAcc
    :initarg :vAcc
    :type cl:integer
    :initform 0)
   (velN
    :reader velN
    :initarg :velN
    :type cl:integer
    :initform 0)
   (velE
    :reader velE
    :initarg :velE
    :type cl:integer
    :initform 0)
   (velD
    :reader velD
    :initarg :velD
    :type cl:integer
    :initform 0)
   (gSpeed
    :reader gSpeed
    :initarg :gSpeed
    :type cl:integer
    :initform 0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:integer
    :initform 0)
   (sAcc
    :reader sAcc
    :initarg :sAcc
    :type cl:integer
    :initform 0)
   (headAcc
    :reader headAcc
    :initarg :headAcc
    :type cl:integer
    :initform 0)
   (pDOP
    :reader pDOP
    :initarg :pDOP
    :type cl:fixnum
    :initform 0)
   (reserved1
    :reader reserved1
    :initarg :reserved1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (headVeh
    :reader headVeh
    :initarg :headVeh
    :type cl:integer
    :initform 0)
   (magDec
    :reader magDec
    :initarg :magDec
    :type cl:fixnum
    :initform 0)
   (magAcc
    :reader magAcc
    :initarg :magAcc
    :type cl:fixnum
    :initform 0))
)

(cl:defclass NavPVTWithHeader (<NavPVTWithHeader>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavPVTWithHeader>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavPVTWithHeader)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name local_pkg-msg:<NavPVTWithHeader> is deprecated: use local_pkg-msg:NavPVTWithHeader instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:header-val is deprecated.  Use local_pkg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'iTOW-val :lambda-list '(m))
(cl:defmethod iTOW-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:iTOW-val is deprecated.  Use local_pkg-msg:iTOW instead.")
  (iTOW m))

(cl:ensure-generic-function 'year-val :lambda-list '(m))
(cl:defmethod year-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:year-val is deprecated.  Use local_pkg-msg:year instead.")
  (year m))

(cl:ensure-generic-function 'month-val :lambda-list '(m))
(cl:defmethod month-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:month-val is deprecated.  Use local_pkg-msg:month instead.")
  (month m))

(cl:ensure-generic-function 'day-val :lambda-list '(m))
(cl:defmethod day-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:day-val is deprecated.  Use local_pkg-msg:day instead.")
  (day m))

(cl:ensure-generic-function 'hour-val :lambda-list '(m))
(cl:defmethod hour-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:hour-val is deprecated.  Use local_pkg-msg:hour instead.")
  (hour m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:min-val is deprecated.  Use local_pkg-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'sec-val :lambda-list '(m))
(cl:defmethod sec-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:sec-val is deprecated.  Use local_pkg-msg:sec instead.")
  (sec m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:valid-val is deprecated.  Use local_pkg-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'tAcc-val :lambda-list '(m))
(cl:defmethod tAcc-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:tAcc-val is deprecated.  Use local_pkg-msg:tAcc instead.")
  (tAcc m))

(cl:ensure-generic-function 'nano-val :lambda-list '(m))
(cl:defmethod nano-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:nano-val is deprecated.  Use local_pkg-msg:nano instead.")
  (nano m))

(cl:ensure-generic-function 'fixType-val :lambda-list '(m))
(cl:defmethod fixType-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:fixType-val is deprecated.  Use local_pkg-msg:fixType instead.")
  (fixType m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:flags-val is deprecated.  Use local_pkg-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'flags2-val :lambda-list '(m))
(cl:defmethod flags2-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:flags2-val is deprecated.  Use local_pkg-msg:flags2 instead.")
  (flags2 m))

(cl:ensure-generic-function 'numSV-val :lambda-list '(m))
(cl:defmethod numSV-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:numSV-val is deprecated.  Use local_pkg-msg:numSV instead.")
  (numSV m))

(cl:ensure-generic-function 'lon-val :lambda-list '(m))
(cl:defmethod lon-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:lon-val is deprecated.  Use local_pkg-msg:lon instead.")
  (lon m))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:lat-val is deprecated.  Use local_pkg-msg:lat instead.")
  (lat m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:height-val is deprecated.  Use local_pkg-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'hMSL-val :lambda-list '(m))
(cl:defmethod hMSL-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:hMSL-val is deprecated.  Use local_pkg-msg:hMSL instead.")
  (hMSL m))

(cl:ensure-generic-function 'hAcc-val :lambda-list '(m))
(cl:defmethod hAcc-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:hAcc-val is deprecated.  Use local_pkg-msg:hAcc instead.")
  (hAcc m))

(cl:ensure-generic-function 'vAcc-val :lambda-list '(m))
(cl:defmethod vAcc-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:vAcc-val is deprecated.  Use local_pkg-msg:vAcc instead.")
  (vAcc m))

(cl:ensure-generic-function 'velN-val :lambda-list '(m))
(cl:defmethod velN-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:velN-val is deprecated.  Use local_pkg-msg:velN instead.")
  (velN m))

(cl:ensure-generic-function 'velE-val :lambda-list '(m))
(cl:defmethod velE-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:velE-val is deprecated.  Use local_pkg-msg:velE instead.")
  (velE m))

(cl:ensure-generic-function 'velD-val :lambda-list '(m))
(cl:defmethod velD-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:velD-val is deprecated.  Use local_pkg-msg:velD instead.")
  (velD m))

(cl:ensure-generic-function 'gSpeed-val :lambda-list '(m))
(cl:defmethod gSpeed-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:gSpeed-val is deprecated.  Use local_pkg-msg:gSpeed instead.")
  (gSpeed m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:heading-val is deprecated.  Use local_pkg-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'sAcc-val :lambda-list '(m))
(cl:defmethod sAcc-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:sAcc-val is deprecated.  Use local_pkg-msg:sAcc instead.")
  (sAcc m))

(cl:ensure-generic-function 'headAcc-val :lambda-list '(m))
(cl:defmethod headAcc-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:headAcc-val is deprecated.  Use local_pkg-msg:headAcc instead.")
  (headAcc m))

(cl:ensure-generic-function 'pDOP-val :lambda-list '(m))
(cl:defmethod pDOP-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:pDOP-val is deprecated.  Use local_pkg-msg:pDOP instead.")
  (pDOP m))

(cl:ensure-generic-function 'reserved1-val :lambda-list '(m))
(cl:defmethod reserved1-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:reserved1-val is deprecated.  Use local_pkg-msg:reserved1 instead.")
  (reserved1 m))

(cl:ensure-generic-function 'headVeh-val :lambda-list '(m))
(cl:defmethod headVeh-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:headVeh-val is deprecated.  Use local_pkg-msg:headVeh instead.")
  (headVeh m))

(cl:ensure-generic-function 'magDec-val :lambda-list '(m))
(cl:defmethod magDec-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:magDec-val is deprecated.  Use local_pkg-msg:magDec instead.")
  (magDec m))

(cl:ensure-generic-function 'magAcc-val :lambda-list '(m))
(cl:defmethod magAcc-val ((m <NavPVTWithHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_pkg-msg:magAcc-val is deprecated.  Use local_pkg-msg:magAcc instead.")
  (magAcc m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NavPVTWithHeader>)))
    "Constants for message type '<NavPVTWithHeader>"
  '((:CLASS_ID . 1)
    (:MESSAGE_ID . 7)
    (:VALID_DATE . 1)
    (:VALID_TIME . 2)
    (:VALID_FULLY_RESOLVED . 4)
    (:VALID_MAG . 8)
    (:FIX_TYPE_NO_FIX . 0)
    (:FIX_TYPE_DEAD_RECKONING_ONLY . 1)
    (:FIX_TYPE_2D . 2)
    (:FIX_TYPE_3D . 3)
    (:FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED . 4)
    (:FIX_TYPE_TIME_ONLY . 5)
    (:FLAGS_GNSS_FIX_OK . 1)
    (:FLAGS_DIFF_SOLN . 2)
    (:FLAGS_PSM_MASK . 28)
    (:PSM_OFF . 0)
    (:PSM_ENABLED . 4)
    (:PSM_ACQUIRED . 8)
    (:PSM_TRACKING . 12)
    (:PSM_POWER_OPTIMIZED_TRACKING . 16)
    (:PSM_INACTIVE . 20)
    (:FLAGS_HEAD_VEH_VALID . 32)
    (:FLAGS_CARRIER_PHASE_MASK . 192)
    (:CARRIER_PHASE_NO_SOLUTION . 0)
    (:CARRIER_PHASE_FLOAT . 64)
    (:CARRIER_PHASE_FIXED . 128)
    (:FLAGS2_CONFIRMED_AVAILABLE . 32)
    (:FLAGS2_CONFIRMED_DATE . 64)
    (:FLAGS2_CONFIRMED_TIME . 128))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NavPVTWithHeader)))
    "Constants for message type 'NavPVTWithHeader"
  '((:CLASS_ID . 1)
    (:MESSAGE_ID . 7)
    (:VALID_DATE . 1)
    (:VALID_TIME . 2)
    (:VALID_FULLY_RESOLVED . 4)
    (:VALID_MAG . 8)
    (:FIX_TYPE_NO_FIX . 0)
    (:FIX_TYPE_DEAD_RECKONING_ONLY . 1)
    (:FIX_TYPE_2D . 2)
    (:FIX_TYPE_3D . 3)
    (:FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED . 4)
    (:FIX_TYPE_TIME_ONLY . 5)
    (:FLAGS_GNSS_FIX_OK . 1)
    (:FLAGS_DIFF_SOLN . 2)
    (:FLAGS_PSM_MASK . 28)
    (:PSM_OFF . 0)
    (:PSM_ENABLED . 4)
    (:PSM_ACQUIRED . 8)
    (:PSM_TRACKING . 12)
    (:PSM_POWER_OPTIMIZED_TRACKING . 16)
    (:PSM_INACTIVE . 20)
    (:FLAGS_HEAD_VEH_VALID . 32)
    (:FLAGS_CARRIER_PHASE_MASK . 192)
    (:CARRIER_PHASE_NO_SOLUTION . 0)
    (:CARRIER_PHASE_FLOAT . 64)
    (:CARRIER_PHASE_FIXED . 128)
    (:FLAGS2_CONFIRMED_AVAILABLE . 32)
    (:FLAGS2_CONFIRMED_DATE . 64)
    (:FLAGS2_CONFIRMED_TIME . 128))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavPVTWithHeader>) ostream)
  "Serializes a message object of type '<NavPVTWithHeader>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iTOW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iTOW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'iTOW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'iTOW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'month)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'day)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'valid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tAcc)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'nano)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fixType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numSV)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'lon)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hMSL)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vAcc)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'velN)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velE)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velD)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gSpeed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'heading)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'headAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'headAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'headAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'headAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pDOP)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pDOP)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'reserved1))
  (cl:let* ((signed (cl:slot-value msg 'headVeh)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'magDec)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'magAcc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'magAcc)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavPVTWithHeader>) istream)
  "Deserializes a message object of type '<NavPVTWithHeader>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iTOW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iTOW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'iTOW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'iTOW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'month)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'day)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'valid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tAcc)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nano) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fixType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numSV)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lon) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lat) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hMSL) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vAcc)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velN) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velE) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velD) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gSpeed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'heading) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'headAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'headAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'headAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'headAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pDOP)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pDOP)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'reserved1) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'reserved1)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'headVeh) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'magDec) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'magAcc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'magAcc)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavPVTWithHeader>)))
  "Returns string type for a message object of type '<NavPVTWithHeader>"
  "local_pkg/NavPVTWithHeader")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavPVTWithHeader)))
  "Returns string type for a message object of type 'NavPVTWithHeader"
  "local_pkg/NavPVTWithHeader")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavPVTWithHeader>)))
  "Returns md5sum for a message object of type '<NavPVTWithHeader>"
  "76a709426465468a80d5b09ed58df78d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavPVTWithHeader)))
  "Returns md5sum for a message object of type 'NavPVTWithHeader"
  "76a709426465468a80d5b09ed58df78d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavPVTWithHeader>)))
  "Returns full string definition for message of type '<NavPVTWithHeader>"
  (cl:format cl:nil "# NAV-PVT (0x01 0x07)~%# Navigation Position Velocity Time Solution~%#~%# Note that during a leap second there may be more (or less) than 60 seconds in~%# a minute; see the description of leap seconds for details.~%#~%# This message combines Position, velocity and time solution in LLH, ~%# including accuracy figures~%#~%# WARNING: For firmware version 7, this message is a different length.~%#~%~%std_msgs/Header header~%uint8 CLASS_ID = 1~%uint8 MESSAGE_ID = 7~%~%uint32 iTOW             # GPS Millisecond time of week [ms]~%uint16 year             # Year (UTC)~%uint8 month             # Month, range 1..12 (UTC)~%uint8 day               # Day of month, range 1..31 (UTC)~%uint8 hour              # Hour of day, range 0..23 (UTC)~%uint8 min               # Minute of hour, range 0..59 (UTC)~%uint8 sec               # Seconds of minute, range 0..60 (UTC)~%~%uint8 valid             # Validity flags~%uint8 VALID_DATE = 1            # Valid UTC Date~%uint8 VALID_TIME = 2            # Valid ~%uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved ~%                                # (no seconds uncertainty)~%uint8 VALID_MAG = 8             # Valid Magnetic Declination~%~%uint32 tAcc             # time accuracy estimate [ns] (UTC)~%int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)~%~%uint8 fixType           # GNSS fix Type, range 0..5~%uint8 FIX_TYPE_NO_FIX = 0~%uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1~%uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, ~%                                                # constant altitude assumed~%uint8 FIX_TYPE_3D = 3~%uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning~%uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix (High precision ~%                                                # devices)~%~%uint8 flags             # Fix Status Flags~%uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks~%uint8 FLAGS_DIFF_SOLN = 2            # DGPS used~%uint8 FLAGS_PSM_MASK = 28            # Power Save Mode~%uint8 PSM_OFF = 0                       # PSM is off~%uint8 PSM_ENABLED = 4                   # Enabled (state before acquisition)~%uint8 PSM_ACQUIRED = 8                  # Acquisition~%uint8 PSM_TRACKING = 12                 # Tracking~%uint8 PSM_POWER_OPTIMIZED_TRACKING = 16 # Power Optimized Tracking~%uint8 PSM_INACTIVE = 20                 # Inactive~%uint8 FLAGS_HEAD_VEH_VALID = 32         # heading of vehicle is valid~%uint8 FLAGS_CARRIER_PHASE_MASK = 192 # Carrier Phase Range Solution Status     ~%uint8 CARRIER_PHASE_NO_SOLUTION = 0     # no carrier phase range solution~%uint8 CARRIER_PHASE_FLOAT = 64          # carrier phase float solution (no fixed ~%                                        # integer measurements have been used to ~%                                        # calculate the solution)~%uint8 CARRIER_PHASE_FIXED = 128         # fixed solution (>=1 fixed integer ~%                                        # carrier phase range measurements have ~%                                        # been used to calculate  the solution)~%~%uint8 flags2            # Additional Flags~%uint8 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of ~%                                        # Day validity confirmation is available~%uint8 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed~%uint8 FLAGS2_CONFIRMED_TIME = 128       # UTC Time of Day could be confirmed~%~%uint8 numSV             # Number of SVs used in Nav Solution~%int32 lon               # Longitude [deg / 1e-7]~%int32 lat               # Latitude [deg / 1e-7]~%int32 height            # Height above Ellipsoid [mm]~%int32 hMSL              # Height above mean sea level [mm]~%uint32 hAcc             # Horizontal Accuracy Estimate [mm]~%uint32 vAcc             # Vertical Accuracy Estimate [mm]~%~%int32 velN              # NED north velocity [mm/s]~%int32 velE              # NED east velocity [mm/s]~%int32 velD              # NED down velocity [mm/s]~%int32 gSpeed            # Ground Speed (2-D) [mm/s]~%int32 heading           # Heading of motion 2-D [deg / 1e-5]~%uint32 sAcc             # Speed Accuracy Estimate [mm/s]~%uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) ~%                        # [deg / 1e-5]~%~%uint16 pDOP             # Position DOP [1 / 0.01]~%uint8[6] reserved1      # Reserved~%~%int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]~%int16 magDec            # Magnetic declination [deg / 1e-2]~%uint16 magAcc           # Magnetic declination accuracy [deg / 1e-2]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavPVTWithHeader)))
  "Returns full string definition for message of type 'NavPVTWithHeader"
  (cl:format cl:nil "# NAV-PVT (0x01 0x07)~%# Navigation Position Velocity Time Solution~%#~%# Note that during a leap second there may be more (or less) than 60 seconds in~%# a minute; see the description of leap seconds for details.~%#~%# This message combines Position, velocity and time solution in LLH, ~%# including accuracy figures~%#~%# WARNING: For firmware version 7, this message is a different length.~%#~%~%std_msgs/Header header~%uint8 CLASS_ID = 1~%uint8 MESSAGE_ID = 7~%~%uint32 iTOW             # GPS Millisecond time of week [ms]~%uint16 year             # Year (UTC)~%uint8 month             # Month, range 1..12 (UTC)~%uint8 day               # Day of month, range 1..31 (UTC)~%uint8 hour              # Hour of day, range 0..23 (UTC)~%uint8 min               # Minute of hour, range 0..59 (UTC)~%uint8 sec               # Seconds of minute, range 0..60 (UTC)~%~%uint8 valid             # Validity flags~%uint8 VALID_DATE = 1            # Valid UTC Date~%uint8 VALID_TIME = 2            # Valid ~%uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved ~%                                # (no seconds uncertainty)~%uint8 VALID_MAG = 8             # Valid Magnetic Declination~%~%uint32 tAcc             # time accuracy estimate [ns] (UTC)~%int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)~%~%uint8 fixType           # GNSS fix Type, range 0..5~%uint8 FIX_TYPE_NO_FIX = 0~%uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1~%uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, ~%                                                # constant altitude assumed~%uint8 FIX_TYPE_3D = 3~%uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning~%uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix (High precision ~%                                                # devices)~%~%uint8 flags             # Fix Status Flags~%uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks~%uint8 FLAGS_DIFF_SOLN = 2            # DGPS used~%uint8 FLAGS_PSM_MASK = 28            # Power Save Mode~%uint8 PSM_OFF = 0                       # PSM is off~%uint8 PSM_ENABLED = 4                   # Enabled (state before acquisition)~%uint8 PSM_ACQUIRED = 8                  # Acquisition~%uint8 PSM_TRACKING = 12                 # Tracking~%uint8 PSM_POWER_OPTIMIZED_TRACKING = 16 # Power Optimized Tracking~%uint8 PSM_INACTIVE = 20                 # Inactive~%uint8 FLAGS_HEAD_VEH_VALID = 32         # heading of vehicle is valid~%uint8 FLAGS_CARRIER_PHASE_MASK = 192 # Carrier Phase Range Solution Status     ~%uint8 CARRIER_PHASE_NO_SOLUTION = 0     # no carrier phase range solution~%uint8 CARRIER_PHASE_FLOAT = 64          # carrier phase float solution (no fixed ~%                                        # integer measurements have been used to ~%                                        # calculate the solution)~%uint8 CARRIER_PHASE_FIXED = 128         # fixed solution (>=1 fixed integer ~%                                        # carrier phase range measurements have ~%                                        # been used to calculate  the solution)~%~%uint8 flags2            # Additional Flags~%uint8 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of ~%                                        # Day validity confirmation is available~%uint8 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed~%uint8 FLAGS2_CONFIRMED_TIME = 128       # UTC Time of Day could be confirmed~%~%uint8 numSV             # Number of SVs used in Nav Solution~%int32 lon               # Longitude [deg / 1e-7]~%int32 lat               # Latitude [deg / 1e-7]~%int32 height            # Height above Ellipsoid [mm]~%int32 hMSL              # Height above mean sea level [mm]~%uint32 hAcc             # Horizontal Accuracy Estimate [mm]~%uint32 vAcc             # Vertical Accuracy Estimate [mm]~%~%int32 velN              # NED north velocity [mm/s]~%int32 velE              # NED east velocity [mm/s]~%int32 velD              # NED down velocity [mm/s]~%int32 gSpeed            # Ground Speed (2-D) [mm/s]~%int32 heading           # Heading of motion 2-D [deg / 1e-5]~%uint32 sAcc             # Speed Accuracy Estimate [mm/s]~%uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) ~%                        # [deg / 1e-5]~%~%uint16 pDOP             # Position DOP [1 / 0.01]~%uint8[6] reserved1      # Reserved~%~%int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]~%int16 magDec            # Magnetic declination [deg / 1e-2]~%uint16 magAcc           # Magnetic declination accuracy [deg / 1e-2]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavPVTWithHeader>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     2
     1
     1
     1
     1
     1
     1
     4
     4
     1
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'reserved1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavPVTWithHeader>))
  "Converts a ROS message object to a list"
  (cl:list 'NavPVTWithHeader
    (cl:cons ':header (header msg))
    (cl:cons ':iTOW (iTOW msg))
    (cl:cons ':year (year msg))
    (cl:cons ':month (month msg))
    (cl:cons ':day (day msg))
    (cl:cons ':hour (hour msg))
    (cl:cons ':min (min msg))
    (cl:cons ':sec (sec msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':tAcc (tAcc msg))
    (cl:cons ':nano (nano msg))
    (cl:cons ':fixType (fixType msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':flags2 (flags2 msg))
    (cl:cons ':numSV (numSV msg))
    (cl:cons ':lon (lon msg))
    (cl:cons ':lat (lat msg))
    (cl:cons ':height (height msg))
    (cl:cons ':hMSL (hMSL msg))
    (cl:cons ':hAcc (hAcc msg))
    (cl:cons ':vAcc (vAcc msg))
    (cl:cons ':velN (velN msg))
    (cl:cons ':velE (velE msg))
    (cl:cons ':velD (velD msg))
    (cl:cons ':gSpeed (gSpeed msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':sAcc (sAcc msg))
    (cl:cons ':headAcc (headAcc msg))
    (cl:cons ':pDOP (pDOP msg))
    (cl:cons ':reserved1 (reserved1 msg))
    (cl:cons ':headVeh (headVeh msg))
    (cl:cons ':magDec (magDec msg))
    (cl:cons ':magAcc (magAcc msg))
))
