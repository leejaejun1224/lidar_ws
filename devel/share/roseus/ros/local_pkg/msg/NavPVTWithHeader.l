;; Auto-generated. Do not edit!


(when (boundp 'local_pkg::NavPVTWithHeader)
  (if (not (find-package "LOCAL_PKG"))
    (make-package "LOCAL_PKG"))
  (shadow 'NavPVTWithHeader (find-package "LOCAL_PKG")))
(unless (find-package "LOCAL_PKG::NAVPVTWITHHEADER")
  (make-package "LOCAL_PKG::NAVPVTWITHHEADER"))

(in-package "ROS")
;;//! \htmlinclude NavPVTWithHeader.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CLASS_ID*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*CLASS_ID* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*CLASS_ID* 1)
(intern "*MESSAGE_ID*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*MESSAGE_ID* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*MESSAGE_ID* 7)
(intern "*VALID_DATE*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*VALID_DATE* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*VALID_DATE* 1)
(intern "*VALID_TIME*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*VALID_TIME* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*VALID_TIME* 2)
(intern "*VALID_FULLY_RESOLVED*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*VALID_FULLY_RESOLVED* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*VALID_FULLY_RESOLVED* 4)
(intern "*VALID_MAG*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*VALID_MAG* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*VALID_MAG* 8)
(intern "*FIX_TYPE_NO_FIX*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FIX_TYPE_NO_FIX* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FIX_TYPE_NO_FIX* 0)
(intern "*FIX_TYPE_DEAD_RECKONING_ONLY*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FIX_TYPE_DEAD_RECKONING_ONLY* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FIX_TYPE_DEAD_RECKONING_ONLY* 1)
(intern "*FIX_TYPE_2D*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FIX_TYPE_2D* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FIX_TYPE_2D* 2)
(intern "*FIX_TYPE_3D*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FIX_TYPE_3D* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FIX_TYPE_3D* 3)
(intern "*FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED* 4)
(intern "*FIX_TYPE_TIME_ONLY*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FIX_TYPE_TIME_ONLY* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FIX_TYPE_TIME_ONLY* 5)
(intern "*FLAGS_GNSS_FIX_OK*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS_GNSS_FIX_OK* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS_GNSS_FIX_OK* 1)
(intern "*FLAGS_DIFF_SOLN*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS_DIFF_SOLN* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS_DIFF_SOLN* 2)
(intern "*FLAGS_PSM_MASK*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS_PSM_MASK* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS_PSM_MASK* 28)
(intern "*PSM_OFF*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*PSM_OFF* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*PSM_OFF* 0)
(intern "*PSM_ENABLED*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*PSM_ENABLED* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*PSM_ENABLED* 4)
(intern "*PSM_ACQUIRED*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*PSM_ACQUIRED* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*PSM_ACQUIRED* 8)
(intern "*PSM_TRACKING*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*PSM_TRACKING* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*PSM_TRACKING* 12)
(intern "*PSM_POWER_OPTIMIZED_TRACKING*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*PSM_POWER_OPTIMIZED_TRACKING* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*PSM_POWER_OPTIMIZED_TRACKING* 16)
(intern "*PSM_INACTIVE*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*PSM_INACTIVE* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*PSM_INACTIVE* 20)
(intern "*FLAGS_HEAD_VEH_VALID*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS_HEAD_VEH_VALID* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS_HEAD_VEH_VALID* 32)
(intern "*FLAGS_CARRIER_PHASE_MASK*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS_CARRIER_PHASE_MASK* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS_CARRIER_PHASE_MASK* 192)
(intern "*CARRIER_PHASE_NO_SOLUTION*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*CARRIER_PHASE_NO_SOLUTION* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*CARRIER_PHASE_NO_SOLUTION* 0)
(intern "*CARRIER_PHASE_FLOAT*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*CARRIER_PHASE_FLOAT* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*CARRIER_PHASE_FLOAT* 64)
(intern "*CARRIER_PHASE_FIXED*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*CARRIER_PHASE_FIXED* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*CARRIER_PHASE_FIXED* 128)
(intern "*FLAGS2_CONFIRMED_AVAILABLE*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS2_CONFIRMED_AVAILABLE* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS2_CONFIRMED_AVAILABLE* 32)
(intern "*FLAGS2_CONFIRMED_DATE*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS2_CONFIRMED_DATE* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS2_CONFIRMED_DATE* 64)
(intern "*FLAGS2_CONFIRMED_TIME*" (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(shadow '*FLAGS2_CONFIRMED_TIME* (find-package "LOCAL_PKG::NAVPVTWITHHEADER"))
(defconstant local_pkg::NavPVTWithHeader::*FLAGS2_CONFIRMED_TIME* 128)
(defclass local_pkg::NavPVTWithHeader
  :super ros::object
  :slots (_header _iTOW _year _month _day _hour _min _sec _valid _tAcc _nano _fixType _flags _flags2 _numSV _lon _lat _height _hMSL _hAcc _vAcc _velN _velE _velD _gSpeed _heading _sAcc _headAcc _pDOP _reserved1 _headVeh _magDec _magAcc ))

(defmethod local_pkg::NavPVTWithHeader
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:iTOW __iTOW) 0)
    ((:year __year) 0)
    ((:month __month) 0)
    ((:day __day) 0)
    ((:hour __hour) 0)
    ((:min __min) 0)
    ((:sec __sec) 0)
    ((:valid __valid) 0)
    ((:tAcc __tAcc) 0)
    ((:nano __nano) 0)
    ((:fixType __fixType) 0)
    ((:flags __flags) 0)
    ((:flags2 __flags2) 0)
    ((:numSV __numSV) 0)
    ((:lon __lon) 0)
    ((:lat __lat) 0)
    ((:height __height) 0)
    ((:hMSL __hMSL) 0)
    ((:hAcc __hAcc) 0)
    ((:vAcc __vAcc) 0)
    ((:velN __velN) 0)
    ((:velE __velE) 0)
    ((:velD __velD) 0)
    ((:gSpeed __gSpeed) 0)
    ((:heading __heading) 0)
    ((:sAcc __sAcc) 0)
    ((:headAcc __headAcc) 0)
    ((:pDOP __pDOP) 0)
    ((:reserved1 __reserved1) (make-array 6 :initial-element 0 :element-type :char))
    ((:headVeh __headVeh) 0)
    ((:magDec __magDec) 0)
    ((:magAcc __magAcc) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _iTOW (round __iTOW))
   (setq _year (round __year))
   (setq _month (round __month))
   (setq _day (round __day))
   (setq _hour (round __hour))
   (setq _min (round __min))
   (setq _sec (round __sec))
   (setq _valid (round __valid))
   (setq _tAcc (round __tAcc))
   (setq _nano (round __nano))
   (setq _fixType (round __fixType))
   (setq _flags (round __flags))
   (setq _flags2 (round __flags2))
   (setq _numSV (round __numSV))
   (setq _lon (round __lon))
   (setq _lat (round __lat))
   (setq _height (round __height))
   (setq _hMSL (round __hMSL))
   (setq _hAcc (round __hAcc))
   (setq _vAcc (round __vAcc))
   (setq _velN (round __velN))
   (setq _velE (round __velE))
   (setq _velD (round __velD))
   (setq _gSpeed (round __gSpeed))
   (setq _heading (round __heading))
   (setq _sAcc (round __sAcc))
   (setq _headAcc (round __headAcc))
   (setq _pDOP (round __pDOP))
   (setq _reserved1 __reserved1)
   (setq _headVeh (round __headVeh))
   (setq _magDec (round __magDec))
   (setq _magAcc (round __magAcc))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:iTOW
   (&optional __iTOW)
   (if __iTOW (setq _iTOW __iTOW)) _iTOW)
  (:year
   (&optional __year)
   (if __year (setq _year __year)) _year)
  (:month
   (&optional __month)
   (if __month (setq _month __month)) _month)
  (:day
   (&optional __day)
   (if __day (setq _day __day)) _day)
  (:hour
   (&optional __hour)
   (if __hour (setq _hour __hour)) _hour)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:sec
   (&optional __sec)
   (if __sec (setq _sec __sec)) _sec)
  (:valid
   (&optional __valid)
   (if __valid (setq _valid __valid)) _valid)
  (:tAcc
   (&optional __tAcc)
   (if __tAcc (setq _tAcc __tAcc)) _tAcc)
  (:nano
   (&optional __nano)
   (if __nano (setq _nano __nano)) _nano)
  (:fixType
   (&optional __fixType)
   (if __fixType (setq _fixType __fixType)) _fixType)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:flags2
   (&optional __flags2)
   (if __flags2 (setq _flags2 __flags2)) _flags2)
  (:numSV
   (&optional __numSV)
   (if __numSV (setq _numSV __numSV)) _numSV)
  (:lon
   (&optional __lon)
   (if __lon (setq _lon __lon)) _lon)
  (:lat
   (&optional __lat)
   (if __lat (setq _lat __lat)) _lat)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:hMSL
   (&optional __hMSL)
   (if __hMSL (setq _hMSL __hMSL)) _hMSL)
  (:hAcc
   (&optional __hAcc)
   (if __hAcc (setq _hAcc __hAcc)) _hAcc)
  (:vAcc
   (&optional __vAcc)
   (if __vAcc (setq _vAcc __vAcc)) _vAcc)
  (:velN
   (&optional __velN)
   (if __velN (setq _velN __velN)) _velN)
  (:velE
   (&optional __velE)
   (if __velE (setq _velE __velE)) _velE)
  (:velD
   (&optional __velD)
   (if __velD (setq _velD __velD)) _velD)
  (:gSpeed
   (&optional __gSpeed)
   (if __gSpeed (setq _gSpeed __gSpeed)) _gSpeed)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:sAcc
   (&optional __sAcc)
   (if __sAcc (setq _sAcc __sAcc)) _sAcc)
  (:headAcc
   (&optional __headAcc)
   (if __headAcc (setq _headAcc __headAcc)) _headAcc)
  (:pDOP
   (&optional __pDOP)
   (if __pDOP (setq _pDOP __pDOP)) _pDOP)
  (:reserved1
   (&optional __reserved1)
   (if __reserved1 (setq _reserved1 __reserved1)) _reserved1)
  (:headVeh
   (&optional __headVeh)
   (if __headVeh (setq _headVeh __headVeh)) _headVeh)
  (:magDec
   (&optional __magDec)
   (if __magDec (setq _magDec __magDec)) _magDec)
  (:magAcc
   (&optional __magAcc)
   (if __magAcc (setq _magAcc __magAcc)) _magAcc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _iTOW
    4
    ;; uint16 _year
    2
    ;; uint8 _month
    1
    ;; uint8 _day
    1
    ;; uint8 _hour
    1
    ;; uint8 _min
    1
    ;; uint8 _sec
    1
    ;; uint8 _valid
    1
    ;; uint32 _tAcc
    4
    ;; int32 _nano
    4
    ;; uint8 _fixType
    1
    ;; uint8 _flags
    1
    ;; uint8 _flags2
    1
    ;; uint8 _numSV
    1
    ;; int32 _lon
    4
    ;; int32 _lat
    4
    ;; int32 _height
    4
    ;; int32 _hMSL
    4
    ;; uint32 _hAcc
    4
    ;; uint32 _vAcc
    4
    ;; int32 _velN
    4
    ;; int32 _velE
    4
    ;; int32 _velD
    4
    ;; int32 _gSpeed
    4
    ;; int32 _heading
    4
    ;; uint32 _sAcc
    4
    ;; uint32 _headAcc
    4
    ;; uint16 _pDOP
    2
    ;; uint8[6] _reserved1
    (* 1    6)
    ;; int32 _headVeh
    4
    ;; int16 _magDec
    2
    ;; uint16 _magAcc
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _iTOW
       (write-long _iTOW s)
     ;; uint16 _year
       (write-word _year s)
     ;; uint8 _month
       (write-byte _month s)
     ;; uint8 _day
       (write-byte _day s)
     ;; uint8 _hour
       (write-byte _hour s)
     ;; uint8 _min
       (write-byte _min s)
     ;; uint8 _sec
       (write-byte _sec s)
     ;; uint8 _valid
       (write-byte _valid s)
     ;; uint32 _tAcc
       (write-long _tAcc s)
     ;; int32 _nano
       (write-long _nano s)
     ;; uint8 _fixType
       (write-byte _fixType s)
     ;; uint8 _flags
       (write-byte _flags s)
     ;; uint8 _flags2
       (write-byte _flags2 s)
     ;; uint8 _numSV
       (write-byte _numSV s)
     ;; int32 _lon
       (write-long _lon s)
     ;; int32 _lat
       (write-long _lat s)
     ;; int32 _height
       (write-long _height s)
     ;; int32 _hMSL
       (write-long _hMSL s)
     ;; uint32 _hAcc
       (write-long _hAcc s)
     ;; uint32 _vAcc
       (write-long _vAcc s)
     ;; int32 _velN
       (write-long _velN s)
     ;; int32 _velE
       (write-long _velE s)
     ;; int32 _velD
       (write-long _velD s)
     ;; int32 _gSpeed
       (write-long _gSpeed s)
     ;; int32 _heading
       (write-long _heading s)
     ;; uint32 _sAcc
       (write-long _sAcc s)
     ;; uint32 _headAcc
       (write-long _headAcc s)
     ;; uint16 _pDOP
       (write-word _pDOP s)
     ;; uint8[6] _reserved1
     (princ _reserved1 s)
     ;; int32 _headVeh
       (write-long _headVeh s)
     ;; int16 _magDec
       (write-word _magDec s)
     ;; uint16 _magAcc
       (write-word _magAcc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _iTOW
     (setq _iTOW (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _year
     (setq _year (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _month
     (setq _month (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _day
     (setq _day (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _hour
     (setq _hour (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _min
     (setq _min (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sec
     (setq _sec (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _valid
     (setq _valid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _tAcc
     (setq _tAcc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _nano
     (setq _nano (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _fixType
     (setq _fixType (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _flags
     (setq _flags (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _flags2
     (setq _flags2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _numSV
     (setq _numSV (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _lon
     (setq _lon (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _lat
     (setq _lat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _hMSL
     (setq _hMSL (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _hAcc
     (setq _hAcc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _vAcc
     (setq _vAcc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _velN
     (setq _velN (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _velE
     (setq _velE (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _velD
     (setq _velD (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _gSpeed
     (setq _gSpeed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _heading
     (setq _heading (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _sAcc
     (setq _sAcc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _headAcc
     (setq _headAcc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _pDOP
     (setq _pDOP (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8[6] _reserved1
   (setq _reserved1 (make-array 6 :element-type :char))
   (replace _reserved1 buf :start2 ptr-) (incf ptr- 6)
   ;; int32 _headVeh
     (setq _headVeh (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _magDec
     (setq _magDec (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _magAcc
     (setq _magAcc (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get local_pkg::NavPVTWithHeader :md5sum-) "76a709426465468a80d5b09ed58df78d")
(setf (get local_pkg::NavPVTWithHeader :datatype-) "local_pkg/NavPVTWithHeader")
(setf (get local_pkg::NavPVTWithHeader :definition-)
      "# NAV-PVT (0x01 0x07)
# Navigation Position Velocity Time Solution
#
# Note that during a leap second there may be more (or less) than 60 seconds in
# a minute; see the description of leap seconds for details.
#
# This message combines Position, velocity and time solution in LLH, 
# including accuracy figures
#
# WARNING: For firmware version 7, this message is a different length.
#

std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

uint32 iTOW             # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 valid             # Validity flags
uint8 VALID_DATE = 1            # Valid UTC Date
uint8 VALID_TIME = 2            # Valid 
uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved 
                                # (no seconds uncertainty)
uint8 VALID_MAG = 8             # Valid Magnetic Declination

uint32 tAcc             # time accuracy estimate [ns] (UTC)
int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)

uint8 fixType           # GNSS fix Type, range 0..5
uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, 
                                                # constant altitude assumed
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning
uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix (High precision 
                                                # devices)

uint8 flags             # Fix Status Flags
uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks
uint8 FLAGS_DIFF_SOLN = 2            # DGPS used
uint8 FLAGS_PSM_MASK = 28            # Power Save Mode
uint8 PSM_OFF = 0                       # PSM is off
uint8 PSM_ENABLED = 4                   # Enabled (state before acquisition)
uint8 PSM_ACQUIRED = 8                  # Acquisition
uint8 PSM_TRACKING = 12                 # Tracking
uint8 PSM_POWER_OPTIMIZED_TRACKING = 16 # Power Optimized Tracking
uint8 PSM_INACTIVE = 20                 # Inactive
uint8 FLAGS_HEAD_VEH_VALID = 32         # heading of vehicle is valid
uint8 FLAGS_CARRIER_PHASE_MASK = 192 # Carrier Phase Range Solution Status     
uint8 CARRIER_PHASE_NO_SOLUTION = 0     # no carrier phase range solution
uint8 CARRIER_PHASE_FLOAT = 64          # carrier phase float solution (no fixed 
                                        # integer measurements have been used to 
                                        # calculate the solution)
uint8 CARRIER_PHASE_FIXED = 128         # fixed solution (>=1 fixed integer 
                                        # carrier phase range measurements have 
                                        # been used to calculate  the solution)

uint8 flags2            # Additional Flags
uint8 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of 
                                        # Day validity confirmation is available
uint8 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed
uint8 FLAGS2_CONFIRMED_TIME = 128       # UTC Time of Day could be confirmed

uint8 numSV             # Number of SVs used in Nav Solution
int32 lon               # Longitude [deg / 1e-7]
int32 lat               # Latitude [deg / 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 hMSL              # Height above mean sea level [mm]
uint32 hAcc             # Horizontal Accuracy Estimate [mm]
uint32 vAcc             # Vertical Accuracy Estimate [mm]

int32 velN              # NED north velocity [mm/s]
int32 velE              # NED east velocity [mm/s]
int32 velD              # NED down velocity [mm/s]
int32 gSpeed            # Ground Speed (2-D) [mm/s]
int32 heading           # Heading of motion 2-D [deg / 1e-5]
uint32 sAcc             # Speed Accuracy Estimate [mm/s]
uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) 
                        # [deg / 1e-5]

uint16 pDOP             # Position DOP [1 / 0.01]
uint8[6] reserved1      # Reserved

int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]
int16 magDec            # Magnetic declination [deg / 1e-2]
uint16 magAcc           # Magnetic declination accuracy [deg / 1e-2]

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :local_pkg/NavPVTWithHeader "76a709426465468a80d5b09ed58df78d")


