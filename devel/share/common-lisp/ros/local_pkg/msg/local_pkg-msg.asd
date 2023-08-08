
(cl:in-package :asdf)

(defsystem "local_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control_Info" :depends-on ("_package_Control_Info"))
    (:file "_package_Control_Info" :depends-on ("_package"))
    (:file "Control_Info" :depends-on ("_package_Control_Info"))
    (:file "_package_Control_Info" :depends-on ("_package"))
    (:file "Local" :depends-on ("_package_Local"))
    (:file "_package_Local" :depends-on ("_package"))
    (:file "Local" :depends-on ("_package_Local"))
    (:file "_package_Local" :depends-on ("_package"))
    (:file "Master" :depends-on ("_package_Master"))
    (:file "_package_Master" :depends-on ("_package"))
    (:file "Master" :depends-on ("_package_Master"))
    (:file "_package_Master" :depends-on ("_package"))
    (:file "NavPVTWithHeader" :depends-on ("_package_NavPVTWithHeader"))
    (:file "_package_NavPVTWithHeader" :depends-on ("_package"))
    (:file "NavPVTWithHeader" :depends-on ("_package_NavPVTWithHeader"))
    (:file "_package_NavPVTWithHeader" :depends-on ("_package"))
    (:file "Path" :depends-on ("_package_Path"))
    (:file "_package_Path" :depends-on ("_package"))
    (:file "Path" :depends-on ("_package_Path"))
    (:file "_package_Path" :depends-on ("_package"))
    (:file "Perception" :depends-on ("_package_Perception"))
    (:file "_package_Perception" :depends-on ("_package"))
    (:file "Perception" :depends-on ("_package_Perception"))
    (:file "_package_Perception" :depends-on ("_package"))
    (:file "PoseStamped" :depends-on ("_package_PoseStamped"))
    (:file "_package_PoseStamped" :depends-on ("_package"))
    (:file "PoseStamped" :depends-on ("_package_PoseStamped"))
    (:file "_package_PoseStamped" :depends-on ("_package"))
    (:file "SbgAirData" :depends-on ("_package_SbgAirData"))
    (:file "_package_SbgAirData" :depends-on ("_package"))
    (:file "SbgAirData" :depends-on ("_package_SbgAirData"))
    (:file "_package_SbgAirData" :depends-on ("_package"))
    (:file "SbgAirDataStatus" :depends-on ("_package_SbgAirDataStatus"))
    (:file "_package_SbgAirDataStatus" :depends-on ("_package"))
    (:file "SbgAirDataStatus" :depends-on ("_package_SbgAirDataStatus"))
    (:file "_package_SbgAirDataStatus" :depends-on ("_package"))
    (:file "SbgEkfEuler" :depends-on ("_package_SbgEkfEuler"))
    (:file "_package_SbgEkfEuler" :depends-on ("_package"))
    (:file "SbgEkfEuler" :depends-on ("_package_SbgEkfEuler"))
    (:file "_package_SbgEkfEuler" :depends-on ("_package"))
    (:file "SbgEkfNav" :depends-on ("_package_SbgEkfNav"))
    (:file "_package_SbgEkfNav" :depends-on ("_package"))
    (:file "SbgEkfNav" :depends-on ("_package_SbgEkfNav"))
    (:file "_package_SbgEkfNav" :depends-on ("_package"))
    (:file "SbgEkfQuat" :depends-on ("_package_SbgEkfQuat"))
    (:file "_package_SbgEkfQuat" :depends-on ("_package"))
    (:file "SbgEkfQuat" :depends-on ("_package_SbgEkfQuat"))
    (:file "_package_SbgEkfQuat" :depends-on ("_package"))
    (:file "SbgEkfStatus" :depends-on ("_package_SbgEkfStatus"))
    (:file "_package_SbgEkfStatus" :depends-on ("_package"))
    (:file "SbgEkfStatus" :depends-on ("_package_SbgEkfStatus"))
    (:file "_package_SbgEkfStatus" :depends-on ("_package"))
    (:file "SbgEvent" :depends-on ("_package_SbgEvent"))
    (:file "_package_SbgEvent" :depends-on ("_package"))
    (:file "SbgEvent" :depends-on ("_package_SbgEvent"))
    (:file "_package_SbgEvent" :depends-on ("_package"))
    (:file "SbgGpsHdt" :depends-on ("_package_SbgGpsHdt"))
    (:file "_package_SbgGpsHdt" :depends-on ("_package"))
    (:file "SbgGpsHdt" :depends-on ("_package_SbgGpsHdt"))
    (:file "_package_SbgGpsHdt" :depends-on ("_package"))
    (:file "SbgGpsPos" :depends-on ("_package_SbgGpsPos"))
    (:file "_package_SbgGpsPos" :depends-on ("_package"))
    (:file "SbgGpsPos" :depends-on ("_package_SbgGpsPos"))
    (:file "_package_SbgGpsPos" :depends-on ("_package"))
    (:file "SbgGpsPosStatus" :depends-on ("_package_SbgGpsPosStatus"))
    (:file "_package_SbgGpsPosStatus" :depends-on ("_package"))
    (:file "SbgGpsPosStatus" :depends-on ("_package_SbgGpsPosStatus"))
    (:file "_package_SbgGpsPosStatus" :depends-on ("_package"))
    (:file "SbgGpsRaw" :depends-on ("_package_SbgGpsRaw"))
    (:file "_package_SbgGpsRaw" :depends-on ("_package"))
    (:file "SbgGpsRaw" :depends-on ("_package_SbgGpsRaw"))
    (:file "_package_SbgGpsRaw" :depends-on ("_package"))
    (:file "SbgGpsVel" :depends-on ("_package_SbgGpsVel"))
    (:file "_package_SbgGpsVel" :depends-on ("_package"))
    (:file "SbgGpsVel" :depends-on ("_package_SbgGpsVel"))
    (:file "_package_SbgGpsVel" :depends-on ("_package"))
    (:file "SbgGpsVelStatus" :depends-on ("_package_SbgGpsVelStatus"))
    (:file "_package_SbgGpsVelStatus" :depends-on ("_package"))
    (:file "SbgGpsVelStatus" :depends-on ("_package_SbgGpsVelStatus"))
    (:file "_package_SbgGpsVelStatus" :depends-on ("_package"))
    (:file "SbgImuData" :depends-on ("_package_SbgImuData"))
    (:file "_package_SbgImuData" :depends-on ("_package"))
    (:file "SbgImuData" :depends-on ("_package_SbgImuData"))
    (:file "_package_SbgImuData" :depends-on ("_package"))
    (:file "SbgImuShort" :depends-on ("_package_SbgImuShort"))
    (:file "_package_SbgImuShort" :depends-on ("_package"))
    (:file "SbgImuShort" :depends-on ("_package_SbgImuShort"))
    (:file "_package_SbgImuShort" :depends-on ("_package"))
    (:file "SbgImuStatus" :depends-on ("_package_SbgImuStatus"))
    (:file "_package_SbgImuStatus" :depends-on ("_package"))
    (:file "SbgImuStatus" :depends-on ("_package_SbgImuStatus"))
    (:file "_package_SbgImuStatus" :depends-on ("_package"))
    (:file "SbgMag" :depends-on ("_package_SbgMag"))
    (:file "_package_SbgMag" :depends-on ("_package"))
    (:file "SbgMag" :depends-on ("_package_SbgMag"))
    (:file "_package_SbgMag" :depends-on ("_package"))
    (:file "SbgMagCalib" :depends-on ("_package_SbgMagCalib"))
    (:file "_package_SbgMagCalib" :depends-on ("_package"))
    (:file "SbgMagCalib" :depends-on ("_package_SbgMagCalib"))
    (:file "_package_SbgMagCalib" :depends-on ("_package"))
    (:file "SbgMagStatus" :depends-on ("_package_SbgMagStatus"))
    (:file "_package_SbgMagStatus" :depends-on ("_package"))
    (:file "SbgMagStatus" :depends-on ("_package_SbgMagStatus"))
    (:file "_package_SbgMagStatus" :depends-on ("_package"))
    (:file "SbgOdoVel" :depends-on ("_package_SbgOdoVel"))
    (:file "_package_SbgOdoVel" :depends-on ("_package"))
    (:file "SbgOdoVel" :depends-on ("_package_SbgOdoVel"))
    (:file "_package_SbgOdoVel" :depends-on ("_package"))
    (:file "SbgShipMotion" :depends-on ("_package_SbgShipMotion"))
    (:file "_package_SbgShipMotion" :depends-on ("_package"))
    (:file "SbgShipMotion" :depends-on ("_package_SbgShipMotion"))
    (:file "_package_SbgShipMotion" :depends-on ("_package"))
    (:file "SbgShipMotionStatus" :depends-on ("_package_SbgShipMotionStatus"))
    (:file "_package_SbgShipMotionStatus" :depends-on ("_package"))
    (:file "SbgShipMotionStatus" :depends-on ("_package_SbgShipMotionStatus"))
    (:file "_package_SbgShipMotionStatus" :depends-on ("_package"))
    (:file "SbgStatus" :depends-on ("_package_SbgStatus"))
    (:file "_package_SbgStatus" :depends-on ("_package"))
    (:file "SbgStatus" :depends-on ("_package_SbgStatus"))
    (:file "_package_SbgStatus" :depends-on ("_package"))
    (:file "SbgStatusAiding" :depends-on ("_package_SbgStatusAiding"))
    (:file "_package_SbgStatusAiding" :depends-on ("_package"))
    (:file "SbgStatusAiding" :depends-on ("_package_SbgStatusAiding"))
    (:file "_package_SbgStatusAiding" :depends-on ("_package"))
    (:file "SbgStatusCom" :depends-on ("_package_SbgStatusCom"))
    (:file "_package_SbgStatusCom" :depends-on ("_package"))
    (:file "SbgStatusCom" :depends-on ("_package_SbgStatusCom"))
    (:file "_package_SbgStatusCom" :depends-on ("_package"))
    (:file "SbgStatusGeneral" :depends-on ("_package_SbgStatusGeneral"))
    (:file "_package_SbgStatusGeneral" :depends-on ("_package"))
    (:file "SbgStatusGeneral" :depends-on ("_package_SbgStatusGeneral"))
    (:file "_package_SbgStatusGeneral" :depends-on ("_package"))
    (:file "SbgUtcTime" :depends-on ("_package_SbgUtcTime"))
    (:file "_package_SbgUtcTime" :depends-on ("_package"))
    (:file "SbgUtcTime" :depends-on ("_package_SbgUtcTime"))
    (:file "_package_SbgUtcTime" :depends-on ("_package"))
    (:file "SbgUtcTimeStatus" :depends-on ("_package_SbgUtcTimeStatus"))
    (:file "_package_SbgUtcTimeStatus" :depends-on ("_package"))
    (:file "SbgUtcTimeStatus" :depends-on ("_package_SbgUtcTimeStatus"))
    (:file "_package_SbgUtcTimeStatus" :depends-on ("_package"))
    (:file "Serial_Info" :depends-on ("_package_Serial_Info"))
    (:file "_package_Serial_Info" :depends-on ("_package"))
    (:file "Serial_Info" :depends-on ("_package_Serial_Info"))
    (:file "_package_Serial_Info" :depends-on ("_package"))
  ))