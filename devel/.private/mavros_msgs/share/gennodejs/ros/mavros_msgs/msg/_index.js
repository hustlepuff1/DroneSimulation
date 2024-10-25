
"use strict";

let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let State = require('./State.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CommandCode = require('./CommandCode.js');
let HilGPS = require('./HilGPS.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let HilSensor = require('./HilSensor.js');
let ESCStatus = require('./ESCStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let Thrust = require('./Thrust.js');
let ESCInfo = require('./ESCInfo.js');
let ActuatorControl = require('./ActuatorControl.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let SysStatus = require('./SysStatus.js');
let ManualControl = require('./ManualControl.js');
let DebugValue = require('./DebugValue.js');
let RTKBaseline = require('./RTKBaseline.js');
let RadioStatus = require('./RadioStatus.js');
let FileEntry = require('./FileEntry.js');
let BatteryStatus = require('./BatteryStatus.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let GPSRTK = require('./GPSRTK.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let ParamValue = require('./ParamValue.js');
let Mavlink = require('./Mavlink.js');
let HomePosition = require('./HomePosition.js');
let Param = require('./Param.js');
let WaypointList = require('./WaypointList.js');
let Trajectory = require('./Trajectory.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let ExtendedState = require('./ExtendedState.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let VehicleInfo = require('./VehicleInfo.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let VFR_HUD = require('./VFR_HUD.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let StatusText = require('./StatusText.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let CellularStatus = require('./CellularStatus.js');
let TerrainReport = require('./TerrainReport.js');
let HilControls = require('./HilControls.js');
let Tunnel = require('./Tunnel.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let WaypointReached = require('./WaypointReached.js');
let Vibration = require('./Vibration.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let GPSRAW = require('./GPSRAW.js');
let RTCM = require('./RTCM.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let RCIn = require('./RCIn.js');
let GPSINPUT = require('./GPSINPUT.js');
let LandingTarget = require('./LandingTarget.js');
let PositionTarget = require('./PositionTarget.js');
let Altitude = require('./Altitude.js');
let MountControl = require('./MountControl.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let RCOut = require('./RCOut.js');
let Waypoint = require('./Waypoint.js');
let LogData = require('./LogData.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let LogEntry = require('./LogEntry.js');

module.exports = {
  CompanionProcessStatus: CompanionProcessStatus,
  State: State,
  ADSBVehicle: ADSBVehicle,
  CommandCode: CommandCode,
  HilGPS: HilGPS,
  OverrideRCIn: OverrideRCIn,
  HilSensor: HilSensor,
  ESCStatus: ESCStatus,
  OpticalFlowRad: OpticalFlowRad,
  Thrust: Thrust,
  ESCInfo: ESCInfo,
  ActuatorControl: ActuatorControl,
  ESCTelemetryItem: ESCTelemetryItem,
  CamIMUStamp: CamIMUStamp,
  SysStatus: SysStatus,
  ManualControl: ManualControl,
  DebugValue: DebugValue,
  RTKBaseline: RTKBaseline,
  RadioStatus: RadioStatus,
  FileEntry: FileEntry,
  BatteryStatus: BatteryStatus,
  ESCTelemetry: ESCTelemetry,
  GPSRTK: GPSRTK,
  WheelOdomStamped: WheelOdomStamped,
  ParamValue: ParamValue,
  Mavlink: Mavlink,
  HomePosition: HomePosition,
  Param: Param,
  WaypointList: WaypointList,
  Trajectory: Trajectory,
  ESCStatusItem: ESCStatusItem,
  ExtendedState: ExtendedState,
  MagnetometerReporter: MagnetometerReporter,
  VehicleInfo: VehicleInfo,
  ESCInfoItem: ESCInfoItem,
  VFR_HUD: VFR_HUD,
  HilActuatorControls: HilActuatorControls,
  StatusText: StatusText,
  CameraImageCaptured: CameraImageCaptured,
  AttitudeTarget: AttitudeTarget,
  CellularStatus: CellularStatus,
  TerrainReport: TerrainReport,
  HilControls: HilControls,
  Tunnel: Tunnel,
  HilStateQuaternion: HilStateQuaternion,
  WaypointReached: WaypointReached,
  Vibration: Vibration,
  EstimatorStatus: EstimatorStatus,
  GPSRAW: GPSRAW,
  RTCM: RTCM,
  PlayTuneV2: PlayTuneV2,
  RCIn: RCIn,
  GPSINPUT: GPSINPUT,
  LandingTarget: LandingTarget,
  PositionTarget: PositionTarget,
  Altitude: Altitude,
  MountControl: MountControl,
  GlobalPositionTarget: GlobalPositionTarget,
  TimesyncStatus: TimesyncStatus,
  RCOut: RCOut,
  Waypoint: Waypoint,
  LogData: LogData,
  NavControllerOutput: NavControllerOutput,
  OnboardComputerStatus: OnboardComputerStatus,
  LogEntry: LogEntry,
};
