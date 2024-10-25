
"use strict";

let ParamGet = require('./ParamGet.js')
let LogRequestData = require('./LogRequestData.js')
let MountConfigure = require('./MountConfigure.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileRename = require('./FileRename.js')
let FileWrite = require('./FileWrite.js')
let CommandBool = require('./CommandBool.js')
let ParamPush = require('./ParamPush.js')
let CommandInt = require('./CommandInt.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let SetMode = require('./SetMode.js')
let CommandLong = require('./CommandLong.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let WaypointPush = require('./WaypointPush.js')
let FileRead = require('./FileRead.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandHome = require('./CommandHome.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileList = require('./FileList.js')
let FileClose = require('./FileClose.js')
let WaypointClear = require('./WaypointClear.js')
let MessageInterval = require('./MessageInterval.js')
let StreamRate = require('./StreamRate.js')
let LogRequestList = require('./LogRequestList.js')
let FileOpen = require('./FileOpen.js')
let FileChecksum = require('./FileChecksum.js')
let ParamSet = require('./ParamSet.js')
let CommandAck = require('./CommandAck.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandTOL = require('./CommandTOL.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileRemove = require('./FileRemove.js')
let ParamPull = require('./ParamPull.js')
let FileTruncate = require('./FileTruncate.js')
let WaypointPull = require('./WaypointPull.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')

module.exports = {
  ParamGet: ParamGet,
  LogRequestData: LogRequestData,
  MountConfigure: MountConfigure,
  FileRemoveDir: FileRemoveDir,
  FileRename: FileRename,
  FileWrite: FileWrite,
  CommandBool: CommandBool,
  ParamPush: ParamPush,
  CommandInt: CommandInt,
  LogRequestEnd: LogRequestEnd,
  VehicleInfoGet: VehicleInfoGet,
  SetMode: SetMode,
  CommandLong: CommandLong,
  CommandTriggerInterval: CommandTriggerInterval,
  WaypointPush: WaypointPush,
  FileRead: FileRead,
  FileMakeDir: FileMakeDir,
  CommandHome: CommandHome,
  CommandVtolTransition: CommandVtolTransition,
  FileList: FileList,
  FileClose: FileClose,
  WaypointClear: WaypointClear,
  MessageInterval: MessageInterval,
  StreamRate: StreamRate,
  LogRequestList: LogRequestList,
  FileOpen: FileOpen,
  FileChecksum: FileChecksum,
  ParamSet: ParamSet,
  CommandAck: CommandAck,
  SetMavFrame: SetMavFrame,
  CommandTOL: CommandTOL,
  CommandTriggerControl: CommandTriggerControl,
  FileRemove: FileRemove,
  ParamPull: ParamPull,
  FileTruncate: FileTruncate,
  WaypointPull: WaypointPull,
  WaypointSetCurrent: WaypointSetCurrent,
};
