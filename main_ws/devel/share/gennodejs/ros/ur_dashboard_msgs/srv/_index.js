
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let GetProgramState = require('./GetProgramState.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let RawRequest = require('./RawRequest.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let AddToLog = require('./AddToLog.js')
let Popup = require('./Popup.js')
let Load = require('./Load.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  GetProgramState: GetProgramState,
  GetRobotMode: GetRobotMode,
  GetLoadedProgram: GetLoadedProgram,
  RawRequest: RawRequest,
  GetSafetyMode: GetSafetyMode,
  IsProgramSaved: IsProgramSaved,
  AddToLog: AddToLog,
  Popup: Popup,
  Load: Load,
};
