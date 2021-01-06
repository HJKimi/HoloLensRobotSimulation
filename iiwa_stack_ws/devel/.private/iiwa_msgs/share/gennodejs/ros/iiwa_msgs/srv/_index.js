
"use strict";

let SetSpeedOverride = require('./SetSpeedOverride.js')
let ConfigureControlMode = require('./ConfigureControlMode.js')
let SetSmartServoJointSpeedLimits = require('./SetSmartServoJointSpeedLimits.js')
let SetPTPJointSpeedLimits = require('./SetPTPJointSpeedLimits.js')
let SetSmartServoLinSpeedLimits = require('./SetSmartServoLinSpeedLimits.js')
let SetPTPCartesianSpeedLimits = require('./SetPTPCartesianSpeedLimits.js')
let SetWorkpiece = require('./SetWorkpiece.js')
let TimeToDestination = require('./TimeToDestination.js')
let SetEndpointFrame = require('./SetEndpointFrame.js')

module.exports = {
  SetSpeedOverride: SetSpeedOverride,
  ConfigureControlMode: ConfigureControlMode,
  SetSmartServoJointSpeedLimits: SetSmartServoJointSpeedLimits,
  SetPTPJointSpeedLimits: SetPTPJointSpeedLimits,
  SetSmartServoLinSpeedLimits: SetSmartServoLinSpeedLimits,
  SetPTPCartesianSpeedLimits: SetPTPCartesianSpeedLimits,
  SetWorkpiece: SetWorkpiece,
  TimeToDestination: TimeToDestination,
  SetEndpointFrame: SetEndpointFrame,
};
