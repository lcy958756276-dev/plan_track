
"use strict";

let ImmDebugInfo = require('./ImmDebugInfo.js');
let TrackedPersons2d = require('./TrackedPersons2d.js');
let TrackedPersons = require('./TrackedPersons.js');
let TrackedPerson2d = require('./TrackedPerson2d.js');
let CompositeDetectedPersons = require('./CompositeDetectedPersons.js');
let ImmDebugInfos = require('./ImmDebugInfos.js');
let TrackingTimingMetrics = require('./TrackingTimingMetrics.js');
let PersonTrajectoryEntry = require('./PersonTrajectoryEntry.js');
let TrackedGroups = require('./TrackedGroups.js');
let TrackedPerson = require('./TrackedPerson.js');
let DetectedPersons = require('./DetectedPersons.js');
let TrackedGroup = require('./TrackedGroup.js');
let PersonTrajectory = require('./PersonTrajectory.js');
let DetectedPerson = require('./DetectedPerson.js');
let CompositeDetectedPerson = require('./CompositeDetectedPerson.js');

module.exports = {
  ImmDebugInfo: ImmDebugInfo,
  TrackedPersons2d: TrackedPersons2d,
  TrackedPersons: TrackedPersons,
  TrackedPerson2d: TrackedPerson2d,
  CompositeDetectedPersons: CompositeDetectedPersons,
  ImmDebugInfos: ImmDebugInfos,
  TrackingTimingMetrics: TrackingTimingMetrics,
  PersonTrajectoryEntry: PersonTrajectoryEntry,
  TrackedGroups: TrackedGroups,
  TrackedPerson: TrackedPerson,
  DetectedPersons: DetectedPersons,
  TrackedGroup: TrackedGroup,
  PersonTrajectory: PersonTrajectory,
  DetectedPerson: DetectedPerson,
  CompositeDetectedPerson: CompositeDetectedPerson,
};
