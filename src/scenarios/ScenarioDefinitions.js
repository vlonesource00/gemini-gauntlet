/**
 * ScenarioDefinitions.js
 *
 * Authored tactical scenarios, race presets, hotlap benchmarks, and battle setups
 * for the AI and physics simulation.
 */

export const SCENARIO_CATEGORIES = Object.freeze([
  'attack',
  'defense',
  'hotlap',
  'duel'
]);

export const SCENARIOS = Object.freeze({
  A1_STRAIGHT_SLIPSTREAM: Object.freeze({
    id: 'A1_STRAIGHT_SLIPSTREAM',
    name: 'Straight Slipstream & ERS Dive',
    category: 'attack',
    trackSection: 'Start-Finish Straight',
    description: 'Player in front at 200 km/h, AI behind at 225 km/h drafting on main straight (distance ~100m), tests high-speed slipstream + ERS divebomb.',
    trackDistance: 100,
    durationS: 18,
    playerConfig: Object.freeze({
      spec: 'prototype',
      distance: 100,
      lateralOffset: 0.0,
      initialSpeedKph: 200,
      initialSpeedMps: 200 / 3.6,
      ersMode: 'AUTO',
      behavior: 'pace'
    }),
    aiConfig: Object.freeze({
      spec: 'prototype',
      distance: 75,
      lateralOffset: 0.0,
      initialSpeedKph: 225,
      initialSpeedMps: 225 / 3.6,
      initialManeuver: 'SLIPSTREAM_DRAFT',
      initialAggression: 0.88,
      ersMode: 'ATTACK',
      targetPaceKph: 260
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_PASS_CLEAN',
      minClearanceM: 1.2,
      maxContactCount: 0,
      holdLeadTimeS: 1.0,
      allowOffTrack: false,
      description: 'AI drafts in high-speed wake, executes ERS-assisted slingshot pass, and maintains clean lead.'
    })
  }),

  A2_CHICANE_ATTACK: Object.freeze({
    id: 'A2_CHICANE_ATTACK',
    name: 'Quarry Chicane Late Brake Dive',
    category: 'attack',
    trackSection: 'Quarry Chicane Approach',
    description: 'Approach to Quarry Chicane (distance ~650m), player braking for chicane at 160 km/h, AI at 180 km/h executing late braking dive on inside.',
    trackDistance: 650,
    durationS: 16,
    playerConfig: Object.freeze({
      spec: 'gt',
      distance: 660,
      lateralOffset: 2.0,
      initialSpeedKph: 160,
      initialSpeedMps: 160 / 3.6,
      ersMode: 'OFF',
      behavior: 'braking_standard'
    }),
    aiConfig: Object.freeze({
      spec: 'gt',
      distance: 635,
      lateralOffset: -1.8,
      initialSpeedKph: 180,
      initialSpeedMps: 180 / 3.6,
      initialManeuver: 'ATTACK_DIVE',
      initialAggression: 0.92,
      ersMode: 'OFF',
      targetPaceKph: 195
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_PASS_CLEAN',
      minClearanceM: 1.0,
      maxContactCount: 0,
      holdLeadTimeS: 1.0,
      allowOffTrack: false,
      description: 'AI executes inside threshold trail-braking dive into chicane apex without collision and secures lead.'
    })
  }),

  A3_SWITCHBACK_COUNTER: Object.freeze({
    id: 'A3_SWITCHBACK_COUNTER',
    name: 'North Esses Switchback Cutback',
    category: 'attack',
    trackSection: 'North Esses Complex',
    description: 'Approach to North Esses (distance ~1150m), player blocks inside entry and overslows, AI executes wide-entry late-apex cutback.',
    trackDistance: 1150,
    durationS: 18,
    playerConfig: Object.freeze({
      spec: 'gt',
      distance: 1160,
      lateralOffset: -2.6,
      initialSpeedKph: 130,
      initialSpeedMps: 130 / 3.6,
      ersMode: 'OFF',
      behavior: 'defensive_slow_apex'
    }),
    aiConfig: Object.freeze({
      spec: 'gt',
      distance: 1140,
      lateralOffset: 2.2,
      initialSpeedKph: 155,
      initialSpeedMps: 155 / 3.6,
      initialManeuver: 'CUTBACK',
      initialAggression: 0.86,
      ersMode: 'OFF',
      targetPaceKph: 175
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_PASS_CLEAN',
      minClearanceM: 1.1,
      maxContactCount: 0,
      holdLeadTimeS: 1.0,
      allowOffTrack: false,
      description: 'AI exploits player defensive pinch by geometric wide entry and late cutback underneath on exit.'
    })
  }),

  A4_OAKLAND_BOWL_SWEEP: Object.freeze({
    id: 'A4_OAKLAND_BOWL_SWEEP',
    name: 'Oakland Bowl High Banking Sweep',
    category: 'attack',
    trackSection: 'Oakland Bowl Banked Turn',
    description: 'Oakland Bowl high-speed turn (distance ~1800m), AI tests high-momentum outside sweep and banking carry.',
    trackDistance: 1800,
    durationS: 20,
    playerConfig: Object.freeze({
      spec: 'prototype',
      distance: 1820,
      lateralOffset: -2.2,
      initialSpeedKph: 175,
      initialSpeedMps: 175 / 3.6,
      ersMode: 'AUTO',
      behavior: 'tight_inside'
    }),
    aiConfig: Object.freeze({
      spec: 'prototype',
      distance: 1795,
      lateralOffset: 3.2,
      initialSpeedKph: 195,
      initialSpeedMps: 195 / 3.6,
      initialManeuver: 'OUTSIDE_SWEEP',
      initialAggression: 0.89,
      ersMode: 'ATTACK',
      targetPaceKph: 230
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_PASS_CLEAN',
      minClearanceM: 1.4,
      maxContactCount: 0,
      holdLeadTimeS: 1.0,
      allowOffTrack: false,
      description: 'AI uses banked corner camber and downforce to sweep around outside of player and carry momentum onto exit.'
    })
  }),

  A5_SLOW_OBSTACLE: Object.freeze({
    id: 'A5_SLOW_OBSTACLE',
    name: 'Slow Traffic Evasion & High-Speed Pass',
    category: 'attack',
    trackSection: 'Parkland Straight Evasion',
    description: 'Player moving slowly in middle/weaving at 80 km/h, AI at 190 km/h quickly recognizes safe passing corridor and executes instant clean evasion/pass.',
    trackDistance: 300,
    durationS: 12,
    playerConfig: Object.freeze({
      spec: 'touring',
      distance: 355,
      lateralOffset: 0.0,
      initialSpeedKph: 80,
      initialSpeedMps: 80 / 3.6,
      ersMode: 'OFF',
      behavior: 'slow_obstacle'
    }),
    aiConfig: Object.freeze({
      spec: 'prototype',
      distance: 280,
      lateralOffset: 0.0,
      initialSpeedKph: 190,
      initialSpeedMps: 190 / 3.6,
      initialManeuver: 'OBSTACLE_AVOIDANCE',
      initialAggression: 0.78,
      ersMode: 'AUTO',
      targetPaceKph: 220
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_PASS_CLEAN',
      minClearanceM: 1.5,
      maxContactCount: 0,
      holdLeadTimeS: 0.5,
      allowOffTrack: false,
      description: 'AI detects slow-moving vehicle ahead, chooses free lateral lane with zero latency, and executes clean high-speed evasion.'
    })
  }),

  D1_STRAIGHT_DEFENSE: Object.freeze({
    id: 'D1_STRAIGHT_DEFENSE',
    name: 'Main Straight Break-Tow & Inside Cover',
    category: 'defense',
    trackSection: 'Main Straight Braking Zone Approach',
    description: 'AI in front at 210 km/h, Player approaching fast behind at 235 km/h, AI breaks slipstream and covers inside defensive line approaching braking zone.',
    trackDistance: 500,
    durationS: 15,
    playerConfig: Object.freeze({
      spec: 'prototype',
      distance: 495,
      lateralOffset: 0.0,
      initialSpeedKph: 235,
      initialSpeedMps: 235 / 3.6,
      ersMode: 'ATTACK',
      behavior: 'attacking_charger'
    }),
    aiConfig: Object.freeze({
      spec: 'prototype',
      distance: 530,
      lateralOffset: 0.0,
      initialSpeedKph: 210,
      initialSpeedMps: 210 / 3.6,
      initialManeuver: 'DEFEND_INSIDE',
      initialAggression: 0.86,
      ersMode: 'AUTO',
      targetPaceKph: 240
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_HOLD_LEAD',
      minClearanceM: 1.0,
      maxContactCount: 0,
      allowOffTrack: false,
      description: 'AI moves off slipstream line, covers inside defensive lane approaching braking zone, and denies overtaking corridor.'
    })
  }),

  D2_CHICANE_DEFENSE: Object.freeze({
    id: 'D2_CHICANE_DEFENSE',
    name: 'Quarry Chicane Inside Line Defense',
    category: 'defense',
    trackSection: 'Quarry Chicane Entry',
    description: 'Approach to Chicane, AI in front takes defensive inside position, forcing player to long dirty outside.',
    trackDistance: 680,
    durationS: 15,
    playerConfig: Object.freeze({
      spec: 'gt',
      distance: 680,
      lateralOffset: 1.8,
      initialSpeedKph: 190,
      initialSpeedMps: 190 / 3.6,
      ersMode: 'OFF',
      behavior: 'outside_attacker'
    }),
    aiConfig: Object.freeze({
      spec: 'gt',
      distance: 702,
      lateralOffset: -2.4,
      initialSpeedKph: 170,
      initialSpeedMps: 170 / 3.6,
      initialManeuver: 'DEFEND_INSIDE',
      initialAggression: 0.88,
      ersMode: 'OFF',
      targetPaceKph: 185
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_HOLD_LEAD',
      minClearanceM: 1.0,
      maxContactCount: 0,
      allowOffTrack: false,
      description: 'AI holds committed inside defensive corridor through chicane braking and entry, forcing attacker wide and keeping position.'
    })
  }),

  D3_SWITCHBACK_DEFENSE: Object.freeze({
    id: 'D3_SWITCHBACK_DEFENSE',
    name: 'North Esses Switchback Defense',
    category: 'defense',
    trackSection: 'North Esses Exit',
    description: 'AI defends inside while anticipating player cutback on exit.',
    trackDistance: 1200,
    durationS: 18,
    playerConfig: Object.freeze({
      spec: 'gt',
      distance: 1195,
      lateralOffset: 2.2,
      initialSpeedKph: 160,
      initialSpeedMps: 160 / 3.6,
      ersMode: 'OFF',
      behavior: 'switchback_attacker'
    }),
    aiConfig: Object.freeze({
      spec: 'gt',
      distance: 1218,
      lateralOffset: -1.6,
      initialSpeedKph: 145,
      initialSpeedMps: 145 / 3.6,
      initialManeuver: 'DEFEND_CUTBACK',
      initialAggression: 0.85,
      ersMode: 'OFF',
      targetPaceKph: 165
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_HOLD_LEAD',
      minClearanceM: 1.0,
      maxContactCount: 0,
      allowOffTrack: false,
      description: 'AI guards entry apex then squares off corner exit early to block cutback line, retaining lead into next straight.'
    })
  }),

  D4_EXIT_DEFENSE: Object.freeze({
    id: 'D4_EXIT_DEFENSE',
    name: 'Oakland Exit Acceleration Defense',
    category: 'defense',
    trackSection: 'Oakland Bowl Exit Run',
    description: 'Corner exit acceleration battle, AI positions car to maximize drive while denying player room.',
    trackDistance: 1500,
    durationS: 16,
    playerConfig: Object.freeze({
      spec: 'gt',
      distance: 1500,
      lateralOffset: 2.2,
      initialSpeedKph: 135,
      initialSpeedMps: 135 / 3.6,
      ersMode: 'OFF',
      behavior: 'traction_attacker'
    }),
    aiConfig: Object.freeze({
      spec: 'gt',
      distance: 1516,
      lateralOffset: -0.6,
      initialSpeedKph: 130,
      initialSpeedMps: 130 / 3.6,
      initialManeuver: 'DEFEND_EXIT_DRIVE',
      initialAggression: 0.85,
      ersMode: 'OFF',
      targetPaceKph: 170
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'AI_HOLD_LEAD',
      minClearanceM: 1.0,
      maxContactCount: 0,
      allowOffTrack: false,
      description: 'AI positions vehicle on optimal traction exit arc while maintaining defensive lateral cushion, holding off drag race to next corner.'
    })
  }),

  H1_HOTLAP_PACING: Object.freeze({
    id: 'H1_HOTLAP_PACING',
    name: 'Solo Time Attack & Friction Circle Pacing',
    category: 'hotlap',
    trackSection: 'Full Circuit',
    description: 'Solo AI testing maximum speed profile, trail braking, friction circle utilization, and curb cutting.',
    trackDistance: 0,
    durationS: 90,
    playerConfig: null,
    aiConfig: Object.freeze({
      spec: 'prototype',
      distance: 0,
      lateralOffset: 0.0,
      initialSpeedKph: 220,
      initialSpeedMps: 220 / 3.6,
      initialManeuver: 'PACE_HOTLAP',
      initialAggression: 1.0,
      ersMode: 'AUTO',
      targetPaceKph: 275
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'LAP_COMPLETED',
      maxLapTimeS: 65.0,
      minFrictionCircleUtilisation: 0.85,
      maxContactCount: 0,
      allowOffTrack: false,
      description: 'Solo hotlap execution testing trail braking, tire friction envelope, curbs utilization, and aerodynamic stability.'
    })
  }),

  FREE_DUEL: Object.freeze({
    id: 'FREE_DUEL',
    name: 'Endurance Park Free Duel',
    category: 'duel',
    trackSection: 'Full Circuit Grid',
    description: 'Open track free battle starting from grid or moving start.',
    trackDistance: 0,
    durationS: 300,
    playerConfig: Object.freeze({
      spec: 'gt',
      distance: 0,
      lateralOffset: 2.65,
      initialSpeedKph: 0,
      initialSpeedMps: 0,
      ersMode: 'OFF',
      behavior: 'human'
    }),
    aiConfig: Object.freeze({
      spec: 'gt',
      distance: 8.8,
      lateralOffset: -2.65,
      initialSpeedKph: 0,
      initialSpeedMps: 0,
      initialManeuver: 'RACE',
      initialAggression: 0.82,
      ersMode: 'OFF',
      targetPaceKph: 220
    }),
    successCriteria: Object.freeze({
      targetOutcome: 'RACE_FINISH',
      maxContactCount: 4,
      allowOffTrack: true,
      description: 'Multi-lap full battle across Endurance Park with dynamic tactics, passing, defense, and drafting.'
    })
  })
});

export const SCENARIO_LIST = Object.freeze(Object.values(SCENARIOS));

export const SCENARIOS_BY_ID = Object.freeze(
  new Map(SCENARIO_LIST.map((scenario) => [scenario.id, scenario]))
);

export function getScenarioById(id) {
  if (!id) return null;
  if (SCENARIOS_BY_ID.has(id)) return SCENARIOS_BY_ID.get(id);
  const normalized = String(id).toUpperCase();
  for (const scenario of SCENARIO_LIST) {
    if (scenario.id === normalized) return scenario;
    if (scenario.id.startsWith(`${normalized}_`) || (normalized === 'FREE' && scenario.id === 'FREE_DUEL')) {
      return scenario;
    }
  }
  return null;
}

export function getScenariosByCategory(category) {
  if (!category) return [];
  const normalized = String(category).toLowerCase();
  return SCENARIO_LIST.filter((s) => s.category.toLowerCase() === normalized);
}
