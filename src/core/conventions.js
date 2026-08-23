import { clamp } from './math.js';

// Three.js renders this simulator's +Z-forward car convention from a camera on -Z.
// Positive screen-axis input means a player asks to turn right; the matching physics-X
// direction is negative so that the turn appears on the right side of the screen.
export function playerSteerFromScreenAxis(screenAxis) {
  return -clamp(screenAxis, -1, 1);
}
