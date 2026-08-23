export const TAU = Math.PI * 2;

export const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
export const saturate = (value) => clamp(value, 0, 1);
export const lerp = (a, b, t) => a + (b - a) * t;
export const damp = (current, target, lambda, dt) => lerp(current, target, 1 - Math.exp(-lambda * dt));
export const wrap = (value, length) => ((value % length) + length) % length;
export const wrapAngle = (angle) => {
  let result = (angle + Math.PI) % TAU;
  if (result < 0) result += TAU;
  return result - Math.PI;
};

export function length2(x, z) {
  return Math.hypot(x, z);
}

export function normalize2(x, z) {
  const length = Math.hypot(x, z) || 1;
  return { x: x / length, z: z / length };
}

export function dot2(ax, az, bx, bz) {
  return ax * bx + az * bz;
}

export function cross2(ax, az, bx, bz) {
  return ax * bz - az * bx;
}

export function localToWorld(x, z, yaw) {
  const s = Math.sin(yaw);
  const c = Math.cos(yaw);
  return { x: x * c + z * s, z: -x * s + z * c };
}

export function worldToLocal(x, z, yaw) {
  const s = Math.sin(yaw);
  const c = Math.cos(yaw);
  return { x: x * c - z * s, z: x * s + z * c };
}

export function smoothstep(edge0, edge1, value) {
  const span = Math.abs(edge1 - edge0);
  const denom = span < 1e-7 ? (edge1 >= edge0 ? 1e-7 : -1e-7) : (edge1 - edge0);
  const t = saturate((value - edge0) / denom);
  return t * t * (3 - 2 * t);
}

export function hash01(seed) {
  const x = Math.sin(seed * 127.1 + 311.7) * 43758.5453123;
  return x - Math.floor(x);
}
