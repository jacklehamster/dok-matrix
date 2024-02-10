export type Angle = number;

export function angleStep(angle: Angle, step: Angle) {
  return Math.round(angle / step) * step;
}
