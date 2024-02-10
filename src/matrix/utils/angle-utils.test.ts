import { angleStep } from './angle-utils';

describe('angleStep function', () => {
  it('should round angle to the nearest step', () => {
    expect(angleStep(Math.PI, Math.PI / 2)).toBeCloseTo(Math.PI); // Nearest step is π
    expect(angleStep(1.5 * Math.PI, Math.PI / 2)).toBeCloseTo(1.5 * Math.PI); // Nearest step is 1.5π
    expect(angleStep(0.9 * Math.PI, Math.PI / 2)).toBeCloseTo(Math.PI); // Nearest step is π
    expect(angleStep(0.1 * Math.PI, Math.PI / 2)).toBeCloseTo(0); // Nearest step is 0
  });

  it('should handle step of 0', () => {
    expect(angleStep(Math.PI, 0)).toBeNaN(); // Step of 0 should return NaN
    expect(angleStep(0, 0)).toBeNaN(); // Step of 0 should return NaN
  });
});
