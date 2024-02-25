import { Angle } from "./angle-utils";
import { IAngleMatrix } from "../IAngleMatrix";
import Matrix from "../Matrix";
import { NumVal } from "progressive-value";
import { IMatrix } from "../IMatrix";
import { IChangeListener } from "change-listener";

function createAngleMatrix(applyAngle: (matrix: Matrix, angle: Angle) => void, onChange?: () => void): IAngleMatrix {
  const matrix = Matrix.create();
  const listeners = new Set<IChangeListener<IMatrix>>();
  if (onChange) {
    listeners.add({ onChange });
  }
  const angleMatrix: IAngleMatrix = {
    angle: new NumVal(0, tilt => {
      applyAngle(matrix, tilt);
      for (const listener of listeners) {
        listener.onChange(angleMatrix);
      }
    }),
    getMatrix() {
      return matrix.getMatrix();
    },
    addChangeListener(listener) {
      listeners.add(listener);
      return this;
    },
    removeChangeListener(listener) {
      listeners.delete(listener);
    },
  };
  return angleMatrix;
}

export function createTurnMatrix(onChange?: () => void): IAngleMatrix {
  return createAngleMatrix(
    (matrix, angle) => matrix.setYRotation(angle),
    onChange);
}
export function createTiltMatrix(onChange?: () => void): IAngleMatrix {
  return createAngleMatrix(
    (matrix, angle) => matrix.setXRotation(angle),
    onChange);
}
