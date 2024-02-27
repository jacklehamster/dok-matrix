import { Vector } from "dok-types";
import { IMatrix } from "../IMatrix";
import { VectorPool } from "dok-types";

export class PositionUtils {
  static toVector(x: number, y: number, z: number, vector: Vector): Vector {
    vector[0] = x;
    vector[1] = y;
    vector[2] = z;
    return vector;
  }

  static transformToPosition(transform: IMatrix, pos: Vector) {
    const m = transform.getMatrix();
    pos[0] = m[12]; // Value in the 4th column, 1st row (indices start from 0)
    pos[1] = m[13]; // Value in the 4th column, 2nd row
    pos[2] = m[14]; // Value in the 4th column, 3rd row
  }
}
