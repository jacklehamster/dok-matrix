import { Vector } from "dok-types";
import { IMatrix } from "../IMatrix";
import { IMotor, Cycle, UpdatePayload } from "motor-loop";
import { VectorPool } from "dok-types";

interface Props extends Partial<Data> {
  motor: IMotor;
}

interface Data {
  vectorPool: VectorPool;
}

export class PositionUtils implements Cycle<Data> {
  private readonly vectorPool: VectorPool;
  private readonly motor;
  private readonly data;

  constructor({ motor, vectorPool }: Props) {
    this.motor = motor;
    this.vectorPool = vectorPool ?? new VectorPool();
    this.data = { vectorPool: this.vectorPool };
  }

  refresh({ data: { vectorPool } }: UpdatePayload<Data>): void {
    vectorPool.recycleAll();
  }

  transformToPosition(transform: IMatrix) {
    this.motor.scheduleUpdate(this, this.data);
    const m = transform.getMatrix();
    return this.vectorPool.create(m[12], m[13], m[14]);
  }

  toVector(x: number, y: number, z: number): Vector {
    this.motor.scheduleUpdate(this, this.data);
    return this.vectorPool.create(x, y, z);
  }

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
