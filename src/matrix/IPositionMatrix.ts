import { List } from "abstract-list";
import { IMatrix } from "./IMatrix";
import { ICollisionDetector } from "./collision/ICollisionDetector";
import { Vector } from "dok-types";

export enum MoveResult {
  AT_POSITION = 0,
  MOVED = 1,
  BLOCKED = 2
}

export interface IPositionMatrix extends IMatrix {
  moveBy(x: number, y: number, z: number, turnMatrix?: IMatrix): MoveResult;
  moveTo(x: number, y: number, z: number): MoveResult;
  moveTowards(x: number, y: number, z: number, speed?: number): MoveResult;
  attemptMoveTowards(x: number, y: number, z: number, speed?: number): MoveResult;
  movedTo(x: number, y: number, z: number): this;
  get position(): Vector;
  blockers?: List<ICollisionDetector>;
}
