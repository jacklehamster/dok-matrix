import { List } from "abstract-list";
import { IMatrix } from "./IMatrix";
import { ICollisionDetector } from "./collision/ICollisionDetector";
import { Vector } from "dok-types";

export enum MoveResult {
  AT_POSITION = 0,
  MOVED = 1,
  BLOCKED = 2
}

export type ChangeListener = (dx: number, dy: number, dz: number) => void;

export interface IPositionMatrix extends IMatrix {
  moveBy(x: number, y: number, z: number, turnMatrix?: IMatrix): MoveResult;
  moveTo(x: number, y: number, z: number): MoveResult;
  gotoPos(x: number, y: number, z: number, speed?: number): MoveResult;
  movedTo(x: number, y: number, z: number): this;
  onChange(listener: ChangeListener): this;
  removeChangeListener(listener: ChangeListener): void;
  get position(): Vector;
  blockers?: List<ICollisionDetector>;
}
