import { ICollisionDetector } from "./matrix/collision/ICollisionDetector";
import { Angle, angleStep } from "./matrix/utils/angle-utils";
import { createTurnMatrix, createTiltMatrix } from "./matrix/utils/angle-matrix-utils";
import { IAngleMatrix } from "./matrix/IAngleMatrix";
import { IMatrix } from "./matrix/IMatrix";
import { PositionMatrix } from "./matrix/PositionMatrix";
import { IPositionMatrix, MoveResult } from "./matrix/IPositionMatrix";
import { ProjectionMatrix } from "./matrix/ProjectionMatrix";
import Matrix from "./matrix/Matrix";
import { transformToPosition, toVector } from "./matrix/utils/position-utils";

export type { ICollisionDetector, Angle, IAngleMatrix, IMatrix, IPositionMatrix };
export { angleStep, createTurnMatrix, createTiltMatrix, PositionMatrix, ProjectionMatrix, Matrix, MoveResult, transformToPosition, toVector };
