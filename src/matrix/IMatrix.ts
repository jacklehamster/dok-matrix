import { IChangeNotifier } from "change-listener";

export interface IMatrix extends IChangeNotifier<IMatrix> {
  getMatrix(): Float32Array;
}
