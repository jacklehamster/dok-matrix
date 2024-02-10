import { NumVal } from "progressive-value";
import { IMatrix } from "./IMatrix";

export interface IAngleMatrix extends IMatrix {
  get angle(): NumVal;
}
