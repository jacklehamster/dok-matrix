import { IChangeListener } from "change-listener";
import { IMatrix } from "./IMatrix";
import Matrix from "./Matrix";
import { Active } from "dok-types";

export class InvertMatrix extends Matrix implements IMatrix, Active {
  readonly #matrix: IMatrix;
  readonly #listener: IChangeListener<IMatrix> = {
    onChange: (elem) => {
      this.invert(elem);
    },
  };
  constructor(matrix: IMatrix, activate: boolean = true) {
    super();
    this.#matrix = matrix;
    if (activate) {
      this.activate();
    }
  }

  activate() {
    this.#matrix.addChangeListener(this.#listener);
  }

  deactivate() {
    this.#matrix.removeChangeListener(this.#listener);
  }
}
