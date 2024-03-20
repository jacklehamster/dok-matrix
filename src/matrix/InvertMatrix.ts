import { IChangeListener } from "change-listener";
import { IMatrix } from "./IMatrix";
import Matrix from "./Matrix";
import { Active } from "dok-types";

export class InvertMatrix extends Matrix implements IMatrix, Active {
  readonly #listener: IChangeListener<IMatrix> = {
    onChange: (elem) => {
      this.invert(elem);
    },
  };
  constructor(activate: boolean = true) {
    super();
    if (activate) {
      this.activate();
    }
  }

  activate() {
    this.addChangeListener(this.#listener);
  }

  deactivate() {
    this.removeChangeListener(this.#listener);
  }
}
