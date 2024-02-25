import { ChangeNotifier, IChangeListener } from "change-listener";
import { IMatrix } from "./IMatrix";
import Matrix from "./Matrix";
import { NumVal } from "progressive-value";

const DEFAULT_PERSPECTIVE_LEVEL = 1;
const DEFAULT_ZOOM = 1;

export class ProjectionMatrix implements IMatrix {
  readonly #baseMatrix = Matrix.create();
  readonly #perspectiveMatrix = Matrix.create();
  readonly #orthoMatrix = Matrix.create();
  readonly #size: [number, number] = [0, 0];
  readonly #changeNotifier = new ChangeNotifier(this);
  readonly perspective: NumVal;
  readonly zoom: NumVal;

  constructor() {
    const onChangeProjection = {
      onChange: () => {
        this.#baseMatrix.combine(this.#orthoMatrix, this.#perspectiveMatrix, this.perspective.valueOf());
      }
    };
    this.perspective = new NumVal(DEFAULT_PERSPECTIVE_LEVEL, onChangeProjection.onChange);
    this.zoom = new NumVal(DEFAULT_ZOOM, zoom => {
      this.configure(this.#size, zoom);
    });
    this.#perspectiveMatrix.addChangeListener(onChangeProjection);
    this.#orthoMatrix.addChangeListener(onChangeProjection);
    this.#baseMatrix.addChangeListener(this.#changeNotifier);
  }

  addChangeListener(listener: IChangeListener<IMatrix>): this {
    this.#changeNotifier.addChangeListener(listener);
    return this;
  }

  removeChangeListener(listener: IChangeListener<IMatrix>): void {
    this.#changeNotifier.removeChangeListener(listener);
  }

  private configPerspectiveMatrix(angle: number, ratio: number, near: number, far: number) {
    this.#perspectiveMatrix.perspective(angle, ratio, near, far);
  }

  private configOrthoMatrix(width: number, height: number, near: number, far: number) {
    this.#orthoMatrix.ortho(-width / 2, width / 2, -height / 2, height / 2, near, far);
  }

  configure(size: [number, number], zoom?: number, near = 0.5, far = 10000) {
    if (!zoom) {
      zoom = this.zoom.valueOf();
    }
    this.#size[0] = size[0];
    this.#size[1] = size[1];
    const ratio: number = this.#size[0] / this.#size[1];
    const angle = 45 / Math.sqrt(zoom);
    this.configPerspectiveMatrix(angle, ratio, Math.max(near, 0.00001), far);
    this.configOrthoMatrix(ratio / zoom / zoom, 1 / zoom / zoom, -far, far);
  }

  getMatrix(): Float32Array {
    return this.#baseMatrix.getMatrix();
  }
}
