export class Quaternion {
  constructor(readonly x: number, readonly y: number, readonly z: number, readonly w: number) {}

  static of() {
    return new Quaternion(0, 0, 0, 1)
  }
}
