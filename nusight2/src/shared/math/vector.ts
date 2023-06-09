export abstract class Vector {
  /**
   * Returns a copy of this vector.
   */
  abstract copy(): Vector;

  /**
   * Return a new vector where all the values of this vector are multiplied by a scalar value
   *
   * @param scalar  Number to multiply each value of the vector by
   * @returns       New vector where the values are this vector's values multiplied by the
   *                scalar value.
   */
  abstract multiplyScalar(scalar: number): Vector;

  /**
   * Return a new vector where all the values of this vector are divided by a scalar value
   *
   * @param scalar  Number to divide each value of the vector by
   * @returns       New vector where the values are this vector's values divided by the
   *                scalar value.
   */
  abstract divideScalar(scalar: number): Vector;

  /**
   * Return new vector where each component of the given vector are added to the
   * equivalent component of this vector.
   *
   * @param v Vector to add to this Vector
   * @returns New vector where the components of each vector have been added
   */
  abstract add(v: Vector): Vector;

  /**
   * Return new vector where each component of the given vector are subtracted from the
   * equivalent component of this vector.
   *
   * @param v Vector to add to this Vector
   * @returns New vector where the components of the given vector are subtracted from
   *          this vector's components
   */
  abstract subtract(v: Vector): Vector;

  /**
   * Dot product between this vector and another vector.
   *
   * @param v Another vector
   */
  abstract dot(v: Vector): number;

  /**
   * The length of this vector
   */
  get length(): number {
    return Math.sqrt(this.dot(this));
  }

  /**
   * Get the normalised vector for this vector, with the same direction as this vector, but
   * with a length of 1.
   *
   * @returns Normalised vector of this vector
   */
  normalize<T extends Vector>(this: T): T {
    return this.divideScalar(this.length) as T;
  }

  /**
   * Linear interpolation from this vector to another vector.
   *
   * @param v Other vector with the same number of dimensions
   * @param t Normalised interpolation value where 0 returns this vector, 1 returns the other vector,
   *          and a value between returns a vector on the line between them
   *
   * @returns A new vector linearly interpolated between each vector
   */
  lerpTo<T extends Vector>(this: T, v: T, t: number): T {
    return this.add(v.subtract(this).multiplyScalar(t)) as T;
  }

  /**
   * Angle from this vector to another vector.
   *
   * @details Based on the implementation found here https://www.plunk.org/~hatch/rightway.html
   *
   * @param v Other vector with the same number of dimensions
   *
   * @returns The angle in radians between this vector and another vector.
   */
  angleTo<T extends Vector>(this: T, v: T): number {
    return this.dot(v) < 0.0
      ? Math.PI - 2 * Math.asin(v.multiplyScalar(-1).subtract(this).length / 2)
      : 2 * Math.asin(v.subtract(this).length / 2);
  }

  /**
   * Spherical linear interpolation from this vector to another vector.
   *
   * @details Based on the implementation found here https://www.plunk.org/~hatch/rightway.html
   *
   * @param v Other vector with the same number of dimensions
   * @param t Normalised interpolation value where 0 returns this vector, 1 returns the other vector,
   *          and a value between returns a vector rotated part of the way from this vector to the
   *          other vector
   *
   * @returns A new vector rotated part of the way from this vector to another vector
   */
  slerpTo<T extends Vector>(this: T, v: T, t: number): T {
    const angle = this.angleTo(v);

    if (angle === 0) {
      return this.copy() as T;
    }

    /**
     * Stable version of 'Math.sin(x) / x'
     * @details Based on the implementation found here https://www.plunk.org/~hatch/rightway.html
     */
    const sinOverX = (x: number): number => {
      return 1.0 + x * x * (1.0 / 6.0) === 1 ? 1.0 : Math.sin(x) / x;
    };

    const sinOverXAngle = sinOverX(angle);
    const t2 = 1 - t;

    const v0 = this.multiplyScalar(t2 * (sinOverX(t2 * angle) / sinOverXAngle));
    const v1 = v.multiplyScalar(t * (sinOverX(t * angle) / sinOverXAngle));
    return v0.add(v1) as T;
  }
}
