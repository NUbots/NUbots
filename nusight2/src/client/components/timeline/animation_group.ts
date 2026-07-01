function lerp(start: bigint, end: bigint, percent: number) {
  return start + BigInt(Math.floor(percent * Number(end - start)));
}

/**
 * Options for animating a group of values in sync.
 * Generic enforces a fixed length for the number of values to animate simultaneously.
 */
export interface AnimationGroupOpts<T extends [...bigint[]]> {
  duration: number;
  startValues: [...T];
  endValues: [...T];
  onUpdate: (values: T) => void;
  onEnd: () => void;
}

export interface AnimationGroup<T extends [...bigint[]]> {
  startValues: [...T];
  endValues: [...T];
  /** Cancel the animation at the current duration */
  cancel: () => void;
}

/**
 * Animates a set of values to another set of values linearly from one set of values to another
 * over a given duration.
 *
 * Calls the `onUpdate` callback with the current animation values for each frame until the animation
 * is complete.
 */
export function animateValues<T extends [...bigint[]]>(opts: AnimationGroupOpts<T>): AnimationGroup<T> {
  const { duration, startValues, endValues, onUpdate, onEnd } = opts;

  const startTime = Date.now();
  let animationHandle: number | undefined;

  const update = () => {
    const elapsedPercent = (Date.now() - startTime) / duration;

    // Exit condition
    if (elapsedPercent >= 1) {
      onUpdate(opts.endValues);
      onEnd();
      return;
    }

    onUpdate(startValues.map((val, i) => lerp(val, endValues[i], elapsedPercent)) as T);
    animationHandle = requestAnimationFrame(update);
  };

  const cancel = () => {
    onEnd();
    if (animationHandle !== undefined) {
      cancelAnimationFrame(animationHandle);
    }
  };

  update();
  return { startValues, endValues, cancel };
}
