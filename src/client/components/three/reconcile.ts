import { Object3D } from 'three'
import { Texture } from 'three'

import { Stage } from './three'

/**
 * This function deeply copies all values from `source` into `target`.
 *
 * It implements a three.js specific reconciliation algorithm between two stage objects. Where possible, it will prefer
 * to update objects rather than replacing them, thus maintain reference identity for most objects.
 *
 * Note: Assumes that any object reference that is in both `source` and `target` are deeply equal to save computation.
 *
 * Ref: https://reactjs.org/docs/reconciliation.html
 */
export function reconcile(source: Stage, target: Stage) {
  fixObjectAdd()
  if (source.camera.type === target.camera.type) {
    reconcileObjects(source.camera, target.camera)
  } else {
    target.camera = source.camera
  }
  reconcileObjects(source.scene, target.scene)
}

export function reconcileObjects(source: any, target: any, visited = new Set()): void {
  if (target === source) {
    return
  }
  if (visited.has(source)) {
    throw new Error('cycle detected')
  }
  visited.add(source)
  if (source && !target) {
    throw new Error('Cannot copy into an empty target')
  }
  if (target !== source && target instanceof Texture) {
    target.needsUpdate = true
  }
  // tslint:disable-next-line forin
  for (const key in source) {
    const sourceValue: any = source[key]
    const targetValue: any = target[key]
    if (key === 'parent') {
      continue
    } else if (sourceValue === targetValue) {
      // nothing changed, move on
    } else if (targetValue == null) {
      target[key] = sourceValue
    } else if (typeof sourceValue === 'function') {
      target[key] = sourceValue
    } else if (Array.isArray(sourceValue)) {
      reconcileArrays(sourceValue, targetValue, visited)
    } else if (sourceValue == null) {
      target[key] = sourceValue
    } else if (typeof sourceValue === 'object') {
      if (sourceValue.constructor !== targetValue.constructor) {
        target[key] = sourceValue
      } else {
        reconcileObjects(sourceValue, targetValue, visited)
      }
    } else if (key !== 'uuid' && key !== 'id' && key !== 'version') {
      target[key] = sourceValue
    }
  }
}

export function reconcileArrays(source: any[], target: any[], visited = new Set()): void {
  if (target === source) {
    return
  }
  if (target.length !== source.length) {
    target.length = source.length
  }
  for (const [index, sourceChild] of source.entries()) {
    if (typeof sourceChild === 'object') {
      const targetChild: any = target[index]
      if (targetChild == null) {
        target[index] = sourceChild
      } else if (targetChild === sourceChild) {
        // nothing changed, move on
        continue
      } else if (Array.isArray(sourceChild)) {
        reconcileArrays(sourceChild, targetChild, visited)
      } else if (typeof targetChild === 'object') {
        reconcileObjects(sourceChild, targetChild, visited)
      } else {
        target[index] = sourceChild
      }
    } else {
      target[index] = sourceChild
    }
  }
}

export function fixObjectAdd() {
  if (Object3D.prototype.add !== objectAdd) {
    Object3D.prototype.add = objectAdd
  }
}

export function objectAdd(this: Object3D, ...children: Object3D[]) {
  for (const child of children) {
    this.children.push(child)
    if (child.parent == null) {
      child.parent = this
    }
  }
}
