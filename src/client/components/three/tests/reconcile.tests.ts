import { MeshPhongMaterial } from 'three'
import { SpotLight } from 'three'
import { MeshLambertMaterial } from 'three'
import { PointLight } from 'three'
import { Color } from 'three'
import { Vector3 } from 'three'
import { Texture } from 'three'
import { CircleGeometry } from 'three'
import { ConeGeometry } from 'three'
import { MeshBasicMaterial } from 'three'
import { BoxGeometry } from 'three'
import { Mesh } from 'three'
import { OrthographicCamera } from 'three'
import { Scene } from 'three'
import { Object3D } from 'three'
import { PerspectiveCamera } from 'three'

import { objectAdd } from '../reconcile'
import { fixObjectAdd } from '../reconcile'
import { reconcileArrays } from '../reconcile'
import { reconcileObjects } from '../reconcile'
import { reconcile } from '../reconcile'

describe('reconcile', () => {
  it('reconciles a stage', () => {
    const target = {
      camera: new OrthographicCamera(-1, 1, 1, -1),
      scene: sceneWith([
        groupWith([
          new Mesh(new CircleGeometry(10), new MeshPhongMaterial({ color: 'red' })),
        ]),
        new OrthographicCamera(-1, 1, 1, -1),
        groupWith([
          new SpotLight(),
        ]),
      ]),
    }
    const source = {
      camera: new PerspectiveCamera(),
      scene: sceneWith([
        new Mesh(new BoxGeometry(1, 1, 1), new MeshBasicMaterial({ color: 'red' })),
        new Mesh(new CircleGeometry(50), new MeshLambertMaterial({ color: 'blue' })),
        groupWith([
          new Mesh(new ConeGeometry(), new MeshBasicMaterial({ color: 'green' })),
        ]),
        groupWith([
          new PointLight(),
        ]),
      ]),
    }
    reconcile(source, target)
    expect(target.scene).toMatchObject({
      children: [
        {
          type: 'Mesh',
          geometry: { type: 'BoxGeometry' },
          material: { type: 'MeshBasicMaterial', color: new Color('red') },
        },
        {
          type: 'Mesh',
          geometry: { type: 'CircleGeometry' },
          material: { type: 'MeshLambertMaterial', color: new Color('blue') },
        },
        {
          children: [
            {
              type: 'Mesh',
              geometry: { type: 'ConeGeometry' },
              material: { type: 'MeshBasicMaterial', color: new Color('green') },
            },
          ],
        },
        { children: [{ type: 'PointLight' }] },
      ],
    })
    expect(target.scene).not.toBe(source.scene)
    expect(target.camera).toBe(source.camera)
  })
})

describe('reconcileObjects', () => {
  function expectReconcileEqual(source: any, target: any) {
    reconcileObjects(source, target)
    expect(target).toEqual(source)
    expect(target).not.toBe(source)
  }

  it('reconciles primitive values', () => {
    const target = { a: 'bar', b: 2, c: false, d: null, e: 'abc' }
    const source = { a: 'foo', b: 1, c: true, d: 123, e: undefined }
    expectReconcileEqual(source, target)
  })

  it('does not delete removed fields', () => {
    // Note this behaviour is not required, but we have no current need for it.
    const target = { foo: 'bar' }
    const source = { abc: 123 }
    reconcileObjects(source, target)
    expect(target).toEqual({ foo: 'bar', abc: 123 })
  })

  it('reconciles deeply', () => {
    const target = { a: { foo: 'bar' }, b: { abc: 123 } }
    const source = { a: { foo: 'baz' }, b: { abc: 456 } }
    expectReconcileEqual(source, target)
    expect(target.a).not.toBe(source.a)
    expect(target.b).not.toBe(source.b)
  })

  it('throws on an empty target', () => {
    const source = { foo: 'bar' }
    expect(() => reconcileObjects(source, null)).toThrow(/empty target/)
    expect(() => reconcileObjects(source, undefined)).toThrow(/empty target/)
  })

  describe('three.js', () => {
    it('reconciles vector objects', () => {
      const target = new Vector3(4, 5, 6)
      const source = new Vector3(1, 2, 3)
      reconcileObjects(source, target)
      expect(target).toMatchObject({ x: 1, y: 2, z: 3 })
    })

    it('reconciles mesh objects', () => {
      const target = new Mesh(
        new CircleGeometry(50),
        new MeshPhongMaterial({ color: 'blue' }),
      )
      const source = new Mesh(
        new BoxGeometry(1, 1, 1),
        new MeshBasicMaterial({ color: 'red' }),
      )
      reconcileObjects(source, target)
      expect(target).toMatchObject({
        geometry: { type: 'BoxGeometry' },
        material: { type: 'MeshBasicMaterial', color: new Color('red') },
      })
    })

    it('does not reconsile the fields id, uuid and version', () => {
      const target = new Texture()
      const source = new Texture()
      reconcileObjects(source, target)
      expect(source.id).not.toEqual(target.id)
      expect(source.uuid).not.toEqual(target.uuid)
      expect(source.version).not.toEqual(target.version)
    })

    it('increments Texture versions', () => {
      const target = new Texture()
      const source = new Texture()
      expect(target.version).toEqual(0)
      reconcileObjects(source, target)
      expect(target.version).toEqual(1)
    })

    it('skips parent keys', () => {
      const target = { parent: 'bar' }
      const source = { parent: 'foo' }
      reconcileObjects(source, target)
      expect(target.parent).toBe('bar')
    })

    it('throws on cycles', () => {
      const target = { next: { next: {} } }
      const source = { next: { next: {} } }
      const b = { next: source }
      source.next = b
      expect(() => reconcileObjects(source, target)).toThrow(/cycle detected/)
    })
  })
})

describe('reconcileArrays', () => {
  function expectReconcileEqual(source: any, target: any) {
    reconcileArrays(source, target)
    expect(target).toEqual(source)
    expect(target).not.toBe(source)
  }

  it('reconciles primitive values', () => {
    const target = [1, null, true, 'foo']
    const source = ['bar', undefined, false, 2]
    expectReconcileEqual(source, target)
  })

  it('reconciles object values', () => {
    const target = [{ foo: 'bar' }, { abc: 123 }]
    const source = [{ foo: 456 }, { abc: 'baz' }]
    expectReconcileEqual(source, target)
  })

  it('reconciles array values', () => {
    const target = [['bar'], [123]]
    const source = [[456], ['baz']]
    expectReconcileEqual(source, target)
  })

  it('expands to larger source', () => {
    const target = [1]
    const source = [1, 2, 3, 4, 5]
    expectReconcileEqual(source, target)
  })

  it('shrinks to smaller source', () => {
    const target = [1, 2, 3, 4, 5]
    const source = [1]
    expectReconcileEqual(source, target)
  })

  it('changes types', () => {
    const target = [1, 2, 3, 4, 5]
    const source = [{ foo: 'bar' }]
    expectReconcileEqual(source, target)
  })

  it('reconciles empty target values', () => {
    const target = [undefined]
    const source = [{ foo: 'bar' }]
    expectReconcileEqual(source, target)
  })
})

describe('fixObjectAdd', () => {
  beforeEach(() => {
    fixObjectAdd()
  })

  it('overrides Object3D prototype', () => {
    expect(Object3D.prototype.add).toBe(objectAdd)
  })

  it('allows objects to be added to multiple parents', () => {
    const a = new Object3D()
    const b = new Object3D()
    b.add(a)
    const c = new Object3D()
    c.add(a)
    expect(b.children[0]).toBe(a)
    expect(c.children[0]).toBe(a)
  })

  it('only sets parent once', () => {
    const a = new Object3D()
    const b = new Object3D()
    const c = new Object3D()
    b.add(a)
    c.add(a)
    expect(a.parent).toBe(b)
  })
})

function sceneWith(children: Object3D[]): Scene {
  const scene = new Scene()
  children.length && scene.add(...children)
  return scene
}

function groupWith(children: Object3D[]): Object3D {
  const scene = new Object3D()
  children.length && scene.add(...children)
  return scene
}
