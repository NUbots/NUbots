import { observe } from 'mobx'
import { observable } from 'mobx'

import { createUpdatableComputed } from '../create_updatable_computed'

describe('createUpdatableComputed', () => {
  let model: Model
  let viewModel: ViewModel
  let onChange: jest.Mock

  const computedTriangle = createUpdatableComputed(
    (opts: TriangleOpts) => new Triangle(opts),
    (triangle, { color }) => triangle.color = color,
    triangle => triangle.dispose(),
  )

  class Model {
    @observable.ref color: string

    constructor({ color }: { color: string }) {
      this.color = color
    }
  }

  class ViewModel {
    constructor(
      private readonly model: { color: string },
    ) {
    }

    readonly triangle = computedTriangle(() => ({
      color: this.model.color,
    }))
  }

  type TriangleOpts = { color: string }

  class Triangle {
    color: string
    disposed: boolean = false

    constructor({ color }: TriangleOpts) {
      this.color = color
    }

    dispose() {
      this.disposed = true
    }
  }

  beforeEach(() => {
    model = new Model({ color: 'red' })
    viewModel = new ViewModel(model)
    onChange = jest.fn()
  })

  it('returns computed value', () => {
    const triangle = viewModel.triangle.get()
    expect(triangle).toBeInstanceOf(Triangle)
    expect(triangle.color).toBe('red')
  })

  it('caches computed value', () => {
    expect(viewModel.triangle.get()).toBe(viewModel.triangle.get())
  })

  it('maintains reference as its updated', () => {
    const dispose = observe(viewModel.triangle, onChange)

    const firstTriangle = viewModel.triangle.get()
    model.color = 'green'
    expect(firstTriangle.color).toBe('green')

    const secondTriangle = viewModel.triangle.get()
    model.color = 'blue'
    expect(secondTriangle.color).toBe('blue')

    expect(secondTriangle).toBe(firstTriangle)
    expect(onChange).toBeCalledTimes(2)
    dispose()
  })

  it('does not dispose when updated', () => {
    const dispose = observe(viewModel.triangle, onChange)
    const triangle = viewModel.triangle.get()
    model.color = 'green'
    expect(triangle.color).toBe('green')
    expect(triangle.disposed).toBe(false)
    expect(onChange).toBeCalledTimes(1)
    dispose()
  })

  it('disposes when no longer observed', () => {
    const dispose = observe(viewModel.triangle, onChange)
    const triangle = viewModel.triangle.get()
    dispose()
    expect(triangle.disposed).toBe(true)
  })
})
