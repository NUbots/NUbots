import { IComputedValue } from 'mobx'
import { observable } from 'mobx'
import { autorun } from 'mobx'

import { disposableComputed } from '../disposable_computed'

describe('disposableComputed', () => {
  let model: { a: number, b: number }
  let expr: IComputedValue<{ sum: number, dispose: jest.Mock<() => void> }>

  beforeEach(() => {
    model = observable({ a: 1, b: 1 })
    expr = disposableComputed(() => ({ sum: model.a + model.b, dispose: jest.fn<() => void>() }))
  })

  const countUnique = <T extends unknown>(arr: T[]): number => new Set(arr).size

  it('returns value on evaluation', () => {
    const value = { foo: 'bar', dispose: jest.fn() }
    const expr = disposableComputed(() => value)
    expect(expr.get()).toBe(value)
  })

  describe('when unobserved', () => {
    it('creates new values when repeatably evaluated', () => {
      const someValues = Array.from({ length: 5 }, () => expr.get())
      expect(countUnique(someValues)).toBe(5)
    })

    it('does not dispose value after evaluation', () => {
      const value = expr.get()
      expect(value.dispose).not.toHaveBeenCalled()
    })

    it('disposes stale values when repeatably evaluated', () => {
      const someValues = Array.from({ length: 5 }, () => expr.get())
      const allButLast = someValues.slice(0, -1)
      allButLast.forEach(value => expect(value.dispose).toHaveBeenCalled())
    })
  })

  describe('when observed', () => {
    let dispose: () => void

    beforeEach(() => {
      dispose = autorun(() => expr.get())
    })

    it('caches value when repeatably evaluated', () => {
      const someValues = Array.from({ length: 5 }, () => expr.get())
      expect(countUnique(someValues)).toBe(1)
    })

    it('does not dispose value after evaluation', () => {
      const value = expr.get()
      expect(value.dispose).not.toHaveBeenCalled()
    })

    it('disposes stale value after recomputation', () => {
      const value = expr.get()
      model.a++
      expect(value.dispose).toHaveBeenCalled()
    })

    it('disposes value after disposing observing reaction', () => {
      const value = expr.get()
      dispose()
      expect(value.dispose).toHaveBeenCalled()
    })

    it('creates new values when repeatably evaluated after disposing observing reaction', () => {
      dispose()
      const someValues = Array.from({ length: 5 }, () => expr.get())
      expect(countUnique(someValues)).toBe(5)
    })
  })
})
