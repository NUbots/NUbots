import { Transform } from '../transform'
import { Vector2 } from '../vector2'

describe('Vector2', () => {
  it('should apply transform rotate correctly', () => {
    const vec2 = Vector2.of(2, 3)
    const transform = Transform.of({ rotate: Math.PI })

    const actual = vec2.transform(transform)
    const expected = Vector2.of(-2, -3)
    expect(actual.x).toBeCloseTo(expected.x)
    expect(actual.y).toBeCloseTo(expected.y)
  })

  it('should apply transform scale correctly', () => {
    const vec2 = Vector2.of(2, 3)
    const transform = Transform.of({ scale: Vector2.of(2, 2) })

    const actual = vec2.transform(transform)
    const expected = Vector2.of(4, 6)
    expect(actual.x).toBeCloseTo(expected.x)
    expect(actual.y).toBeCloseTo(expected.y)
  })

  it('should apply transform translate correctly', () => {
    const vec2 = Vector2.of(2, 3)
    const transform = Transform.of({ translate: { x: 1, y: 1 } })

    const actual = vec2.transform(transform)
    const expected = Vector2.of(3, 4)
    expect(actual.x).toBeCloseTo(expected.x)
    expect(actual.y).toBeCloseTo(expected.y)
  })

  it('should apply all transforms correctly', () => {
    const vec2 = Vector2.of(2, 3)
    const transform = Transform.of({
      rotate: Math.PI,
      scale: Vector2.of(2, 2),
      translate: { x: 1, y: 1 },
    })

    const actual = vec2.transform(transform)
    const expected = Vector2.of(-3, -5)
    expect(actual.x).toBeCloseTo(expected.x)
    expect(actual.y).toBeCloseTo(expected.y)
  })
})
