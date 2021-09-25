import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'

import { Transform } from '../../../math/transform'
import { Vector2 } from '../../../math/vector2'
import { Matrix2 } from '../../../math/matrix2'
import { BasicAppearance } from '../../../render2d/appearance/basic_appearance'
import { LineAppearance } from '../../../render2d/appearance/line_appearance'
import { ArcGeometry } from '../../../render2d/geometry/arc_geometry'
import { ArrowGeometry } from '../../../render2d/geometry/arrow_geometry'
import { CircleGeometry } from '../../../render2d/geometry/circle_geometry'
import { LineGeometry } from '../../../render2d/geometry/line_geometry'
import { MarkerGeometry } from '../../../render2d/geometry/marker_geometry'
import { TextGeometry } from '../../../render2d/geometry/text_geometry'
import { Group } from '../../../render2d/object/group'
import { Shape } from '../../../render2d/object/shape'

import { DashboardRobotModel } from './model'
import { PolygonGeometry } from '../../../render2d/geometry/polygon_geometry'

export class DashboardRobotViewModel {
  constructor(private model: DashboardRobotModel) {}

  static of = createTransformer((model: DashboardRobotModel): DashboardRobotViewModel => {
    return new DashboardRobotViewModel(model)
  })

  @computed
  get robot(): Group {
    return Group.of({
      children: [this.fieldSpaceGroup, this.robotSpaceGroup],
    })
  }

  @computed
  get fieldSpaceGroup() {
    return Group.of({
      children: [this.ballSight, this.kickTarget, this.ball],
    })
  }

  @computed
  get robotSpaceGroup() {
    return Group.of({
      children: [this.walkCommand, this.robotMarker],
      transform: Transform.of({
        rotate: this.model.robotPosition.z,
        translate: {
          x: this.model.robotPosition.x,
          y: this.model.robotPosition.y,
        },
      }),
    })
  }

  @computed
  private get walkCommand() {
    const t = 2
    const translation = Vector2.from(this.model.walkCommand)
    const rotation = this.model.walkCommand.z
    const radius = translation.length / (Math.abs(rotation) + 1e-10)
    const origin = Vector2.of(-translation.y, translation.x).divideScalar(rotation)
    const arcLength = rotation * t
    const angle = Math.atan2(translation.y / rotation, translation.x / rotation) - Math.PI / 2

    const startAngle = angle
    const endAngle = startAngle + arcLength

    return Shape.of(
      ArcGeometry.of({
        origin,
        radius,
        startAngle,
        endAngle,
        anticlockwise: rotation < 0,
      }),
      BasicAppearance.of({
        stroke: { width: 0.025, color: '#000000' },
      }),
    )
  }

  @computed
  private get ballProbabilities() {
    console.log('Ball covariance')
    console.log(this.model.ballCovariance)
    console.log('Ball position')
    console.log(this.model.ballPosition)
    console.log('\n')

    // Different colours of the probabilities
    const STD_1_COLOR = '#ff0000'
    const STD_2_COLOR = '#eeff00'
    const STD_3_COLOR = '#00ff04'

    // Draw the points for the distributions
    function drawDistribution(
      numPoints: number = 20,
      covarianceMatrix: Matrix2,
      positionVector: Vector2,
      zScore: number,
    ) {
      // Vectors for the standard deviation
      const vectors: Vector2[] = []

      // Get the variances
      const stdX = Math.sqrt(covarianceMatrix.x.x) * zScore
      const stdY = Math.sqrt(covarianceMatrix.y.y) * zScore

      // Get the points that make the ellipse
      for (let i = 0; i < numPoints; i++) {
        // Calculate angle and ray of ellipse for probability distribution
        const angle = (2 * Math.PI * i) / numPoints
        const hypotenuse =
          (stdX * stdY) /
          Math.sqrt(Math.pow(stdX * Math.sin(angle), 2) + Math.pow(stdY * Math.cos(angle), 2))

        // Calculate coords relative to point
        const relCoordX = hypotenuse * Math.cos(angle)
        const relCoordY = hypotenuse * Math.sin(angle)

        // Calculate coords of points and add to list
        const coordX = positionVector.x + relCoordX
        const coordY = positionVector.y + relCoordY
        const vector = new Vector2(coordX, coordY)
        vectors.push(vector)
      }

      return vectors
    }

    const vecs1 = drawDistribution(undefined, this.model.ballCovariance, this.model.ballPosition, 1)
    const vecs2 = drawDistribution(undefined, this.model.ballCovariance, this.model.ballPosition, 2)
    const vecs3 = drawDistribution(undefined, this.model.ballCovariance, this.model.ballPosition, 3)

    return (
      Shape.of(
        PolygonGeometry.of(vecs1),
        BasicAppearance.of({ fill: { color: STD_1_COLOR, alpha: 0.5 } }),
      ),
      Shape.of(
        PolygonGeometry.of(vecs2),
        BasicAppearance.of({ fill: { color: STD_2_COLOR, alpha: 0.3 } }),
      ),
      Shape.of(
        PolygonGeometry.of(vecs3),
        BasicAppearance.of({ fill: { color: STD_3_COLOR, alpha: 0.1 } }),
      )
    )
  }

  @computed
  private get ball() {
    return Shape.of(
      CircleGeometry.of({
        radius: 0.1,
        x: this.model.ballPosition.x,
        y: this.model.ballPosition.y,
      }),
      BasicAppearance.of({
        fill: { color: this.model.ballColor },
      }),
    )
  }

  @computed
  private get ballSight() {
    return Shape.of(
      LineGeometry.of({
        origin: Vector2.from(this.model.robotPosition),
        target: this.model.ballPosition,
      }),
      LineAppearance.of({
        stroke: { width: 0.025, color: this.model.ballSightColor },
      }),
    )
  }

  @computed
  private get kickTarget() {
    const origin = this.model.ballPosition
    const difference = this.model.kickTarget.subtract(origin)
    return Shape.of(
      ArrowGeometry.of({
        direction: difference.normalize(),
        headLength: 0.3,
        headWidth: 0.15,
        length: difference.length,
        origin,
        width: 0.025,
      }),
      BasicAppearance.of({
        fill: { color: this.model.kickTargetColor },
      }),
    )
  }

  @computed
  private get robotMarker() {
    const radius = 0.15
    return Group.of({
      children: [
        Shape.of(
          MarkerGeometry.of({
            radius,
            x: 0,
            y: 0,
          }),
          BasicAppearance.of({
            fill: { color: this.model.robotColor },
          }),
        ),
        Shape.of(
          TextGeometry.of({
            text: this.model.playerId.toString(),
            worldAlignment: true,
            textAlign: 'middle',
            textBaseline: 'middle',
            fontSize: `${radius * 1.9}px`,
            x: 0,
            y: 0,
          }),
          BasicAppearance.of({
            fill: { color: this.model.textColor },
          }),
        ),
      ],
    })
  }
}
