import { Vector2 } from '../../../math/vector2'
import { Matrix2 } from '../../../math/matrix2'

// Draw the ellipse points for the distributions
export default function drawDistribution(
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
