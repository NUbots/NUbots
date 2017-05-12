import { createTransformer } from 'mobx'
import { MeshLambertMaterial } from 'three'
import { Color } from 'three'
import { JSONLoader } from 'three'

export function geometryAndMaterial(config: any, color: string) {
  const { geometry, materials } = parseConfig(config)
  const newMaterials = materials.map(material => coloredMaterial(material, color))
  return { geometry, materials: newMaterials }
}

const coloredMaterial = (material, color) => {
  if (material instanceof MeshLambertMaterial && material.name === 'Plastic' && color) {
    const newMaterial = material.clone()
    newMaterial.color.lerp(new Color(color), 0.5)
    return newMaterial
  } else {
    return material
  }
}

const parseConfig = createTransformer((config: any) => {
  const { geometry, materials } = new JSONLoader().parse(config)
  // Compute vertex normals with a crease angle of 0.52
  computeVertexNormals(geometry, 0.52)
  return { geometry, materials }
})

/**
 * Drew's crease angle computeVertexNormals function for the darwin
 *
 * @author Drew Noakes http://drewnoakes.com
 */
function computeVertexNormals(geometry, maxSmoothAngle) {
  const faceIndicesPerVertex = []
  for (let v = 0, vl = geometry.vertices.length; v < vl; v++) {
    faceIndicesPerVertex.push([])
  }
  for (let f = 0, fl = geometry.faces.length; f < fl; f++) {
    const face = geometry.faces[f]
    faceIndicesPerVertex[face.a].push(f)
    faceIndicesPerVertex[face.b].push(f)
    faceIndicesPerVertex[face.c].push(f)
  }
  for (const face of geometry.faces) {
    for (let fv = 0; fv < 3; fv++) {
      const vertexIndex = face['abcd'.charAt(fv)]
      const vertexFaces = faceIndicesPerVertex[vertexIndex]
      const vertexNormal = face.normal.clone()
      for (const neighbourFaceIndex of vertexFaces) {
        const neighbourFace = geometry.faces[neighbourFaceIndex]
        // disregard the face we're working with
        if (neighbourFace === face) {
          continue
        }
        // given both normals are unit vectors, the angle is just acos(a.dot(b))
        const theta = Math.acos(face.normal.dot(neighbourFace.normal))
        if (theta <= maxSmoothAngle) {
          vertexNormal.add(neighbourFace.normal)
        }
      }
      vertexNormal.normalize()
      face.vertexNormals[fv] = vertexNormal
    }
  }
}
