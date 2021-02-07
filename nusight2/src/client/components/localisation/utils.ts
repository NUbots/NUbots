import { createTransformer } from 'mobx-utils'
import { MeshLambertMaterial } from 'three'
import { Color } from 'three'
import { Geometry } from 'three'
import { Material } from 'three'
import { LegacyJSONLoader } from 'three/examples/jsm/loaders/deprecated/LegacyJSONLoader'

export function geometryAndMaterial(config: any, color?: string) {
  const geometryAndMaterial = parseConfig(config)
  const geometry = geometryAndMaterial.geometry
  let materials = geometryAndMaterial.materials
  if (materials !== undefined) {
    materials = materials.map(material => coloredMaterial(material, color))
  }
  return {
    geometry,
    materials,
    dispose() {
      this.geometry.dispose()
      if (this.materials) {
        this.materials.forEach(material => material.dispose())
      }
    },
  }
}

const coloredMaterial = (material: Material, color?: string) => {
  if (material instanceof MeshLambertMaterial && material.name === 'Plastic' && color) {
    const newMaterial = material.clone()
    newMaterial.color.lerp(new Color(color), 0.5)
    return newMaterial
  } else {
    return material
  }
}

const parseConfig = createTransformer((config: any) => {
  const { geometry, materials } = new LegacyJSONLoader().parse(config, '/')
  // Compute vertex normals with a crease angle of 0.52
  computeVertexNormals(geometry, 0.52)
  return { geometry, materials }
})

/**
 * Drew's crease angle computeVertexNormals function for the darwin
 *
 * @author Drew Noakes http://drewnoakes.com
 */
function computeVertexNormals(geometry: Geometry, maxSmoothAngle: number) {
  const faceIndicesPerVertex: number[][] = []
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
    const faces = [face.a, face.b, face.c]
    for (let fv = 0; fv < 3; fv++) {
      const vertexIndex = faces[fv]
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
