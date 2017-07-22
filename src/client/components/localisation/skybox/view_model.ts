import { createTransformer } from 'mobx'
import { computed } from 'mobx'
import { BackSide } from 'three'
import { Mesh } from 'three'
import { SphereBufferGeometry } from 'three'
import { MeshBasicMaterial } from 'three'
import { Object3D } from 'three'
import { ShaderMaterial } from 'three'
import { Vector3 } from 'three'
import { PlaneBufferGeometry } from 'three'
import { SkyboxModel } from './model'
import * as SkyboxFrag from './skybox.frag'
import * as SkyboxVert from './skybox.vert'

export class SkyboxViewModel {

  public constructor(private model: SkyboxModel) {
  }

  public static of = createTransformer((model: SkyboxModel): SkyboxViewModel => {
    return new SkyboxViewModel(model)
  })

  @computed
  public get skybox() {
    // reference: http://threejs.org/examples/#webgl_shaders_sky
    const skybox = new Object3D()

    skybox.add(this.sky)
    skybox.add(this.ground)

    if (this.model.showSun) {
      skybox.add(this.sun)
    }

    return skybox
  }

  @computed
  private get sky() {
    const geo = new SphereBufferGeometry(40, 32, 15)
    const mat = new ShaderMaterial({
      fragmentShader: String(SkyboxFrag),
      vertexShader: String(SkyboxVert),
      uniforms: {
        luminance: { value: this.model.luminance },
        turbidity: { value: this.model.turbidity },
        rayleigh: { value: this.model.rayleigh },
        mieCoefficient: { value: this.model.mieCoefficient },
        mieDirectionalG: { value: this.model.mieDirectionalG },
        sunPosition: { value: this.sunPosition },
      },
      side: BackSide,
    })

    const mesh = new Mesh(geo, mat)
    mesh.name = 'skyboxSky'

    return mesh
  }

  @computed
  private get ground() {
    const groundGeo = new PlaneBufferGeometry(27, 18)
    const groundMat = new MeshBasicMaterial({ color: '#3d7926' })
    const ground = new Mesh(groundGeo, groundMat)
    ground.name = 'skyboxGround'
    ground.position.z = -0.01

    return ground
  }


  @computed
  private get sun() {
    const sunSphere = new Mesh(
      new SphereBufferGeometry(40, 16, 8),
      new MeshBasicMaterial({ color: 0xffffff }),
    )

    sunSphere.name = 'skyboxSun'
    sunSphere.position.copy(this.sunPosition)

    return sunSphere
  }

  @computed
  private get sunPosition(): Vector3 {
    const position = new Vector3()
    const distance = 40
    const theta = Math.PI * (this.model.inclination - 0.5)
    const phi = 2 * Math.PI * (this.model.azimuth - 0.5)

    position.x = distance * Math.cos(phi)
    position.y = distance * Math.sin(phi) * Math.sin(theta)
    position.z = distance * Math.sin(phi) * Math.cos(theta)

    return position
  }
}
