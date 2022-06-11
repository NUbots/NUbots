import { computed } from 'mobx'
import { createTransformer, lazyObservable } from 'mobx-utils'
import * as THREE from 'three'
import { Euler, Matrix4, Mesh, Object3D, Quaternion } from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader'
import { group } from '../../three/builders'
import { KinematicsRobotModel } from '../darwin_robot/model'
import url from './config/nugus.glb'

export class NUgusViewModel {
  constructor(private readonly model: KinematicsRobotModel) {
    const robot = this.robotObject
    if (robot)
      Object.keys(meshes).forEach(mesh => {
        if (meshes[mesh]) findMesh(robot, mesh).add(worldLines(mesh))
      })
  }

  static of = createTransformer((model: KinematicsRobotModel) => {
    return new NUgusViewModel(model)
  })

  @computed({ equals: () => false })
  get robot(): Object3D {
    const robot = this.robotObject
    if (!robot) {
      // Model has not loaded yet, show a dummy object until it loads.
      return this.dummyObject
    }
    const motors = this.model.motors
    // TODO (Annable): Baking the offsets into the model geometries would be ideal.
    const PI = Math.PI
    const PI_2 = PI / 2
    robot.position.x = this.model.rWTt.x
    robot.position.y = this.model.rWTt.y
    robot.position.z = this.model.rWTt.z
    const rotation = new Quaternion(
      this.model.Rwt.x,
      this.model.Rwt.y,
      this.model.Rwt.z,
      this.model.Rwt.w,
    )
    rotation.multiply(new Quaternion().setFromEuler(new Euler(PI_2, 0, 0)))
    robot.setRotationFromQuaternion(rotation)

    findMesh(robot, 'R_Shoulder').rotation.set(0, 0, PI_2 - motors.rightShoulderPitch.angle)
    findMesh(robot, 'L_Shoulder').rotation.set(0, 0, -PI_2 - motors.leftShoulderPitch.angle)
    findMesh(robot, 'R_Arm_Upper').rotation.set(0, PI_2, motors.rightShoulderRoll.angle)
    findMesh(robot, 'L_Arm_Upper').rotation.set(0, PI_2, -motors.leftShoulderRoll.angle)
    findMesh(robot, 'R_Arm_Lower').rotation.set(motors.rightElbow.angle, 0, 0)
    findMesh(robot, 'L_Arm_Lower').rotation.set(motors.leftElbow.angle, 0, 0)
    findMesh(robot, 'R_Hip_Yaw').rotation.set(0, motors.rightHipYaw.angle - PI_2, -PI_2)
    findMesh(robot, 'L_Hip_Yaw').rotation.set(0, PI_2 + motors.leftHipYaw.angle, -PI_2)
    findMesh(robot, 'R_Hip').rotation.set(0, 0, PI_2 - motors.rightHipRoll.angle)
    findMesh(robot, 'L_Hip').rotation.set(0, 0, motors.leftHipRoll.angle - PI_2)
    findMesh(robot, 'R_Upper_Leg').rotation.set(-motors.rightHipPitch.angle, 0, PI)
    findMesh(robot, 'L_Upper_Leg').rotation.set(-motors.leftHipPitch.angle, 0, PI)
    findMesh(robot, 'R_Lower_Leg').rotation.set(motors.rightKnee.angle, 0, 0)
    findMesh(robot, 'L_Lower_Leg').rotation.set(motors.leftKnee.angle, 0, 0)
    findMesh(robot, 'R_Ankle').rotation.set(motors.rightAnklePitch.angle, 0, 0)
    findMesh(robot, 'L_Ankle').rotation.set(motors.leftAnklePitch.angle, 0, 0)
    findMesh(robot, 'R_Foot').rotation.set(0, 0, -motors.rightAnkleRoll.angle)
    findMesh(robot, 'L_Foot').rotation.set(0, 0, motors.leftAnkleRoll.angle)
    findMesh(robot, 'Neck').rotation.set(0, PI + motors.headPan.angle, 0)
    findMesh(robot, 'Head').rotation.set(0, 0, motors.headTilt.angle)
    return robot
  }

  @computed
  public get robotObject() {
    const current = NUgusViewModel.robotObjectBase.current()
    return current
      ? current.clone() // Clone so that each view model instance has its own copy that it may mutate.
      : undefined
  }

  private static robotObjectBase = lazyObservable<Object3D | undefined>(sink => {
    new GLTFLoader().load(url, gltf => {
      // TODO (Annable): Baking this rotation into the model geometry would be ideal.
      findMesh(gltf.scene, 'Head').geometry.applyMatrix(new Matrix4().makeRotationY(-Math.PI / 2))
      sink(findMesh(gltf.scene, 'Torso'))
    })
  })

  @computed
  get dummyObject() {
    return new Object3D()
  }
}

const createLine = (
  to: [number, number, number],
  color: any = '0xffffff',
  linewidth: number = 1,
  from: [number, number, number] = [0, 0, 0],
): THREE.Line => {
  return new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(...from),
      new THREE.Vector3(...to),
    ]),
    new THREE.LineBasicMaterial({ color, linewidth }),
  )
}

const worldLines = (mesh: string): Object3D => {
  const lines = group(() => ({
    children: [
      createLine([0, 0, 0.1], 0x0000ff, 3),
      createLine([0, 0.1, 0], 0xff0000, 3),
      createLine([0.1, 0, 0], 0x00ff00, 3),
    ],
  }))
  const lineGroup = <Object3D>lines()
  lineGroup.name = `${mesh}_kinematics`
  return lineGroup
}

const meshes: { [key: string]: boolean } = {
  R_Shoulder: true,
  L_Shoulder: true,
  R_Arm_Upper: true,
  L_Arm_Upper: true,
  R_Arm_Lower: true,
  L_Arm_Lower: true,
  R_Hip_Yaw: true,
  L_Hip_Yaw: true,
  R_Hip: true,
  L_Hip: true,
  R_Upper_Leg: true,
  L_Upper_Leg: true,
  R_Lower_Leg: false,
  L_Lower_Leg: false,
  R_Ankle: true,
  L_Ankle: true,
  R_Foot: true,
  L_Foot: true,
  Neck: true,
  Head: true,
}

const findMesh = (root: Object3D, name: string): Mesh => {
  const object = root.getObjectByName(name)
  if (!object || !(object instanceof Mesh)) {
    throw new Error()
  }
  return object
}
