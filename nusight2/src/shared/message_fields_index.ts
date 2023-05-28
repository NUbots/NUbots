export interface MessageField {
  wireType: number
  type: string
  id: number
}

export interface MessageFieldsIndex {
  [messageName: string]: {
    [fieldName: string]: MessageField
  }
}

export const messageFieldsIndex: MessageFieldsIndex = {
  "message.actuation.ServoTarget": {
    "id": {
      "type": "uint32",
      "id": 2,
      "wireType": 0
    }
  },
  "message.behaviour.ServoCommand": {
    "id": {
      "type": "uint32",
      "id": 3,
      "wireType": 0
    }
  },
  "message.input.GameState.Data.Robot": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.Image": {
    "id": {
      "type": "uint32",
      "id": 4,
      "wireType": 0
    }
  },
  "message.input.MotionCapture.Marker": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.MotionCapture.RigidBody": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.MotionCapture.Skeleton": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.MotionCapture.ForcePlate": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.MotionCapture.Device": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.Sensors.Servo": {
    "id": {
      "type": "uint32",
      "id": 2,
      "wireType": 0
    }
  },
  "message.input.Sensors.Button": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.input.Sensors.LED": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.output.CompressedImage": {
    "id": {
      "type": "uint32",
      "id": 4,
      "wireType": 0
    }
  },
  "message.vision.Balls": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.vision.FieldLines": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.vision.Goals": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.vision.GreenHorizon": {
    "id": {
      "type": "uint32",
      "id": 3,
      "wireType": 0
    }
  },
  "message.vision.Obstacle": {
    "id": {
      "type": "uint32",
      "id": 1,
      "wireType": 0
    }
  },
  "message.vision.VisualMesh": {
    "id": {
      "type": "uint32",
      "id": 2,
      "wireType": 0
    }
  }
}