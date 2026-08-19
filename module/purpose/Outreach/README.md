# Outreach

## Description

This module runs a public outreach demo. The robot stands on the spot and looks for people using the COCO YOLO model. When it sees one, it looks at them so that they sit in the middle of the camera image, turns its body on the spot until they are straight ahead of the torso (which unwinds the head yaw), and waves at them with its right arm. When nobody is visible it stands still and scans around with its head until someone appears.

The head is driven by a `Look` task straight from the bounding box, and the body heading is driven by a proportional controller on the person's bearing in torso space with a deadband so the robot does not shuffle when it is already facing them. The wave is the existing `Wave.yaml` script, rate limited by a cooldown so the robot does not restart the wave on every detection.

Only the right arm is claimed by the wave, so the walk keeps control of the legs and left arm while the robot is waving.

## Usage

Add this module to a role with a camera, `vision::YoloCoco`, `skill::Walk`, `skill::Look`, `planning::PlanLook` and the actuation modules. See `roles/outreach.role`.

## Consumes

- `message::vision::BoundingBoxes` YOLO detections, filtered down to the most confident `person` box.
- `message::input::Sensors` to convert the detection from camera space into torso space.
- `message::purpose::Outreach` a task telling the robot to greet people.
- `message::purpose::WaveAtPerson` a task to wave the right arm at the person currently being tracked.

## Emits

- `message::skill::Look` to point the head at the person.
- `message::planning::LookAround` to scan for people when none are visible.
- `message::skill::Walk` to turn the body towards the person, or to stand still.
- `message::purpose::Outreach` and `message::purpose::WaveAtPerson` to drive its own Director tree.
- `message::strategy::FallRecovery` so the robot gets up if it falls over.
- `message::actuation::RightArmSequence` the `Wave.yaml` script.
- `message::behaviour::state::Stability` set to `STANDING` on startup, since the walk needs a stability state.

## Dependencies

- Director
- `vision::YoloCoco` for the person detections
- `utility::skill::load_script` to load the `Wave.yaml` script
