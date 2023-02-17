# PlanLook

## Description

Plans what to look at based on requested features to look for.

Currently support features are ball, goals and field. The field request will make the robot periodically look over the field for field line information for localisation.

If any of the requested features have not been seen for a specified time, then a search pattern runs.

If no features, or only the field, have been requested, then the search pattern runs.

If no particular search is required, the robot will fixate on the ball. If the ball wasn't requested, it will fixate on the goals instead. If the goals were not requested, it will search.

## Usage

Add this module to your role to make the robot look for particular features.

## Consumes

- `message::planning::LookForFeatures` a Task requesting to look for the specified features
- `message::localisation::FilteredBall` with the position of the ball for fixation and knowing if we've seen the ball
- `message::vision::Goals` with the position of the goals for fixation and knowing if we've seen goals
- `message::input::Sensors` for the world matrix to convert the goals measurement into the right space
- `message::planning::LookSearch` a Task requesting to run the search pattern

## Emits

- `message::skill::Look` a Task requesting to look in a direction
- `message::planning::LookSearch` a Task requesting to run the search pattern

## Dependencies

- The coordinates utility
