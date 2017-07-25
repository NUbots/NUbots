#include "BonnWalk.h"

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/WalkCommand.h"

#include "utility/support/eigen_armadillo.h"

namespace module {
namespace motion {

    using extension::Configuration;
    using message::behaviour::ServoCommand;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStarted;
    using message::motion::WalkStopped;

    BonnWalk::BonnWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), engine(*this) {

        using namespace std::chrono;

        on<Configuration>("BonnWalk.yaml").then([this](const Configuration& config) { engine.updateConfig(config); });

        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& cmd) {
            engine.reset();
            subsumptionId = cmd.subsumptionId;
            handle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this] {
            subsumptionId = 0;
            handle.disable();
        });

        handle =
            on<Every<90, Per<seconds>>, With<Sensors>, Single, Priority::HIGH>().then([this](const Sensors& sensors) {

                // Update our angles
                for (size_t i = 0; i < sensors.servo.size(); ++i) {
                    engine.in.jointPos[i] = sensors.servo[i].presentPosition;
                }

                // Update our timestamps
                double timestamp    = duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
                engine.in.truedT    = timestamp - engine.in.timestamp;
                engine.in.timestamp = timestamp;
                engine.in.nominaldT = duration_cast<duration<double>>(updateRate).count();
                engine.in.Htw       = sensors.world;
                engine.in.gyroscope = sensors.gyroscope;

                // See if we finished walking
                bool walking = engine.out.walking;

                // Execute our walk engine
                engine.step();

                // If we just stopped, let everyone know we are not walking
                if (walking && !engine.out.walking) {
                    emit(std::make_unique<WalkStopped>());
                }
                else if (!walking && engine.out.walking) {
                    emit(std::make_unique<WalkStarted>());
                }

                auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                // Get the outputs
                for (size_t i = 0; i < engine.out.jointCmd.size(); ++i) {
                    ServoCommand waypoint;
                    waypoint.source   = subsumptionId;
                    waypoint.time     = NUClear::clock::now() + updateRate;
                    waypoint.id       = i;
                    waypoint.position = engine.out.jointCmd[i];
                    waypoint.gain     = 100 * engine.out.jointEffort[i];
                    waypoint.torque   = 100;

                    waypoints->push_back(std::move(waypoint));
                }

                emit(waypoints);
            });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& cmd) {

            // Update the engines command
            engine.in.gaitCmd.linVelX = cmd.command.x();
            engine.in.gaitCmd.linVelY = cmd.command.y();
            engine.in.gaitCmd.angVelZ = cmd.command.z();
            engine.in.gaitCmd.walk    = true;
        });

        on<Trigger<StopCommand>>().then([this] {

            // Update the engines command
            engine.in.gaitCmd.linVelX = 0;
            engine.in.gaitCmd.linVelY = 0;
            engine.in.gaitCmd.angVelZ = 0;
            engine.in.gaitCmd.walk    = false;
        });
    }

}  // namespace motion
}  // namespace module
