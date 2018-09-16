#include "NSGA2Evaluator.h"

#include "extension/Configuration.h"
#include "message/support/optimisation/NSGA2EvaluationRequest.h"
#include "message/support/optimisation/NSGA2EvaluationParameters.h"
#include "message/support/optimisation/NSGA2FitnessScores.h"
#include "message/support/optimisation/NSGA2Terminate.h"
#include "message/support/Gazebo/GazeboWorldCtrl.h"
#include "message/support/Gazebo/GazeboWorldStatus.h"
#include "message/support/Gazebo/GazeboBallLocation.h"
#include "message/support/Gazebo/GazeboRobotLocation.h"
#include "message/input/Sensors.h"
#include "message/behaviour/MotionCommand.h"
#include "utility/behaviour/MotionCommand.h"
#include "utility/behaviour/Action.h"
#include "utility/file/fileutil.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform2D.h"
#include "message/motion/KickCommand.h"
#include "message/motion/ExecuteKick.h"

namespace module {
namespace support {
namespace optimisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2EvaluationParameters;
    using message::support::optimisation::NSGA2Terminate;
    using message::support::Gazebo::GazeboBallLocation;
    using message::support::Gazebo::GazeboRobotLocation;
    using message::support::Gazebo::GazeboWorldCtrl;
    using message::motion::ExecuteKick;
    using message::support::Gazebo::GazeboWorldStatus;
    using message::behaviour::MotionCommand;
    using utility::math::matrix::Transform2D;
    using message::motion::KickCommand;
    using extension::ExecuteScript;
    using extension::Script;
    using utility::behaviour::RegisterAction;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)),
        subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

        on<Configuration>("NSGA2Evaluator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NSGA2Evaluator.yaml

            evaluating = false;
            finished = false;
            walking = false;
            standing = false;
            terminating = false;
            generation = 0;
            id = 0;
            scores = std::vector<double>(2, 0.0);
            constraints = std::vector<double>(2, 0.0);
            parameters = std::vector<double>();

            gyroscope[0] = 0.0;
            gyroscope[1] = 0.0;
            gyroscope[2] = 0.0;

            simTime = 0.0;

            accelerometer[0] = 0.0;
            accelerometer[1] = 0.0;
            accelerometer[2] = 0.0;

            ballLocation[0] = 0.0;
            ballLocation[1] = 0.0;
            ballLocation[2] = 0.0;

            robotLocation[0] = 0.0;
            robotLocation[1] = 0.0;
            robotLocation[2] = 0.0;

            sway[0] = 0.0;
            sway[1] = 0.0;
            sway[2] = 0.0;
            fallenOver = false;

            simTimeDelta = 0.0;

            velocity = {0.5, 0.0};
            rotation = 0.0;

            distanceTravelled = 0.0;
            optimizeScript = true;
            if (optimizeScript)
                script = YAML::LoadFile("scripts/nubotsvm/KickRightOptimised.yaml").as<Script>();

            //ResetWorld();
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumptionId,
            "NSGA2 Evaluator",
            {std::pair<float, std::set<LimbID>>(
                0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>&) {},
            [this](const std::set<LimbID>&) {},
            [this](const std::set<ServoID>&) {}}));

        on<Trigger<NSGA2EvaluationRequest>, Single>().then([this](const NSGA2EvaluationRequest& request) {
            // set config file with request parameters
            // run kick
            generation = request.generation;
            id = request.id;
            parameters = request.reals;
            //if (generation == 1 && id == 0)
            //    ResetWorld();
            ResetWorldTime();

            std::unique_ptr<NSGA2EvaluationParameters> params = std::make_unique<NSGA2EvaluationParameters>();
            params->bodyTilt = parameters[0];
            params->qLArmStartPitch = parameters[1];

            if (optimizeScript)
            {
                int var = 0;
                for (int i = 0; i < script.frames.size(); i++)
                {
                    for (auto& target : script.frames[i].targets)
                    {
                        target.position = (float)parameters[var];
                        var++;
                    }
                }
            }
            //emit(std::make_unique<KickCommand>(KickCommand{
            //       {0, -1, 0}, //Ball is right of centre for right kick
            //{0, 0, 1}, message::motion::KickCommandType::NORMAL
            //   }));//log("EMITTING");
            //emit(std::make_unique<MotionCommand>(
            //    utility::behaviour::DirectCommand(Transform2D(velocity, rotation))));

            emit(params);
            terminating = false;
        });

        on<Trigger<Sensors>, Single>().then([this](const Sensors& sensors) {
            // Get the sensory data
            // if fallen over, trigger end of evaluation
            //if (sensors.)
            accelerometer[0] = sensors.accelerometer[0];
            accelerometer[1] = sensors.accelerometer[1];
            accelerometer[2] = sensors.accelerometer[2];//std::cout << accelerometer[2] << std::endl;

            gyroscope[0] = sensors.gyroscope[0];
            gyroscope[1] = sensors.gyroscope[1];
            gyroscope[2] = sensors.gyroscope[2];//log(distanceTravelled);

            sway[0] += gyroscope[0] * simTimeDelta;
            sway[1] += gyroscope[1] * simTimeDelta;
            sway[2] += gyroscope[2] * simTimeDelta;

            if (!terminating && (accelerometer[0] < -7.0 || accelerometer[1] < -7.0) && accelerometer[2] < 0.0 && simTime > 3.0)
                fallenOver = true;

            fieldPlaneSway = std::pow(std::pow(accelerometer[0], 2) + std::pow(accelerometer[1], 2), 0.5);

            if (!terminating && evaluating && !finished && fieldPlaneSway > maxFieldPlaneSway && simTime > 0.1) {

                maxFieldPlaneSway = fieldPlaneSway;
                //log(maxFieldPlaneSway);
            }
            //log("gyroscope.x = " + std::to_string(gyroscope[0]));
            //log("gyroscope.y = " + std::to_string(gyroscope[1]));
            //log("gyroscope.z = " + std::to_string(gyroscope[2]));
            // log("accelerme.x = " + std::to_string(accelerometer[0]));
            // log("accelerme.y = " + std::to_string(accelerometer[1]));
            // log("accelerme.z = " + std::to_string(accelerometer[2]));

            if (!terminating && evaluating && !finished && simTime > 4.0)
            {
                if (fallenOver)
                    BeginTermination();
                else if (ballVelocity[0] == 0.0 && ballVelocity[1] == 0.0 && ballVelocity[2] == 0.0)
                    BeginTermination();
            }


        });

        on<Trigger<GazeboWorldStatus>, Single>().then([this](const GazeboWorldStatus& status) {
            // Get the sim time
            simTimeDelta = status.simTime - simTime;
            simTime = status.simTime;

            //if (simTime > 3.0 && !walking) {
            //    walking = true;
                //{std::cout << "running\n";
                //std::cout << "EMITTING\n";
                //emit(std::make_unique<ExecuteKick>());


            //}

            if (simTime > 1.0 && !evaluating & !terminating) {
                emit(std::make_unique<ExecuteScript>(subsumptionId, script, NUClear::clock::now()));
                ResetWorldTime();
                evaluating = true;
            }

            if (terminating && !evaluating && !finished && (simTime - timeSinceTermination) > 2.0) {
                terminating = false;
                SendFitnessScores();
            }

        });

        on<Trigger<NSGA2Terminate>, Single>().then([this](const NSGA2Terminate& terminate) {
            // Get the sim time
            finished = true;

        });

        on<Trigger<GazeboBallLocation>, Single>().then([this](const GazeboBallLocation& location) {
            // Get the sensory data
            // if fallen over, trigger end of evaluation
            //if (sensors.)

            ballLocation[0] = location.x;
            ballLocation[1] = location.y;
            ballLocation[2] = location.z;

            ballVelocity[0] = location.velx;
            ballVelocity[1] = location.vely;
            ballVelocity[2] = location.velz;
           //log(ballVelocity[0]);
            //log(ballVelocity[1]);
            //log(ballVelocity[2]);
        });

        on<Trigger<GazeboRobotLocation>, Single>().then([this](const GazeboRobotLocation& location) {

            //distanceTravelled += location.x + 1;//std::pow(std::pow(location.x - robotLocation[0], 2) + std::pow(location.y - robotLocation[1], 2), 0.5);

            robotLocation[0] = location.x;
            robotLocation[1] = location.y;
            robotLocation[2] = location.z;
        });


    }

    void NSGA2Evaluator::SendFitnessScores()
    {
        std::unique_ptr<NSGA2FitnessScores> fitnessScores = std::make_unique<NSGA2FitnessScores>();
        fitnessScores->id = id;
        fitnessScores->generation = generation;
        fitnessScores->objScore = scores;
        fitnessScores->constraints = constraints;
        emit(fitnessScores);
    }

    void NSGA2Evaluator::ResetWorld()
    {
        std::unique_ptr<GazeboWorldCtrl> command = std::make_unique<GazeboWorldCtrl>();
        command->command = "RESET";
        emit(command);

        distanceTravelled = 0.0;
        timeSinceTermination = 0.0;
        sway[0] = 0.0;
        sway[1] = 0.0;
        sway[2] = 0.0;
        fieldPlaneSway = 0.0;
        maxFieldPlaneSway = 0.0;
        simTimeDelta = 0.0;
        constraints[0] = 0.0;
        constraints[1] = 0.0;
        fallenOver = false;
    }

    void NSGA2Evaluator::ResetWorldTime()
    {
        std::unique_ptr<GazeboWorldCtrl> command = std::make_unique<GazeboWorldCtrl>();
        command->command = "RESETTIME";
        emit(command);
    }

    bool NSGA2Evaluator::BeginTermination()
    {
        terminating = true;
        evaluating = false;
        log("terminating..");
        timeSinceTermination = simTime;
        CalculateFitness();
        ResetWorld();
        //emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
    }

    void NSGA2Evaluator::CalculateFitness()
    {
        distanceTravelled = std::abs(ballLocation[0]);

        if (distanceTravelled == 0.0)
            distanceTravelled = 0.001;
        else if (distanceTravelled > 10.0)
            distanceTravelled = 10.0;

        maxFieldPlaneSway = std::abs(maxFieldPlaneSway);

        if (maxFieldPlaneSway == 0 || maxFieldPlaneSway > 1000.0)
            maxFieldPlaneSway = 1000.0;

        if (fallenOver)
            constraints[0] = -10.0;

        sway[2] = std::abs(sway[2]);
        //log(sway[2]);

        if (sway[2] > 6.66)
            constraints[1] = -1.0 * (sway[2] - 6.66);
        else
            constraints[1] = 0.0;

        scores[0] = maxFieldPlaneSway;
        scores[1] = 1.0 / (distanceTravelled);
        log(scores[0]); log(scores[1]);
    }
}
}
}
