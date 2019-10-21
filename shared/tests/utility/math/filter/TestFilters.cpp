/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include <Eigen/Core>
#include <array>
#include <catch.hpp>
#include <utility>

#include "VanDerPolModel.h"
#include "utility/math/filter/eigen/ParticleFilter.h"
#include "utility/math/filter/eigen/UKF.h"

static const std::array<Eigen::Vector2d, 101> true_state   = {Eigen::Vector2d(2.0, 0.0),
                                                            Eigen::Vector2d(1.99762085441568, -0.0928337297585024),
                                                            Eigen::Vector2d(1.99093577483406, -0.172657352827464),
                                                            Eigen::Vector2d(1.98055246651833, -0.241386903989206),
                                                            Eigen::Vector2d(1.96696380975438, -0.300753729369923),
                                                            Eigen::Vector2d(1.95059853034981, -0.352307847904520),
                                                            Eigen::Vector2d(1.93182840250224, -0.397424329526244),
                                                            Eigen::Vector2d(1.91095234669689, -0.437298089113407),
                                                            Eigen::Vector2d(1.88818825507071, -0.472904541550317),
                                                            Eigen::Vector2d(1.86372114295935, -0.505086429882456),
                                                            Eigen::Vector2d(1.83770922476959, -0.534570538546817),
                                                            Eigen::Vector2d(1.81028386745444, -0.561968479046575),
                                                            Eigen::Vector2d(1.78154105317577, -0.587804299589731),
                                                            Eigen::Vector2d(1.75153740848162, -0.612502509271478),
                                                            Eigen::Vector2d(1.72031418632092, -0.636432504802847),
                                                            Eigen::Vector2d(1.68790042040768, -0.659928244801852),
                                                            Eigen::Vector2d(1.65431292522090, -0.683288249793493),
                                                            Eigen::Vector2d(1.61955629600467, -0.706775602209756),
                                                            Eigen::Vector2d(1.58362182566213, -0.730622555186019),
                                                            Eigen::Vector2d(1.54648059123728, -0.755063594997916),
                                                            Eigen::Vector2d(1.50809862284400, -0.780297319798962),
                                                            Eigen::Vector2d(1.46843256790670, -0.806522132034378),
                                                            Eigen::Vector2d(1.42742774220559, -0.833945263101491),
                                                            Eigen::Vector2d(1.38501812987668, -0.862782773349728),
                                                            Eigen::Vector2d(1.34112638341180, -0.893259552080625),
                                                            Eigen::Vector2d(1.29566382365855, -0.925609317547819),
                                                            Eigen::Vector2d(1.24853043982036, -0.960074616957054),
                                                            Eigen::Vector2d(1.19961488945645, -0.996906826466176),
                                                            Eigen::Vector2d(1.14879459404322, -1.03636294964786),
                                                            Eigen::Vector2d(1.09593326975624, -1.07871790115455),
                                                            Eigen::Vector2d(1.04087463005077, -1.12432614026943),
                                                            Eigen::Vector2d(0.983445313363567, -1.17354643642074),
                                                            Eigen::Vector2d(0.923455416382797, -1.22673024463708),
                                                            Eigen::Vector2d(0.860698494048062, -1.28422170554738),
                                                            Eigen::Vector2d(0.794951559550375, -1.34635764538093),
                                                            Eigen::Vector2d(0.725975084332169, -1.41346757596738),
                                                            Eigen::Vector2d(0.653512998087296, -1.48587369473672),
                                                            Eigen::Vector2d(0.577292688761027, -1.56389088471928),
                                                            Eigen::Vector2d(0.497055317138652, -1.64770181864661),
                                                            Eigen::Vector2d(0.412648926485308, -1.73685820919140),
                                                            Eigen::Vector2d(0.323610464411226, -1.83181741634822),
                                                            Eigen::Vector2d(0.229535910505275, -1.93251266722577),
                                                            Eigen::Vector2d(0.130156092757214, -2.03805420185905),
                                                            Eigen::Vector2d(0.0253366875576873, -2.14672927320942),
                                                            Eigen::Vector2d(-0.0849217803017726, -2.25600214716460),
                                                            Eigen::Vector2d(-0.200483937628741, -2.36251410253865),
                                                            Eigen::Vector2d(-0.321079562829912, -2.46208343107195),
                                                            Eigen::Vector2d(-0.446303585911086, -2.54970543743128),
                                                            Eigen::Vector2d(-0.575650122742122, -2.61962709227773),
                                                            Eigen::Vector2d(-0.708303631763921, -2.66531969869566),
                                                            Eigen::Vector2d(-0.842420185882371, -2.67953717100709),
                                                            Eigen::Vector2d(-0.976053546925297, -2.65691858019753),
                                                            Eigen::Vector2d(-1.10729144985891, -2.59428960927815),
                                                            Eigen::Vector2d(-1.23425560278782, -2.49066255328583),
                                                            Eigen::Vector2d(-1.35510168695500, -2.34723631928315),
                                                            Eigen::Vector2d(-1.46801935674183, -2.16739642635835),
                                                            Eigen::Vector2d(-1.57121802763510, -1.95703503110527),
                                                            Eigen::Vector2d(-1.66315568201306, -1.72823558615580),
                                                            Eigen::Vector2d(-1.74339503463771, -1.49099497594566),
                                                            Eigen::Vector2d(-1.81189527188296, -1.25334898486254),
                                                            Eigen::Vector2d(-1.86885805946646, -1.02262792458842),
                                                            Eigen::Vector2d(-1.91472754244954, -0.805456634099574),
                                                            Eigen::Vector2d(-1.95019034523728, -0.607754479666605),
                                                            Eigen::Vector2d(-1.97619804836587, -0.434282613516542),
                                                            Eigen::Vector2d(-1.99407882912730, -0.282620718474227),
                                                            Eigen::Vector2d(-2.00486430481622, -0.150287914299912),
                                                            Eigen::Vector2d(-2.00944662454658, -0.0357380949963045),
                                                            Eigen::Vector2d(-2.00866784052434, 0.0627654042459567),
                                                            Eigen::Vector2d(-2.00331990804734, 0.147149807048304),
                                                            Eigen::Vector2d(-1.99411732626979, 0.219526380806077),
                                                            Eigen::Vector2d(-1.98157063209309, 0.281867412729356),
                                                            Eigen::Vector2d(-1.96610189775093, 0.335809351674559),
                                                            Eigen::Vector2d(-1.94809105485674, 0.382794793342435),
                                                            Eigen::Vector2d(-1.92787517261359, 0.424076426873125),
                                                            Eigen::Vector2d(-1.90573953610047, 0.460734400531667),
                                                            Eigen::Vector2d(-1.88187471160698, 0.493697585832517),
                                                            Eigen::Vector2d(-1.85643122553052, 0.523712531063558),
                                                            Eigen::Vector2d(-1.82953826898319, 0.551427944792877),
                                                            Eigen::Vector2d(-1.80130301828363, 0.577399619160707),
                                                            Eigen::Vector2d(-1.77181032020195, 0.602093947720719),
                                                            Eigen::Vector2d(-1.74111400964313, 0.625913163380626),
                                                            Eigen::Vector2d(-1.70924145334914, 0.649183456284321),
                                                            Eigen::Vector2d(-1.67620852528128, 0.672200377032782),
                                                            Eigen::Vector2d(-1.64202043627092, 0.695236897727251),
                                                            Eigen::Vector2d(-1.60667173401948, 0.718543411969234),
                                                            Eigen::Vector2d(-1.57014630309845, 0.742347734860504),
                                                            Eigen::Vector2d(-1.53241732303089, 0.766855254217985),
                                                            Eigen::Vector2d(-1.49344036453196, 0.792277706932205),
                                                            Eigen::Vector2d(-1.45316407730012, 0.818815232948530),
                                                            Eigen::Vector2d(-1.41153111557671, 0.846660327132370),
                                                            Eigen::Vector2d(-1.36847185213254, 0.876019897101917),
                                                            Eigen::Vector2d(-1.32390437826796, 0.907115263228145),
                                                            Eigen::Vector2d(-1.27773450381283, 0.940182158634809),
                                                            Eigen::Vector2d(-1.22985575712652, 0.975470729198444),
                                                            Eigen::Vector2d(-1.18014938509794, 1.01324553354836),
                                                            Eigen::Vector2d(-1.12848435314549, 1.05378554306667),
                                                            Eigen::Vector2d(-1.07471732837234, 1.09738417004744),
                                                            Eigen::Vector2d(-1.01868832833441, 1.14435860834885),
                                                            Eigen::Vector2d(-0.960219224918902, 1.19505274841269),
                                                            Eigen::Vector2d(-0.899115258278239, 1.24982181395314),
                                                            Eigen::Vector2d(-0.835163273045814, 1.30903290393871)};
using MeasurementType                                      = Eigen::Matrix<double, 1, 1>;
static const std::array<MeasurementType, 101> measurements = {
    MeasurementType(1.41950444107972),    MeasurementType(3.05283113500662),   MeasurementType(1.31562902284910),
    MeasurementType(0.997734921435315),   MeasurementType(1.22317216069731),   MeasurementType(1.45104339243095),
    MeasurementType(1.44916178431237),    MeasurementType(2.06339677800307),   MeasurementType(1.72195383158608),
    MeasurementType(2.35251017984521),    MeasurementType(1.13758715411292),   MeasurementType(2.45821035462853),
    MeasurementType(0.578953962787527),   MeasurementType(2.43761956273235),   MeasurementType(1.53352456818439),
    MeasurementType(1.81381988145109),    MeasurementType(0.200234781374404),  MeasurementType(0.699659342503683),
    MeasurementType(2.41589834122634),    MeasurementType(2.94986227453284),   MeasurementType(1.32252096603566),
    MeasurementType(1.86485688535241),    MeasurementType(2.56451677252815),   MeasurementType(2.48362021303660),
    MeasurementType(0.222481906513196),   MeasurementType(0.686611949943641),  MeasurementType(1.01548108836852),
    MeasurementType(1.95185264477098),    MeasurementType(0.446105603787606),  MeasurementType(0.952557110639149),
    MeasurementType(1.63244604339666),    MeasurementType(1.01247688931802),   MeasurementType(1.10983006882242),
    MeasurementType(0.736674796266488),   MeasurementType(1.07524139971197),   MeasurementType(1.02750403456628),
    MeasurementType(0.510074652985162),   MeasurementType(1.04128160500475),   MeasurementType(0.628361191801920),
    MeasurementType(0.295319621277568),   MeasurementType(0.410928595858317),  MeasurementType(0.174591861010451),
    MeasurementType(0.121129239708858),   MeasurementType(0.0322725917481610), MeasurementType(-0.0452595206149492),
    MeasurementType(-0.169494877079244),  MeasurementType(-0.152876202535486), MeasurementType(-0.309465242157101),
    MeasurementType(-0.814093938562644),  MeasurementType(-0.238963726142840), MeasurementType(-0.632176504716999),
    MeasurementType(-0.963633638072323),  MeasurementType(-0.376237691510885), MeasurementType(-1.37716213599798),
    MeasurementType(-0.131734802488156),  MeasurementType(-1.59915247716806),  MeasurementType(-1.87046013578142),
    MeasurementType(-0.718515356889421),  MeasurementType(-1.36508448350099),  MeasurementType(-2.29346522942158),
    MeasurementType(-1.63779457028304),   MeasurementType(-0.324093827438352), MeasurementType(-1.98571378934842),
    MeasurementType(-2.22628217705972),   MeasurementType(-2.05076140006880),  MeasurementType(-2.39347881695105),
    MeasurementType(-2.38945087335498),   MeasurementType(-3.17603874489820),  MeasurementType(-1.06279765777621),
    MeasurementType(-0.401001820498828),  MeasurementType(-2.70473603679768),  MeasurementType(-1.67751879293312),
    MeasurementType(-0.889945939781879),  MeasurementType(-2.88646633022762),  MeasurementType(-1.47321377167750),
    MeasurementType(-2.63824624916142),   MeasurementType(-2.16960369477233),  MeasurementType(-3.01787696167607),
    MeasurementType(-2.16134730916625),   MeasurementType(-2.28354745732726),  MeasurementType(-1.25548768443738),
    MeasurementType(-1.25424509653701),   MeasurementType(-2.88409421060811),  MeasurementType(-1.38573935340991),
    MeasurementType(-1.72463024885456),   MeasurementType(-0.420956973486694), MeasurementType(-1.93083752178450),
    MeasurementType(-1.40734574623157),   MeasurementType(-1.73446796614902),  MeasurementType(-1.25680458659552),
    MeasurementType(-0.704189303383484),  MeasurementType(-2.31905323471143),  MeasurementType(-1.37853985767260),
    MeasurementType(-0.0905469452467918), MeasurementType(-1.29155787143731),  MeasurementType(-0.834336476324293),
    MeasurementType(-1.08344978673423),   MeasurementType(-1.69966623692842),  MeasurementType(-0.777801698154316),
    MeasurementType(-1.57714784288996),   MeasurementType(-0.971672418368469)};

static constexpr int number_of_particles = 1000;
static constexpr double deltaT           = 0.05;
static const MeasurementType measurement_noise(0.2);
static const Eigen::Vector2d process_noise(0.02, 0.1);
static const Eigen::Vector2d initial_state(2.0, 0.0);
static const Eigen::Matrix2d initial_covariance = Eigen::Matrix2d::Identity() * 0.01;


TEST_CASE("Test the UKF", "[utility][math][filter][UKF]") {

    utility::math::filter::UKF<double, shared::tests::VanDerPolModel> model_filter;

    INFO("Configuring the UKF with")
    INFO("    Time step.........: " << deltaT);
    INFO("    Process Noise.....: " << process_noise.transpose());
    INFO("    Initial State.....: " << initial_state.transpose());
    INFO("    Initial Covariance: \n" << initial_covariance);
    model_filter.model.process_noise = process_noise;
    model_filter.set_state(initial_state, initial_covariance);

    INFO("Feeding noisy measurements into the filter")
    std::array<double, 100> innovations;
    std::array<std::pair<utility::math::filter::UKF<double, shared::tests::VanDerPolModel>::StateVec,
                         utility::math::filter::UKF<double, shared::tests::VanDerPolModel>::StateMat>,
               100>
        actual_state;
    for (size_t time_count = 0; time_count < 100; ++time_count) {
        model_filter.measure(measurements[time_count], measurement_noise);
        model_filter.time(deltaT);
        innovations[time_count]  = measurements[time_count].x() - model_filter.get().x();
        actual_state[time_count] = std::make_pair(model_filter.get(), model_filter.getCovariance());
    }

    INFO("Calculating statistics")

    double count_x1                  = 0.0;
    double count_x2                  = 0.0;
    double mean_innovations          = 0.0;
    double mean_x1_boundary          = 0.0;
    double mean_x2_boundary          = 0.0;
    Eigen::Vector2d mean_state_error = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < actual_state.size(); ++i) {
        Eigen::Vector2d state_error = true_state[i] - actual_state[i].first;
        double covariance_bounds_x1 = std::sqrt(actual_state[i].second(0, 0));
        double covariance_bounds_x2 = std::sqrt(actual_state[i].second(1, 1));
        mean_x1_boundary += covariance_bounds_x1;
        mean_x2_boundary += covariance_bounds_x2;

        count_x1 += (std::abs(state_error.x()) - covariance_bounds_x1) > 0.0 ? 1.0 : 0.0;
        count_x2 += (std::abs(state_error.y()) - covariance_bounds_x2) > 0.0 ? 1.0 : 0.0;

        mean_innovations += innovations[i];
        mean_state_error += state_error;
    }

    mean_innovations /= innovations.size();
    mean_state_error /= actual_state.size();
    mean_x1_boundary /= actual_state.size();
    mean_x2_boundary /= actual_state.size();

    double percentage_x1 = 100.0 * count_x1 / actual_state.size();
    double percentage_x2 = 100.0 * count_x2 / actual_state.size();

    INFO("The mean of the innovations is: " << mean_innovations << ". This should be small.");
    INFO("The mean of the state errors is: " << mean_state_error.transpose() << ". This should be small.");
    INFO(percentage_x1 << "% of state 1 estimates exceed the 1\u03C3 boundary");
    INFO(percentage_x2 << "% of state 2 estimates exceed the 1\u03C3 boundary");
    INFO("The mean 1\u03C3 boundary for state 1 is [" << -mean_x1_boundary << ", " << mean_x1_boundary << "]");
    INFO("The mean 1\u03C3 boundary for state 2 is [" << -mean_x2_boundary << ", " << mean_x2_boundary << "]");

    REQUIRE(percentage_x1 <= 30.0);
}


TEST_CASE("Test the ParticleFilter", "[utility][math][filter][ParticleFilter]") {

    utility::math::filter::ParticleFilter<double, shared::tests::VanDerPolModel> model_filter;

    INFO("Configuring the ParticleFilter with")
    INFO("    Time step..........: " << deltaT);
    INFO("    Number of Particles: " << number_of_particles)
    INFO("    Process Noise......: " << process_noise.transpose());
    INFO("    Initial State......: " << initial_state.transpose());
    INFO("    Initial Covariance.: \n" << initial_covariance);
    model_filter.model.process_noise = process_noise;
    model_filter.set_state(initial_state, initial_covariance, number_of_particles);

    INFO("Feeding noisy measurements into the filter")
    std::array<double, 100> innovations;
    std::array<std::pair<utility::math::filter::UKF<double, shared::tests::VanDerPolModel>::StateVec,
                         utility::math::filter::UKF<double, shared::tests::VanDerPolModel>::StateMat>,
               100>
        actual_state;
    for (size_t time_count = 0; time_count < 100; ++time_count) {
        model_filter.measure(measurements[time_count], measurement_noise);
        model_filter.time(deltaT);
        innovations[time_count]  = measurements[time_count].x() - model_filter.get().x();
        actual_state[time_count] = std::make_pair(model_filter.get(), model_filter.getCovariance());
    }

    INFO("Calculating statistics")

    double count_x1                  = 0.0;
    double count_x2                  = 0.0;
    double mean_innovations          = 0.0;
    double mean_x1_boundary          = 0.0;
    double mean_x2_boundary          = 0.0;
    Eigen::Vector2d mean_state_error = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < actual_state.size(); ++i) {
        Eigen::Vector2d state_error = true_state[i] - actual_state[i].first;
        double covariance_bounds_x1 = std::sqrt(actual_state[i].second(0, 0));
        double covariance_bounds_x2 = std::sqrt(actual_state[i].second(1, 1));
        mean_x1_boundary += covariance_bounds_x1;
        mean_x2_boundary += covariance_bounds_x2;

        count_x1 += (std::abs(state_error.x()) - covariance_bounds_x1) > 0.0 ? 1.0 : 0.0;
        count_x2 += (std::abs(state_error.y()) - covariance_bounds_x2) > 0.0 ? 1.0 : 0.0;

        mean_innovations += innovations[i];
        mean_state_error += state_error;
    }

    mean_innovations /= innovations.size();
    mean_state_error /= actual_state.size();
    mean_x1_boundary /= actual_state.size();
    mean_x2_boundary /= actual_state.size();

    double percentage_x1 = 100.0 * count_x1 / actual_state.size();
    double percentage_x2 = 100.0 * count_x2 / actual_state.size();

    INFO("The mean of the innovations is: " << mean_innovations << ". This should be small.");
    INFO("The mean of the state errors is: " << mean_state_error.transpose() << ". This should be small.");
    INFO(percentage_x1 << "% of state 1 estimates exceed the 1\u03C3 boundary");
    INFO(percentage_x2 << "% of state 2 estimates exceed the 1\u03C3 boundary");
    INFO("The mean 1\u03C3 boundary for state 1 is [" << -mean_x1_boundary << ", " << mean_x1_boundary << "]");
    INFO("The mean 1\u03C3 boundary for state 2 is [" << -mean_x2_boundary << ", " << mean_x2_boundary << "]");

    REQUIRE(percentage_x1 <= 30.0);
}
