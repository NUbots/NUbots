#include "SelfLocalisationTests.h"
#include "SelfLocalisation.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include <iostream>
#include <QTime>
#include "Localisation/Filters/KFBuilder.h"
#include "Localisation/Filters/RobotModel.h"

IWeightedKalmanFilter* robotFilter()
{
    return KFBuilder::getNewFilter(KFBuilder::kseq_ukf_filter, KFBuilder::krobot_model);
}

IWeightedKalmanFilter* newRobotModel()
{
    IWeightedKalmanFilter* filter = robotFilter();

    // set initial settings.
    filter->enableOutlierFiltering();
    filter->setOutlierThreshold(15.f);
    filter->enableWeighting();
    filter->setActive();
    return filter;
}

IWeightedKalmanFilter* newRobotModel(IWeightedKalmanFilter* filter)
{
    IWeightedKalmanFilter* new_filter = robotFilter();

    // This should be fixed in a different way. The default copy copies the pointer value and would otherwise lose the model.
    IKFModel* temp_model = new_filter->model();
    *new_filter = *filter;
    new_filter->setModel(temp_model);

    // set initial settings.
    new_filter->enableOutlierFiltering();
    new_filter->setOutlierThreshold(15.f);
    new_filter->enableWeighting();

    new_filter->initialiseEstimate(filter->estimate());
    new_filter->setFilterWeight(filter->getFilterWeight());

    return new_filter;
}

IWeightedKalmanFilter* newRobotModel(IWeightedKalmanFilter* filter, const StationaryObject& measured_object, const MeasurementError &error, double timestamp)
{
    Matrix meas_noise = error.errorCovariance();

    IWeightedKalmanFilter* new_filter = robotFilter();
    // set initial settings.
    new_filter->enableOutlierFiltering();
    new_filter->setOutlierThreshold(15.f);
    new_filter->enableWeighting();

    new_filter->initialiseEstimate(filter->estimate());
    new_filter->setFilterWeight(filter->getFilterWeight());

    Matrix meas(2,1,false);
    meas[0][0] = measured_object.measuredDistance() * cos(measured_object.measuredElevation());
    meas[1][0] = measured_object.measuredBearing();

    Matrix args(2,1,false);
    args[0][0] = measured_object.X();
    args[1][0] = measured_object.Y();

    bool success = new_filter->measurementUpdate(meas, meas_noise, args, RobotModel::klandmark_measurement);
    new_filter->setActive(success);

    if(new_filter->active())
    {
        new_filter->m_creation_time = timestamp;
        new_filter->m_parent_history_buffer = filter->m_parent_history_buffer;
        new_filter->m_parent_history_buffer.push_back(filter->id());
        new_filter->m_parent_id = filter->id();
        new_filter->m_split_option = measured_object.getID();
        //new_filter->m_previous_decisions[object.getID()] = measured_object.getID();
    }

    return new_filter;
}

bool RunTests()
{
    bool maxLikely, viterbi, merge, nscan;
    maxLikely = MaxLikelyhoodTest();
    std::cout << "Max Likelyhood Test..." << (maxLikely ? "Success.":"Failed.") << std::endl;
    viterbi = ViterbiTest();
    std::cout << "Viterbi Test..." << (viterbi ? "Success.":"Failed.") << std::endl;
    merge = MergingTest();
    std::cout << "Merge Test..." << (merge ? "Success.":"Failed.") << std::endl;
    nscan = NscanTest();
    std::cout << "N-Scan Test..." << (nscan ? "Success.":"Failed.") << std::endl;
    return maxLikely and viterbi and merge and nscan;
}

bool timingTest()
{
    IWeightedKalmanFilter *theModel = newRobotModel();
    bool success = true;
    const unsigned int total_updates = 100000;

    // make the objects.
    FieldObjects objects;
    StationaryObject* leftYGoal = &objects.stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
    StationaryObject* rightYGoal = &objects.stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];

    Vector3<float> left_measure(130,-0.19, 0);
    Vector3<float> right_measure(161.92, -1.17, 0);
    Vector3<float> empty_3f;
    Vector2<float> empty_2f;
    Vector2<int> empty_2i;

    QTime seq_update;
    QTime comb_update;

    // add mesurement
    leftYGoal->UpdateVisualObject(left_measure, empty_3f, empty_2f, empty_2i, empty_2i, 100.00f);
    rightYGoal->UpdateVisualObject(right_measure, empty_3f, empty_2f, empty_2i, empty_2i, 100.00f);

    Matrix initial_mean(3,1,false);
    initial_mean[0][0] = 0;
    initial_mean[1][0] = 0;
    initial_mean[2][0] = 0;

    Matrix initial_cov(3, 3, false);
    initial_cov[0][0] = 150;
    initial_cov[1][1] = 200;
    initial_cov[2][2] = 6.5;

    Matrix leftMeasure(2,1,false);
    leftMeasure[0][0] = 130.f;
    leftMeasure[1][0] = -0.19f;

    Matrix leftLoc(2,1,false);
    leftLoc[0][0] = leftYGoal->X();
    leftLoc[1][0] = leftYGoal->Y();

    Matrix rightMeasure(2,1,false);
    rightMeasure[0][0] = 161.92;
    rightMeasure[1][0] = -1.17;

    Matrix rightLoc(2,1,false);
    rightLoc[0][0] = rightYGoal->X();
    rightLoc[1][0] = rightYGoal->Y();

    theModel->initialiseEstimate(MultivariateGaussian(initial_mean, initial_cov));

    seq_update.start();
    for (unsigned int i = 0; i < total_updates; ++i)
    {
        MeasurementError temp_error;
        temp_error.setDistance(100 + 0.04 * pow(leftYGoal->estimatedDistance(),2));
        temp_error.setHeading(0.01);
        theModel->measurementUpdate(leftMeasure, temp_error.errorCovariance(), leftLoc, RobotModel::klandmark_measurement);

        temp_error.setDistance(100 + 0.04 * pow(rightYGoal->estimatedDistance(),2));
        temp_error.setHeading(0.01);
        theModel->measurementUpdate(rightMeasure, temp_error.errorCovariance(), rightLoc, RobotModel::klandmark_measurement);
    }
    int seq_time = seq_update.elapsed();
    std::cout << seq_time << std::endl;

//    theModel.setMean(initial_mean);
//    theModel.setCovariance(initial_cov);
//    comb_update.start();
//    for (unsigned int i = 0; i < total_updates; ++i)
//    {
//        const unsigned int num_objects = 2;
//        Matrix locations(4, 1, false);
//        Matrix measurements(4, 1, false);
//        Matrix R_measurement(4, 4, false);

//        locations[0][0] = leftYGoal->X();
//        locations[1][0] = leftYGoal->Y();
//        locations[2][0] = rightYGoal->X();
//        locations[3][0] = rightYGoal->Y();

//        measurements[0][0] = leftYGoal->measuredDistance();
//        measurements[1][0] = leftYGoal->measuredBearing();
//        measurements[2][0] = rightYGoal->measuredDistance();
//        measurements[3][0] = rightYGoal->measuredBearing();

//        R_measurement[0][0] = 100 + 0.04 * pow(leftYGoal->measuredDistance(), 2);
//        R_measurement[1][1] = 0.01;
//        R_measurement[2][2] = 100 + 0.04 * pow(rightYGoal->measuredDistance(), 2);
//        R_measurement[3][3] = 0.01;

//        theModel.MultipleObjectUpdate(locations, measurements, R_measurement);
//    }
//    int comb_time = comb_update.elapsed();
//    std::cout << comb_time << std::endl;

    return success;
}

bool MergingTest()
{
    LocalisationSettings settings;
    settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    settings.setPruneMethod(LocalisationSettings::prune_merge);
    SelfLocalisation loc(2,settings);
    bool success = true;
    std::list<IWeightedKalmanFilter*> models;

    // --- Test for correct merge ---

    // Make two models that are at close positions, these should be merged to one model.

    // Create the models.

    IWeightedKalmanFilter *temp = newRobotModel();
    temp->setFilterWeight(0.5);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(99.5f, 0.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.5);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(100.0f, 1.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    loc.setModels(models);

    loc.PruneModels();

    // Success conditions. There should be one model with 100% probability
    success = loc.getNumActiveModels() == 1;
    success = success && loc.getBestModel()->getFilterWeight() == 1.0;

    // --- Test for correct non-merge. ---
    loc.clearModels(); // Reset localisation.
    models.clear();

    // Add models
    temp = newRobotModel();
    temp->setFilterWeight(0.5);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(100.0f, 0.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.5);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(-100.0f, 0.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    loc.setModels(models);

    loc.PruneModels();

    // Success conditions. There should be two models that remain as they were before the merge.
    success = success && loc.getNumActiveModels() == 2;
    success = success && (loc.allModels().front()->getFilterWeight() == 0.5) && (loc.allModels().back()->getFilterWeight() == 0.5);
    return success;
}

bool MaxLikelyhoodTest()
{
    // Configure localisation
    LocalisationSettings settings;
    settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    settings.setPruneMethod(LocalisationSettings::prune_max_likelyhood);
    SelfLocalisation loc(2,settings);

    std::list<IWeightedKalmanFilter*> models;
    // Create the models.
    IWeightedKalmanFilter* temp;

    temp = newRobotModel();
    temp->setFilterWeight(0.5);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(300.0f, 0.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    unsigned int model_to_remain = temp->id();

    temp = newRobotModel();
    temp->setFilterWeight(0.2);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(200.0f, 40.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.3);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(100.0f, 30.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    loc.setModels(models);

    // Do the pruning.
    loc.PruneModels();

    // Check the results.
    bool correct_number_of_models = loc.getNumActiveModels();
    bool correct_model_remains = loc.getBestModel()->id() == model_to_remain;

    return (correct_number_of_models and correct_model_remains);
}

bool ViterbiTest()
{
    // Configure localisation
    LocalisationSettings settings;
    settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    settings.setPruneMethod(LocalisationSettings::prune_viterbi);
    SelfLocalisation loc(2,settings);

    std::list<IWeightedKalmanFilter*> models;
    // Create the models.
    IWeightedKalmanFilter* temp;

    temp = newRobotModel();
    temp->setFilterWeight(0.2);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(300.0f, 0.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    unsigned int model_to_remain = temp->id();

    temp = newRobotModel();
    temp->setFilterWeight(0.1);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(200.0f, 40.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.1);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(100.0f, 30.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.3);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(0.0f, 30.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.14);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(-100.0f, 30.0f, -0.6f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    temp = newRobotModel();
    temp->setFilterWeight(0.16);
    temp->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(-50.0f, 30.0f, -1.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    temp->setActive();
    models.push_back(temp);

    loc.setModels(models);

    bool success = true;
    for(unsigned int it = 5; it > 0; --it)
    {
        // Do the pruning.
        loc.PruneViterbi(it);
        success = success && (loc.getNumActiveModels() ==  it);
    }

    return success;
}

bool NscanTest()
{
    // Make the localisation
    LocalisationSettings settings;
    settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    settings.setPruneMethod(LocalisationSettings::prune_nscan);
    SelfLocalisation loc(2,settings);

    // Firstly we will make up the measurments that will be used.
    AmbiguousObject ambPost(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");   // Unknonw blue post
    ambPost.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);                       // Could be left ...
    ambPost.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);                      // or right

    StationaryObject* leftGoal = &Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
    StationaryObject* rightGoal = &Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];

    // Because we are only really interested in the cropping based on the alpha
    // we are not really worried about what the values are.
    Vector3<float> sphericalPos;
    sphericalPos.x = 100.0f;    // Distance
    sphericalPos.y = 0.0f;      // Heading
    sphericalPos.z = 0.0f;      // Elevation
    Vector3<float> sphericalError;
    sphericalError.x = 20.0f;
    sphericalError.y = 0.2f;
    sphericalError.z = 0.2f;
    Vector2<float> imagePosAngle;
    imagePosAngle.x = 0;
    imagePosAngle.y = 0;
    Vector2<int> imagePos, sizeOnScreen;
    imagePos.x = 0;
    imagePos.y = 0;
    sizeOnScreen.x = 10;
    sizeOnScreen.y = 10;

    ambPost.UpdateVisualObject( sphericalPos,
                                sphericalError,
                                imagePosAngle,
                                imagePos,
                                sizeOnScreen,
                                100);

    // Error for updates
    MeasurementError error;
    error.setDistance(10 + 0.2f*0.2f * pow(ambPost.measuredDistance() * cos(ambPost.measuredElevation()),2));
    error.setHeading(0.2f);

    // Now we need a decsion tree, model numbers are relative. First model left option, second model right option
    //  N = 2  *  N = 1  *  N = 0  *  alpha  *  Sequence  *
    // Model 0 - Model 2 - Model 6     0.1      0-2-6
    //         |         + Model 7     0.2      0-2-7
    //         - Model 3 - Model 8     0.05     0-3-8
    //                   + Model 9     0.1     0-3-9
    // Model 1 - Model 4 - Model 10    0.12     1-4-10
    //         |         + Model 11    0.13     1-4-11
    //         - Model 5 - Model 12    0.17     1-5-12
    //                   + Model 13    0.13     1-5-13

    // --- Expected results ---
    // N = 2
    // Sum for model 0 branch = 0.1 + 0.2 + 0.05 + 0.1 = 0.45
    // Sum for model 1 branch = 0.12 + 0.13 + 0.17 + 0.13 = 0.55 <- This branch should be chosen.
    //
    // Remaining models should be models 10,11,12, and 13
    // ------------------------
    // N = 1 (before normalisation)
    // Sum for model 4 branch = 0.12 + 0.13 = 0.25
    // Sum for model 5 branch = 0.17 + 0.13 = 0.3   <- This branch should be chosen.
    //
    // Remaining models should be models 12 and 13
    // ------------------------

    // Make the first two models
    std::list<IWeightedKalmanFilter*> example_tree;
    // N=2
    IWeightedKalmanFilter* model0 = newRobotModel();
    model0->setFilterWeight(0.5);
    model0->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(0.0f, 30.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    model0->setActive();

    IWeightedKalmanFilter* model1 = newRobotModel();
    model1->setFilterWeight(0.5);
    model0->initialiseEstimate(MultivariateGaussian(SelfLocalisation::mean_matrix(0.0f, 30.0f, 0.0f), SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f)));
    model1->setActive();

    // Write measurement from ambiguous object
    leftGoal->CopyObject(ambPost);
    rightGoal->CopyObject(ambPost);

    // N=1
    IWeightedKalmanFilter* model2 = newRobotModel(model0, *leftGoal, error, 100);
    IWeightedKalmanFilter* model3 = newRobotModel(model0, *rightGoal, error, 100);
    IWeightedKalmanFilter* model4 = newRobotModel(model1, *leftGoal, error, 100);
    IWeightedKalmanFilter* model5 = newRobotModel(model1, *rightGoal, error, 100);

    // N=0
    IWeightedKalmanFilter* model6 = newRobotModel(model2, *leftGoal, error, 200);
    model6->setActive();
    model6->setFilterWeight(0.1f);
    example_tree.push_back(model6);

    IWeightedKalmanFilter* model7 = newRobotModel(model2, *rightGoal, error, 200);
    model7->setActive();
    model7->setFilterWeight(0.2f);
    example_tree.push_back(model7);

    IWeightedKalmanFilter* model8 = newRobotModel(model3, *leftGoal, error, 200);
    model8->setActive();
    model8->setFilterWeight(0.05f);
    example_tree.push_back(model8);

    IWeightedKalmanFilter* model9 = newRobotModel(model3, *rightGoal, error, 200);
    model9->setActive();
    model9->setFilterWeight(0.1f);
    example_tree.push_back(model9);

    IWeightedKalmanFilter* model10 = newRobotModel(model4, *leftGoal, error, 200);
    model10->setActive();
    model10->setFilterWeight(0.12f);
    example_tree.push_back(model10);

    IWeightedKalmanFilter* model11 = newRobotModel(model4, *rightGoal, error, 200);
    model11->setActive();
    model11->setFilterWeight(0.13f);
    example_tree.push_back(model11);

    IWeightedKalmanFilter* model12 = newRobotModel(model5, *leftGoal, error, 200);
    model12->setActive();
    model12->setFilterWeight(0.17f);
    example_tree.push_back(model12);

    IWeightedKalmanFilter* model13 = newRobotModel(model2, *rightGoal, error, 200);
    model13->setActive();
    model13->setFilterWeight(0.13f);
    example_tree.push_back(model13);

    // Can delete non N=0 level models
    delete model0;
    delete model1;
    delete model2;
    delete model3;
    delete model4;
    delete model5;

//    std::cout << "Model6 History. ";
//    for (unsigned int i = 0; i < 10; i++)
//    {
//        if(i < model6->history_depth())
//        {
//            std::cout << model6->history(i) << " ";
//        }
//    }
//    std::cout << std::endl;

//    std::cout << "Tree" << "  (" << example_tree.size() << ")" << std::endl;
//    for (ModelContainer::const_iterator mod_it = example_tree.begin(); mod_it != example_tree.end(); ++mod_it)
//    {
//        std::cout << (*mod_it)->summary(false);
//    }

    std::list<IWeightedKalmanFilter*> tree_copy = example_tree;
    // Make a copy so we can perform two tests
//    for(std::list<IWeightedKalmanFilter*>::iterator model_it = example_tree.begin(); model_it != example_tree.end(); ++model_it)
//    {
//        IWeightedKalmanFilter* temp = new Model(*(*model_it));
//        tree_copy.push_back(temp);
//    }

//    std::cout << "Tree Copy" << "  (" << tree_copy.size() << ")" << std::endl;
//    for (ModelContainer::const_iterator mod_it = tree_copy.begin(); mod_it != tree_copy.end(); ++mod_it)
//    {
//        std:: std::cout << (*mod_it)->summary(false);
//    }
//    std::cout << std::flush;
    // Do the nscan prune at N=2
    loc.setModels(tree_copy);
    tree_copy.clear();
    loc.PruneNScan(2);

    // Check that the prune performed as expected.
    // From above.
    // --- Expected results ---
    // N = 2
    // Sum for model 0 branch = 0.1 + 0.2 + 0.05 + 0.1 = 0.45
    // Sum for model 1 branch = 0.12 + 0.13 + 0.17 + 0.13 = 0.55 <- This branch should be chosen.
    //
    // Remaining models should be models 10,11,12, and 13
    // ------------------------

    bool correct_num_models = loc.getNumActiveModels() == 4;
    tree_copy = loc.allModels();
//    std::cout << "Size - " << tree_copy.size() << std::endl;
//    for (ModelContainer::const_iterator mod_it = tree_copy.begin(); mod_it != tree_copy.end(); ++mod_it)
//    {
//        std:: std::cout << (*mod_it)->summary(false) <<std::endl;
//    }
    bool model10_found = false;
    bool model11_found = false;
    bool model12_found = false;
    bool model13_found = false;

    for(std::list<IWeightedKalmanFilter*>::const_iterator model_it = tree_copy.begin(); model_it != tree_copy.end(); ++model_it)
    {
        if((*model_it)->id() == model10->id()) model10_found = true;
        if((*model_it)->id() == model11->id()) model11_found = true;
        if((*model_it)->id() == model12->id()) model12_found = true;
        if((*model_it)->id() == model13->id()) model13_found = true;
    }
    bool all_models_found_n2 = model10_found and model11_found and model12_found and model13_found;

    //return correct_num_models and all_models_found_n2;
    // Do the nscan prune at N=1
    loc.PruneNScan(1);

    // --- Expected results ---
    // N = 1 (before normalisation)
    // Sum for model 4 branch = 0.12 + 0.13 = 0.25
    // Sum for model 5 branch = 0.17 + 0.13 = 0.3   <- This branch should be chosen.
    //
    // Remaining models should be models 12 and 13
    // ------------------------

    correct_num_models = correct_num_models and (loc.getNumActiveModels() == 2);

    model12_found = false;
    model13_found = false;
    tree_copy = loc.allModels();
    for(std::list<IWeightedKalmanFilter*>::const_iterator model_it = tree_copy.begin(); model_it != tree_copy.end(); ++model_it)
    {
        if((*model_it)->id() == model12->id()) model12_found = true;
        if((*model_it)->id() == model13->id()) model13_found = true;
    }
    bool all_models_found_n1 = model12_found and model13_found;

    return correct_num_models and all_models_found_n1 and all_models_found_n2;
}
