#include "SelfLocalisationTests.h"
#include "SelfLocalisation.h"
#include "Infrastructure/NUBlackboard.h"
#include <iostream>

bool RunTests()
{
    bool success = true;
    success = success && MaxLikelyhoodTest();
    std::cout << "Max Likelyhood Test..." << (success ? "Success.":"Failed.") << std::endl;
    success = success && ViterbiTest();
    std::cout << "Viterbi Test..." << (success ? "Success.":"Failed.") << std::endl;
    success = success && MergingTest();
    std::cout << "Merge Test..." << (success ? "Success.":"Failed.") << std::endl;
    success = success && NscanTest();
    std::cout << "N-Scan Test..." << (success ? "Success.":"Failed.") << std::endl;
    return success;
}

bool MergingTest()
{
    LocalisationSettings settings;
    settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    settings.setPruneMethod(LocalisationSettings::prune_merge);
    SelfLocalisation loc(2,settings);
    bool success = true;
    ModelContainer models;
    Model *temp;

    // --- Test for correct merge ---

    // Make two models that are at close positions, these should be merged to one model.

    // Create the models.

    temp = new Model();
    temp->setAlpha(0.5);
    temp->setMean(SelfLocalisation::mean_matrix(102.0f, 0.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.5);
    temp->setMean(SelfLocalisation::mean_matrix(100.0f, 1.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    loc.setModels(models);

    loc.PruneModels();

    // Success conditions. There should be one model with 100% probability
    success = loc.getNumActiveModels() == 1;
    success = success && loc.getBestModel()->alpha() == 1.0;

    // --- Test for correct non-merge. ---
    loc.clearModels(); // Reset localisation.
    models.clear();

    // Add models
    temp = new Model();
    temp->setAlpha(0.5);
    temp->setMean(SelfLocalisation::mean_matrix(100.0f, 0.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.5);
    temp->setMean(SelfLocalisation::mean_matrix(-100.0f, 0.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    loc.setModels(models);

    loc.PruneModels();

    // Success conditions. There should be two models that remain as they were before the merge.
    success = success && loc.getNumActiveModels() == 2;
    success = success && (loc.allModels().front()->alpha() == 0.5) && (loc.allModels().back()->alpha() == 0.5);
    return success;
}

bool MaxLikelyhoodTest()
{
    // Configure localisation
    LocalisationSettings settings;
    settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    settings.setPruneMethod(LocalisationSettings::prune_max_likelyhood);
    SelfLocalisation loc(2,settings);

    ModelContainer models;
    // Create the models.
    Model *temp;

    temp = new Model();
    temp->setAlpha(0.5);
    temp->setMean(SelfLocalisation::mean_matrix(300.0f, 0.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    unsigned int model_to_remain = temp->id();

    temp = new Model();
    temp->setAlpha(0.2);
    temp->setMean(SelfLocalisation::mean_matrix(200.0f, 40.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.3);
    temp->setMean(SelfLocalisation::mean_matrix(100.0f, 30.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
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

    ModelContainer models;
    // Create the models.
    Model *temp;

    temp = new Model();
    temp->setAlpha(0.2);
    temp->setMean(SelfLocalisation::mean_matrix(300.0f, 0.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    unsigned int model_to_remain = temp->id();

    temp = new Model();
    temp->setAlpha(0.1);
    temp->setMean(SelfLocalisation::mean_matrix(200.0f, 40.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.1);
    temp->setMean(SelfLocalisation::mean_matrix(100.0f, 30.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.3);
    temp->setMean(SelfLocalisation::mean_matrix(0.0f, 30.0f, 0.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.14);
    temp->setMean(SelfLocalisation::mean_matrix(-100.0f, 30.0f, -0.6f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
    temp->setActive();
    models.push_back(temp);

    temp = new Model();
    temp->setAlpha(0.16);
    temp->setMean(SelfLocalisation::mean_matrix(-50.0f, 30.0f, -1.0f));
    temp->setCovariance(SelfLocalisation::covariance_matrix(50.0f,15.0f,0.2f));
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
    ModelContainer example_tree;
    // N=2
    Model* model0 = new Model();
    Model* model1 = new Model();

    // N=1
    Model* model2 = new Model(*model0, ambPost, *leftGoal, error, 100);
    Model* model3 = new Model(*model0, ambPost, *rightGoal, error, 100);
    Model* model4 = new Model(*model1, ambPost, *leftGoal, error, 100);
    Model* model5 = new Model(*model1, ambPost, *rightGoal, error, 100);

    // N=0
    Model* model6 = new Model(*model2, ambPost, *leftGoal, error, 200);
    model6->setActive();
    model6->setAlpha(0.1f);
    example_tree.push_back(model6);

    Model* model7 = new Model(*model2, ambPost, *rightGoal, error, 200);
    model7->setActive();
    model7->setAlpha(0.2f);
    example_tree.push_back(model7);

    Model* model8 = new Model(*model3, ambPost, *leftGoal, error, 200);
    model8->setActive();
    model8->setAlpha(0.05f);
    example_tree.push_back(model8);

    Model* model9 = new Model(*model3, ambPost, *rightGoal, error, 200);
    model9->setActive();
    model9->setAlpha(0.1f);
    example_tree.push_back(model9);

    Model* model10 = new Model(*model4, ambPost, *leftGoal, error, 200);
    model10->setActive();
    model10->setAlpha(0.12f);
    example_tree.push_back(model10);

    Model* model11 = new Model(*model4, ambPost, *rightGoal, error, 200);
    model11->setActive();
    model11->setAlpha(0.13f);
    example_tree.push_back(model11);

    Model* model12 = new Model(*model5, ambPost, *leftGoal, error, 200);
    model12->setActive();
    model12->setAlpha(0.17f);
    example_tree.push_back(model12);

    Model* model13 = new Model(*model5, ambPost, *rightGoal, error, 200);
    model13->setActive();
    model13->setAlpha(0.13f);
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

    ModelContainer tree_copy;
    // Make a copy so we can perform two tests
    for(ModelContainer::iterator model_it = example_tree.begin(); model_it != example_tree.end(); ++model_it)
    {
        Model* temp = new Model(*(*model_it));
        tree_copy.push_back(temp);
    }

//    std::cout << "Tree Copy" << "  (" << tree_copy.size() << ")" << std::endl;
//    for (ModelContainer::const_iterator mod_it = tree_copy.begin(); mod_it != tree_copy.end(); ++mod_it)
//    {
//        std:: cout << (*mod_it)->summary(false);
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
//        std:: cout << (*mod_it)->summary(false) <<std::endl;
//    }
    bool model10_found = false;
    bool model11_found = false;
    bool model12_found = false;
    bool model13_found = false;

    for(ModelContainer::const_iterator model_it = tree_copy.begin(); model_it != tree_copy.end(); ++model_it)
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
    for(ModelContainer::const_iterator model_it = tree_copy.begin(); model_it != tree_copy.end(); ++model_it)
    {
        if((*model_it)->id() == model12->id()) model12_found = true;
        if((*model_it)->id() == model13->id()) model13_found = true;
    }
    bool all_models_found_n1 = model12_found and model13_found;

    return correct_num_models and all_models_found_n1 and all_models_found_n2;
}
