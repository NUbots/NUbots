#include "visioncontrolwrappertraining.h"
#include <boost/foreach.hpp>

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    data_wrapper = DataWrapper::getInstance();
}

/*!
  * @brief runs the vision system for a single frame - reading from the current stream
  * @return error code.
  */
int VisionControlWrapper::runFrame()
{
    if(!data_wrapper->updateFrame()) {
        return -1;  //failure - do not run vision
    }
    return controller.runFrame(true, true, true, true);
}

/*!
  * @brief runs the vision system for a single frame - using the supplied image
  * @param img The image to use.
  * @return error code.
  */
int VisionControlWrapper::runFrame(NUImage& img, NUSensorsData& sensors)
{
    data_wrapper->updateFrame(img, sensors);
    return controller.runFrame(true, true, true, true);
}

/*!
  * @brief sets the lookup table to use.
  * @param filename The file to load the LUT from
  * @return success.
  */
bool VisionControlWrapper::setLUT(const std::string& filename)
{
    return data_wrapper->loadLUTFromFile(filename);
}

/*!
  * @brief sets the image stream to use.
  * @param filename The file to load the images from
  * @return success.
  */
bool VisionControlWrapper::setImageStream(const std::string& filename)
{
    return data_wrapper->setImageStream(filename);
}

/*!
  * @brief sets the sensor stream to use.
  * @param filename The file to load the sensor data from
  * @return success.
  */
bool VisionControlWrapper::setSensorStream(const std::string& filename)
{
    return data_wrapper->setSensorStream(filename);
}

/*!
  * @brief resets the image stream.
  */
void VisionControlWrapper::restartStream()
{
    data_wrapper->resetStream();
}

/*!
  * @brief prints labels to stream
  * @param out The stream to print to.
  */
void VisionControlWrapper::printLabels(std::ostream& out) const
{
    data_wrapper->printLabels(out);
}

/*!
  * @brief reads labels from stream in VisionFieldObject format
  * @param in The stream to read from.
  * @param labels The resulting labels (output parameter)
  */
bool VisionControlWrapper::readLabels(std::istream &in, std::vector<std::vector<VisionFieldObject *> > &labels) const
{
    return data_wrapper->readLabels(in, labels);
}

///*!
//  * @brief reads labels from stream in ID/parameters format
//  * @param in The stream to read from.
//  * @param labels The resulting labels (output parameter)
//  */
//bool VisionControlWrapper::readLabels(std::istream& in, std::vector< std::vector< pair<VFO_ID, Vector2<double> > > >& labels) const
//{
//    return data_wrapper->readLabels(in, labels);
//}

/*!
  * @brief evaluates a frame given costs for total error and incidence count.
  * @param ground_truth The labels to compare with.
  * @param false_pos_costs A map between field object IDs and false positive costs
  * @param false_neg_costs A map between field object IDs and false negative costs
  * @return a map between field object IDs and total cost|incident number
  */
map<VFO_ID, pair<float, int> > VisionControlWrapper::evaluateFrame(const std::vector<VisionFieldObject *>& ground_truth,
                                                                   const map<VFO_ID, float>& false_pos_costs,
                                                                   const map<VFO_ID, float>& false_neg_costs,
                                                                   bool use_ground_errors)
{
    std::vector<bool> matched_detections(data_wrapper->detections.size(), false);    //tracks what detections have been matched
    //initi
    map<VFO_ID, pair<float, int> > errors;                   //the current erors and incidence counts

    //initialise errors map
    for(int i = 0; i < numVFOIDs(); i++)
        errors[VFOFromInt(i)] = pair<float, int>(0,0);

    //for each label
    BOOST_FOREACH(VisionFieldObject* gt, ground_truth) {
        int d_num = 0;
        pair<int,float> best_match = pair<int,float>(-1,std::numeric_limits<float>::max());
        //look for the best matching detection
        BOOST_FOREACH(const VisionFieldObject* d_vfo, data_wrapper->detections) {
            //if the object is a match and the detection has not already been matched
            if(objectTypesMatch(d_vfo->getID(), gt->getID()) && !matched_detections.at(d_num)) {
                float err;

                if(use_ground_errors)
                    err = d_vfo->findGroundError(gt);
                else
                    err = d_vfo->findScreenError(gt);

                //keep the best match
                if(err < best_match.second) {
                    best_match = pair<int, float>(d_num, err);
                }
            }
            d_num++;
        }

        if(best_match.first == -1) {
            //no detection for gt item - false negative
            //only apply if costs have been provided
            if(!false_neg_costs.empty()) {
                errors.at(gt->getID()).first += false_neg_costs.at(gt->getID());
                errors.at(gt->getID()).second++;
            }
        }
        else {
            //detection - add it's cost to the map and mark it as matched
            errors.at(gt->getID()).first += best_match.second;
            errors.at(gt->getID()).second++;
            matched_detections.at(best_match.first) = true;
        }
    }

    //all remaining unmatched detections are false positives
    for(unsigned int d=0; d<matched_detections.size(); d++) {
        if(!matched_detections.at(d)) {
            //false positive
            //only apply if costs have been provided
            if(!false_pos_costs.empty()) {
                VFO_ID id = data_wrapper->detections.at(d)->getID();
                errors.at(id).first += false_pos_costs.at(id);
                errors.at(id).second++;
            }
        }
    }

    return errors;
}

/*!
  * @brief evaluates a frame for precision and recall.
  * @param ground_truth The labels to compare with.
  * @return a map between field object IDs and total detections | false_pos | false_neg
  */
map<VFO_ID, Vector3<double> > VisionControlWrapper::precisionRecall(const std::vector<VisionFieldObject *> &ground_truth, bool use_ground_errors)
{
    //average distance error 3D for now
    std::vector<bool> matched_detections(data_wrapper->detections.size(), false);
    map<VFO_ID, Vector3<double> > result;    //final result
    map<VFO_ID, double> matches;             //accumulator for detections
    map<VFO_ID, double> false_pos;           //accumulator for false positives
    map<VFO_ID, double> false_neg;           //accumulator for false negatives

    //initialise accumulators
    for(int i = 0; i < numVFOIDs(); i++) {
        VFO_ID vfo_id = VFOFromInt(i);
        matches[vfo_id] = 0;
        false_pos[vfo_id] = 0;
        false_neg[vfo_id] = 0;
    }

    //for each label
    BOOST_FOREACH(VisionFieldObject* gt, ground_truth) {
        int d_num = 0;
        pair<int,float> best_match = pair<int,float>(-1,std::numeric_limits<float>::max());
        //find the best match
        BOOST_FOREACH(const VisionFieldObject* d_vfo, data_wrapper->detections) {
            //if the object is a match and the detection has not already been matched
            if(objectTypesMatch(d_vfo->getID(), gt->getID()) && !matched_detections.at(d_num)) {
                float err = 0;
                if(use_ground_errors)
                    err = d_vfo->findGroundError(gt);
                else
                    err = d_vfo->findScreenError(gt);

                if(err < best_match.second) {
                    best_match = pair<int, float>(d_num, err);
                }
            }
            d_num++;
        }
        if(best_match.first == -1) {
            //no detection for gt item - false negative
            false_neg.at(gt->getID())++;
        }
        else {
            //true positive
            matches.at(gt->getID())++;
            matched_detections.at(best_match.first) = true;
        }
    }

    //all remaining unmatched detections are false positives
    for(unsigned int d=0; d<matched_detections.size(); d++) {
        if(!matched_detections.at(d)) {
            //false positive
            VFO_ID id = data_wrapper->detections.at(d)->getID();
            false_pos.at(id)++;
        }
    }

    //build result from accumulators
    for(int i = 0; i < numVFOIDs(); i++) {
        VFO_ID id = VFOFromInt(i);
        result[id].x = matches[id];
        result[id].y = false_pos[id];
        result[id].z = false_neg[id];
    }

    return result;
}

/*!
  * @brief returns whether two IDs are a match
  */
bool VisionControlWrapper::objectTypesMatch(VFO_ID id0, VFO_ID id1) const
{
    switch(id0) {
    case BALL:
        return id1 == id0;
    case OBSTACLE:
        return id1 == id0;
    case FIELDLINE:
        return id1 == id0;
    case GOAL_L:
        return id1 == id0 || id1 == GOAL_U;    //can also match unknown post
    case GOAL_R:
        return id1 == id0 || id1 == GOAL_U;    //can also match unknown post
    case GOAL_U:
        return isGoal(id1);  //can match any post
    default:
        errorlog << "VisionControlWrapper::objectTypesMatch - invalid id" << std::endl;
        return false;
    }
}

/*!
  * @brief clears the detection history
  */
void VisionControlWrapper::resetHistory()
{
    data_wrapper->resetHistory();
}

/*!
  * @brief renders the current image and detections to a cv::Mat.
  * @param mat The cv::Mat to render to.
  * @return success
  */
bool VisionControlWrapper::renderFrame(QImage& img, bool lines_only)
{
    return data_wrapper->renderFrame(img, lines_only);
}
