#include <assert.h>
#include <iostream>
#include <armadillo>
#include "MultivariateGaussian.h"

#include "CircBuffer.h" //for circular buffer class <boost/circular_buffer.hpp>
#include "VisionObjects.h" //Infrastructure/FieldObjects/FieldObjects.h
#include "UnscentedTransform.h"

#include <sstream> //for the .cpp file
#include <iomanip> //std::setprecision

//#include "IKFModel.h" //the model now gets inherited from the template parameter
//PRECONDITION: IKalmanFilter<type> :: type MUST IMPLEMENT THE FOLLOWING METHODS:
    //Clone() 
    //totalStates()
    //CAN GO THROUGH AND REMOVE ALL THE VIRTUAL KEYWORDS WHEN DONE PORTING (we most likely wont be extending this kalman class further, as testing on all variants was conducted in Steve's PHD)

template <typename Model> //model is is a template parameter that Kalman also inherits
class Kalman : public Model {
public:
//----------------------------------IKalmanFilter.h file
//virtual Kalman* Clone() = 0;    //pure virtual functions get commented out
//virtual bool timeUpdate(double delta_t, const arma::mat& measurement, const arma::mat& process_noise, const arma::mat& measurement_noise) = 0;
//virtual bool measurementUpdate(const arma::mat& measurement, const arma::mat& noise, const arma::mat& args, unsigned int type) = 0;
//virtual void initialiseEstimate(const MultivariateGaussian& estimate) = 0;
//virtual std::ostream& writeStreamBinary (std::ostream& output) const = 0;
//virtual std::istream& readStreamBinary (std::istream& input) = 0;
//virtual std::string summary(bool detailed) const = 0;

    Kalman() {} // {} //ox
    virtual ~Kalman() { //IKalmanFilter //DESTRUCTOR OVERWRITTEN IN .cpp file 
        if(m_model != NULL)
            delete m_model;
        
        m_previous_decisions.clear();  //IWeightedKalmanFilter
        //m_parent_history_buffer.clear();   //IWeightedKalmanFilter
    }

    virtual const MultivariateGaussian& estimate() const {return m_estimate;}

    // Outlier filtering settings.
    virtual void enableOutlierFiltering(bool enabled = true) {m_outlier_filtering_enabled = enabled;}
    virtual void setOutlierThreshold(float new_threshold){m_outlier_threshold = new_threshold;}
    virtual bool outlierFiltering() const {return m_outlier_filtering_enabled;}
    virtual float outlierThreshold() const {return m_outlier_threshold;}

    void setModel(Model* newModel) {
        if(m_model) delete m_model;
        m_model = newModel;
    }
    
    
    Model* model() {return m_model;}

    bool operator ==(const Kalman& b) const {
        MultivariateGaussian estA = this->estimate();
        MultivariateGaussian estB = b.estimate();

        if(estA != estB) { // Check estimates are equal (comparing gaussians, not matricies)
            return false;
        }
        return true;
    }
    bool operator !=(const Kalman& b) const {return (!((*this) == b));}
    
protected:
    Model* m_model;
    MultivariateGaussian m_estimate;
    bool m_outlier_filtering_enabled;
    float m_outlier_threshold;

    Kalman(Model* model): m_model(model), m_estimate(model->totalStates()), m_unscented_transform(model->totalStates()) { //IKalmanFilter
        m_outlier_filtering_enabled = false;
        m_outlier_threshold = 15.f;
        
        m_previous_decisions.resize(VisionObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, VisionObjects::NUM_STAT_FIELD_OBJECTS); //IWeightedKalmanFilter
        m_id = GenerateId(); //IWeightedKalmanFilter
        //m_parent_history_buffer.resize(5,0);  //IWeightedKalmanFilter

        initialiseEstimate(m_estimate); //WSeqUKF
        m_weighting_enabled = false;
        m_filter_weight = 1.f;
    }
    
    /*wsequkf
     
    Kalman::Kalman(IKFModel *model): IWeightedKalmanFilter(model), m_unscented_transform(model->totalStates()) {
    initialiseEstimate(m_estimate);
    m_weighting_enabled = false;
    m_filter_weight = 1.f;
     
     */

    Kalman(const Kalman& source);
    
//-----------------------------------------------------------IWeightedKalmanFilter

//class IWeightedKalmanFilter: public IKalmanFilter {
public:
    // Multiple model stuff
    unsigned int m_split_option;    //!< Most recent option used for split from parent.
    std::vector<unsigned int> m_previous_decisions; //!< Stores the last decision for each of the ambiguous object types.
    unsigned int m_parent_id;       //!< Unique id of the parent model.
    double m_creation_time;         //!< Time at which model was created.
    //boost::circular_buffer<unsigned int> m_parent_history_buffer;
    
    // Active controled
    virtual bool active() const {return m_active;}
    virtual void setActive(bool active = true) {m_active = active;}
    virtual unsigned int id() const {return m_id;}
    virtual void AssignNewId() {m_id = GenerateId();}

    //virtual IWeightedKalmanFilter* Clone() = 0;
    //virtual void enableWeighting(bool enabled = true) = 0; // Weighting functions.
    //virtual float getFilterWeight() const = 0;
    //virtual void setFilterWeight(float weight) = 0;

    //Multiple model stuff
    //@brief Get the previous decision path when this ambiguous object was last encountered.
    //@param theObject The ambiguous object.
    //@return The object id of the unique object that was last chosen.
      
    unsigned int previousSplitOption(const VisionObjects& theObject) const {
        unsigned int result;
        unsigned int objectIndex = theObject.getID();
        assert(objectIndex < VisionObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
        result = m_previous_decisions[objectIndex];
        return result;
    }

    double creationTime() const {return m_creation_time;}
    //virtual std::ostream& writeStreamBinary (std::ostream& output) const {} // @brief Outputs a binary representation of the IWeightedKalmanFilter object to a stream. @param output The output stream. @return The output stream.
    //virtual std::istream& readStreamBinary (std::istream& input) {} // @brief Reads in a IWeightedKalmanFilter object from the input stream. @param input The input stream. @return The input stream.

protected:    // Multiple Model stuff
    bool m_active;
    unsigned int m_id;

    static unsigned int GenerateId() {
        static unsigned int id = 0;
        return id++;
    }
//};
//----------------------------------------------------------- WSeqUKF

public:
    //Kalman(Model* model); //WSeqUKF
    Kalman* Clone() { //IWeightedKalmanFilter* Clone()
        return new Kalman(*this); //WSeqUKF(*this)
    }

   //@brief Time update function The time update function predicts the new state of the system. 
   //@param delta_t The elapsed time since the previous time update. 
    //@param measurement Any measurements that can be used to predict the change in state. 
    //@param process_noise The linear process noise to be added to the estimate. 
    //@param measurement_noise The noise present in the measurement provided. 
    //@return True if the update was performed successfully. False if the update was unable to be performed.
    bool timeUpdate(double delta_t, const arma::mat& measurement, const arma::mat& process_noise, const arma::mat& measurement_noise);
    
//@brief Measurement update function The measurement update function corrects the estimated state of the system using observed measurement/s. 
    //@param measurement The measured data to be used for the update. 
    //@param noise The noise associated with the measurement data provided. 
    //@param args Any additional arguments required for the measurement update. 
    //@param type The type of measurement. @return True if the update was performed sucessfully. False if the update was unable to be performed.
    bool measurementUpdate(const arma::mat& measurement, const arma::mat& noise, const arma::mat& args, unsigned int type);            

    void initialiseEstimate(const MultivariateGaussian& estimate); //@brief Initialisation function. Used to initialise the filters estimate. @param estimate The initial estimate of the filter.
    std::string summary(bool detailed) const;

    //depracated
    //std::ostream& writeStreamBinary (std::ostream& output) const; //@brief Outputs a binary representation of the UKF object to a stream. @param output The output stream. @return The output stream.
    //std::istream& readStreamBinary (std::istream& input); //@brief Reads in a UKF object from the input stream. @param input The input stream. @return The input stream.

    void enableWeighting(bool enabled = true) {m_weighting_enabled = enabled;}    // Weighting functions.
    float getFilterWeight() const {return m_filter_weight;}
    void setFilterWeight(float weight) {m_filter_weight = weight;}

protected:
    //Kalman(const Kalman& source); //WSeqUKF(const WSeqUKF& source)
    bool m_weighting_enabled;
    float m_filter_weight;
    arma::mat m_sigma_points;
    arma::mat m_sigma_mean;
    arma::mat m_C;
    arma::mat m_d;
    arma::mat m_X;
    UnscentedTransform m_unscented_transform;

    void init();
    bool evaluateMeasurement(const arma::mat& innovation, const arma::mat& estimate_variance, const arma::mat& measurement_variance);

//extra helper function 
public:
    double convDble(arma::mat X);
//-----------------------------------------------------------
}; //end class

//#include "Kalman.cpp" //flip the includes (its a template function).