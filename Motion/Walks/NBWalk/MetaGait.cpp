#include  "MetaGait.h"
#include  "MotionConstants.h"

//#define DEBUG_META_GAIT


MetaGait::MetaGait():
    curGait(DEFAULT_GAIT),
    nextGait(DEFAULT_GAIT),
    newGait(DEFAULT_GAIT),
    newGaitSent(false),
    transitionCounter(0),
    transitionFrames(0)
{setGaitFromGait(DEFAULT_GAIT);}

MetaGait::~MetaGait(){}

void MetaGait::tick_gait(){
#ifdef DEBUG_META_GAIT
//    std::cout << "MetaGait::tick_gait()"<<std::endl;
#endif
    if(updateGaits()){
#ifdef DEBUG_META_GAIT
        std::cout << "Interpolating Gaits percent = "<<getPercentComplete()<<std::endl;
#endif
        interpolateGaits(*this,curGait,nextGait,getPercentComplete());

#ifdef DEBUG_META_GAIT
        std::cout<< "Gait 1 is "<<std::endl<<curGait.toString()<<std::endl;
        std::cout<< "Gait 2 is "<<std::endl<<nextGait.toString()<<std::endl;
        std::cout<< "result is"<<std::endl<<toString()<<std::endl;
#endif
    }
    //interpolateGaits(*this,DEFAULT_GAIT,DEFAULT_GAIT,1.0f);

}

void MetaGait::setNewGaitTarget(Gait &nextTarget){
#ifdef DEBUG_META_GAIT
        std::cout << "MetaGait got a new target "<<std::endl;
#endif
    newGait = nextTarget;
    newGaitSent = true;
}

void MetaGait::setStartGait(Gait & newCurGait){
    curGait = newCurGait;
    nextGait = newCurGait;
    resetTransitioning();
}

float MetaGait::getPercentComplete(){
    if(transitionFrames == 0)
        return 1.0f;
    return NBMath::clip(static_cast<float>(transitionCounter)/
                        static_cast<float>(transitionFrames),
                        0.0f,1.0f);
}

void MetaGait::resetTransitioning(){
    //First, find the maximum
    const float maxTime = std::max(curGait.stance[WP::TRANS_TIME],
                                   nextGait.stance[WP::TRANS_TIME]);
    transitionFrames  =
        static_cast<unsigned int>(maxTime /
                                  MotionConstants::MOTION_FRAME_LENGTH_S);

    transitionCounter = 1;
}

bool MetaGait::updateGaits(){
    if(newGaitSent){
        //Make a hybrid gait from the currently selected gaits
        AbstractGait hybridGait;
        const float percComplete = getPercentComplete();
        interpolateGaits(hybridGait,curGait,nextGait,percComplete);

        //Swap the gaits
        curGait = hybridGait;
        nextGait = newGait;

        //Reset the counters
        resetTransitioning();
        newGaitSent = false;
    }else{
        transitionCounter = std::min(transitionCounter + 1, transitionFrames+1);
        // std::cout << "Updated transition counter to "<<transitionCounter
        //      << "  Transition frames is "<< transitionFrames<<std::endl;
    }

    //we still need more processing,
    return transitionCounter <= transitionFrames;
}





