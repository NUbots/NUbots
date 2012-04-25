#include "clustercandidates.h"

ClusterCandidates::ClusterCandidates()
{
}

//vector<ObjectCandidate> ClusterCandidates::classifyCandidatesPrims(vector<Transition> &transitions, 
//                                                                      const vector<PointType> &fieldBorders, 
//                                                                      const vector<ClassIndex::Colour> &validColours, 
//                                                                      int spacing, 
//                                                                      float min_aspect, float max_aspect, int min_transitions)

//{
//    //! Overall runtime O( N^2 )
//    vector<ObjectCandidate> candidateList;

//    const int VERT_JOIN_LIMIT = 3;
//    const int HORZ_JOIN_LIMIT = 1;


//    if (!transitions.empty())
//    {
//        //! Sorting O(N*logN)
//        sort(transitions.begin(), transitions.end());

//        queue<int> qUnprocessed;
//        vector<Transition> candidate_transitions;
//        vector<unsigned int> usedTransitions;
//        unsigned int rawSegsLeft = transitions.size();
//        unsigned int nextRawSeg = 0;

//        bool isSegUsed [transitions.size()];
//        usedTransitions.clear();
//        //! Removing invalid colours O(N)
//        for (unsigned int i = 0; i < transitions.size(); i++)
//        {
//            //may have non-robot colour transitions, so pre-mark them as used
//            if (isValidColour(transitions.at(i).getColour(), validColours))
//            {
//                //qDebug() << ClassIndex::getColourNameFromIndex(transitions.at(i).getColour()) << isRobotColour(transitions.at(i).getColour());
//                isSegUsed[i] = false;
//                //qDebug() <<  "(" << transitions.at(i).getStartPoint().x << "," << transitions.at(i).getStartPoint().y << ")-("<< transitions.at(i).getEndPoint().x << "," << transitions.at(i).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(transitions.at(i).getColour()) << "]";
//            }
//            else
//            {
//                rawSegsLeft--;
//                isSegUsed[i] = true;
//            }

//        }

//        //! For all valid transitions O(M)
//        while(rawSegsLeft)
//        {

//            //Roll through and find first unused segment
//            nextRawSeg = 0;

//            //! Find next unused segment O(M)
//            while(isSegUsed[nextRawSeg] && nextRawSeg < transitions.size()) nextRawSeg++;
//            //Prime unprocessed segment queue to build next candidate
//            qUnprocessed.push(nextRawSeg);
//            //take away from pool of raw segs
//            isSegUsed[nextRawSeg] = true;
//            rawSegsLeft--;

//            int min_x, max_x, min_y, max_y, segCount;
//            int * colourHistogram = new int[validColours.size()];
//            min_x = transitions.at(nextRawSeg).getStartPoint().x;
//            max_x = transitions.at(nextRawSeg).getStartPoint().x;
//            min_y = transitions.at(nextRawSeg).getStartPoint().y;
//            max_y = transitions.at(nextRawSeg).getEndPoint().y;
//            segCount = 0;
//            for (unsigned int i = 0; i < validColours.size(); i++)  colourHistogram[i] = 0;

//            //! For all unprocessed joined segment in a candidate O(M)
//            //Build candidate

//            candidate_transitions.clear();

//            while (!qUnprocessed.empty())
//            {
//                unsigned int thisSeg;
//                thisSeg = qUnprocessed.front();
//                qUnprocessed.pop();
//                segCount++;
//                for (unsigned int i = 0; i < validColours.size(); i++)
//                {
//                    if ( transitions.at(thisSeg).getColour() == validColours.at(i) && validColours.at(i) != ClassIndex::white)
//                    {
//                        colourHistogram[i] += 1;
//                        i = validColours.size();
//                    }
//                }

//                if ( min_x > transitions.at(thisSeg).getStartPoint().x)
//                    min_x = transitions.at(thisSeg).getStartPoint().x;
//                if ( max_x < transitions.at(thisSeg).getStartPoint().x)
//                    max_x = transitions.at(thisSeg).getStartPoint().x;
//                if ( min_y > transitions.at(thisSeg).getStartPoint().y)
//                    min_y = transitions.at(thisSeg).getStartPoint().y;
//                if ( max_y < transitions.at(thisSeg).getEndPoint().y)
//                    max_y = transitions.at(thisSeg).getEndPoint().y;



//                //if there is a seg above AND 'close enough', then qUnprocessed->push()
//                if ( thisSeg > 0 &&
//                     !isSegUsed[thisSeg-1] &&
//                     transitions.at(thisSeg).getStartPoint().x == transitions.at(thisSeg-1).getStartPoint().x &&
//                     transitions.at(thisSeg).getStartPoint().y - transitions.at(thisSeg-1).getEndPoint().y < VERT_JOIN_LIMIT)
//                {
//                    //qDebug() << "Up   Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg).getColour()) << ") U " << (thisSeg-1)<< "(" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg-1).getColour()) << ")" << "(" << transitions.at(thisSeg).getStartPoint().x << "," << transitions.at(thisSeg).getStartPoint().y << ")-("<< transitions.at(thisSeg).getEndPoint().x << "," << transitions.at(thisSeg).getEndPoint().y << ")::(" << transitions.at(thisSeg-1).getStartPoint().x << "," << transitions.at(thisSeg-1).getStartPoint().y << ")-("<< transitions.at(thisSeg-1).getEndPoint().x << "," << transitions.at(thisSeg-1).getEndPoint().y << ")";
//                    qUnprocessed.push(thisSeg-1);
//                    //take away from pool of raw segs
//                    isSegUsed[thisSeg-1] = true;
//                    rawSegsLeft--;
//                }

//                //if there is a seg below AND 'close enough', then qUnprocessed->push()
//                if ( thisSeg+1 < transitions.size() &&
//                     !isSegUsed[thisSeg+1] &&
//                     transitions.at(thisSeg).getStartPoint().x == transitions.at(thisSeg+1).getStartPoint().x &&
//                     transitions.at(thisSeg+1).getStartPoint().y - transitions.at(thisSeg).getEndPoint().y < VERT_JOIN_LIMIT)
//                {
//                    //qDebug() << "Down Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg).getColour()) << ") U " << (thisSeg+1)<< "(" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg+1).getColour()) << ")" << "(" << transitions.at(thisSeg).getStartPoint().x << "," << transitions.at(thisSeg).getStartPoint().y << ")-("<< transitions.at(thisSeg).getEndPoint().x << "," << transitions.at(thisSeg).getEndPoint().y << ")::(" << transitions.at(thisSeg+1).getStartPoint().x << "," << transitions.at(thisSeg+1).getStartPoint().y << ")-("<< transitions.at(thisSeg+1).getEndPoint().x << "," << transitions.at(thisSeg+1).getEndPoint().y << ")";
//                    qUnprocessed.push(thisSeg+1);
//                    //take away from pool of raw segs
//                    isSegUsed[thisSeg+1] = true;
//                    rawSegsLeft--;
//                }

//                //! For each segment being processed in a candidate to the RIGHT attempt to join transitions within range O(M)
//                //if there is a seg overlapping on the right AND 'close enough', then qUnprocessed->push()
//                for (unsigned int thatSeg = thisSeg + 1; thatSeg < transitions.size(); thatSeg++)
//                {
//                    if ( transitions.at(thatSeg).getStartPoint().x - transitions.at(thisSeg).getStartPoint().x <=  spacing*HORZ_JOIN_LIMIT)
//                    {
//                        if ( transitions.at(thatSeg).getStartPoint().x > transitions.at(thisSeg).getStartPoint().x &&
//                             !isSegUsed[thatSeg])
//                        {
//                            //NOT in same column as thisSeg and is to the right
//                            if ( transitions.at(thatSeg).getStartPoint().y <= transitions.at(thisSeg).getEndPoint().y &&
//                                 transitions.at(thisSeg).getStartPoint().y <= transitions.at(thatSeg).getEndPoint().y)
//                            {
//                                //thisSeg overlaps with thatSeg
//                                //qDebug() <<  "(" << transitions.at(thisSeg).getStartPoint().x << "," << transitions.at(thisSeg).getStartPoint().y << ")-("<< transitions.at(thisSeg).getEndPoint().x << "," << transitions.at(thisSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg).getColour()) << "]::(" << transitions.at(thatSeg).getStartPoint().x << "," << transitions.at(thatSeg).getStartPoint().y << ")-("<< transitions.at(thatSeg).getEndPoint().x << "," << transitions.at(thatSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(transitions.at(thatSeg).getColour()) << "]";
//                                //! Find intercept O(K), K is number of field border points
//                                int intercept = findInterceptFromPerspectiveFrustum(fieldBorders,
//                                                                                    transitions.at(thisSeg).getStartPoint().x,
//                                                                                    transitions.at(thatSeg).getStartPoint().x,
//                                                                                    spacing*HORZ_JOIN_LIMIT);
//                                if ( intercept >= 0 &&
//                                     transitions.at(thatSeg).getEndPoint().y >= intercept &&
//                                     intercept <= transitions.at(thisSeg).getEndPoint().y)
//                                {
//                                    //within HORZ_JOIN_LIMIT
//                                    //qDebug() << "Right Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(transitions.at(thatSeg).getColour()) << ")" << "(" << transitions.at(thisSeg).getStartPoint().x << "," << transitions.at(thisSeg).getStartPoint().y << ")-("<< transitions.at(thisSeg).getEndPoint().x << "," << transitions.at(thisSeg).getEndPoint().y << ")::(" << transitions.at(thatSeg).getStartPoint().x << "," << transitions.at(thatSeg).getStartPoint().y << ")-("<< transitions.at(thatSeg).getEndPoint().x << "," << transitions.at(thatSeg).getEndPoint().y << ")";
//                                    qUnprocessed.push(thatSeg);
//                                    //take away from pool of raw segs
//                                    isSegUsed[thatSeg] = true;
//                                    rawSegsLeft--;

//                                }
//                                else
//                                {
//                                    //qDebug() << "|" << float(transitions.at(thatSeg).getEndPoint().y) <<  "thatSeg End(y)>= intercept" << intercept << "|";
//                                    //qDebug() << "|" << int(intercept) << "intercept <= thisSeg End(y)" << transitions.at(thisSeg).getEndPoint().y << "|";
//                                }
//                            }
//                        }
//                    }
//                    else
//                    {
//                        thatSeg = transitions.size();
//                    }
//                }

//                //! For each segment being processed in a candidate to the LEFT attempt to join transitions within range O(M)
//                //if there is a seg overlapping on the left AND 'close enough', then qUnprocessed->push()
//                for (int thatSeg = thisSeg - 1; thatSeg >= 0; thatSeg--)
//                {
//                    if ( transitions.at(thisSeg).getStartPoint().x - transitions.at(thatSeg).getStartPoint().x <=  spacing*HORZ_JOIN_LIMIT)
//                    {

//                        if ( !isSegUsed[thatSeg] &&
//                             transitions.at(thatSeg).getStartPoint().x < transitions.at(thisSeg).getStartPoint().x)
//                        {
//                            //NOT in same column as thisSeg and is to the right
//                            if ( transitions.at(thatSeg).getStartPoint().y <= transitions.at(thisSeg).getEndPoint().y &&
//                                 transitions.at(thisSeg).getStartPoint().y <= transitions.at(thatSeg).getEndPoint().y)
//                            {
//                                //thisSeg overlaps with thatSeg
//                                //qDebug() <<  "(" << transitions.at(thisSeg).getStartPoint().x << "," << transitions.at(thisSeg).getStartPoint().y << ")-("<< transitions.at(thisSeg).getEndPoint().x << "," << transitions.at(thisSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg).getColour()) << "]::(" << transitions.at(thatSeg).getStartPoint().x << "," << transitions.at(thatSeg).getStartPoint().y << ")-("<< transitions.at(thatSeg).getEndPoint().x << "," << transitions.at(thatSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(transitions.at(thatSeg).getColour()) << "]";
//                                //! Find intercept O(K), K is number of field border points
//                                int intercept = findInterceptFromPerspectiveFrustum(fieldBorders,
//                                                                                    transitions.at(thisSeg).getStartPoint().x,
//                                                                                    transitions.at(thatSeg).getStartPoint().x,
//                                                                                    spacing*HORZ_JOIN_LIMIT);

//                                if ( intercept >= 0 &&
//                                     transitions.at(thatSeg).getEndPoint().y >= intercept &&
//                                     intercept <= transitions.at(thisSeg).getEndPoint().y)
//                                {
//                                    //within HORZ_JOIN_LIMIT
//                                    //qDebug() << "Left Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(transitions.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(transitions.at(thatSeg).getColour()) << ")" << "(" << transitions.at(thisSeg).getStartPoint().x << "," << transitions.at(thisSeg).getStartPoint().y << ")-("<< transitions.at(thisSeg).getEndPoint().x << "," << transitions.at(thisSeg).getEndPoint().y << ")::(" << transitions.at(thatSeg).getStartPoint().x << "," << transitions.at(thatSeg).getStartPoint().y << ")-("<< transitions.at(thatSeg).getEndPoint().x << "," << transitions.at(thatSeg).getEndPoint().y << ")";
//                                    qUnprocessed.push(thatSeg);
//                                    //take away from pool of raw segs
//                                    isSegUsed[thatSeg] = true;
//                                    rawSegsLeft--;
//                                }
//                                else
//                                {
//                                    //qDebug() << "|" << float(transitions.at(thatSeg).getEndPoint().y) <<  "thatSeg End(y)>= intercept" << intercept << "|";
//                                    //qDebug() << "|" << int(intercept) << "intercept <= thisSeg End(y)" << transitions.at(thisSeg).getEndPoint().y << "|";
//                                }
//                            }
//                        }
//                    }
//                    else
//                    {
//                        thatSeg = -1;
//                    }
//                }

//                //add thisSeg to CandidateVector
//                candidate_transitions.push_back(transitions.at(thisSeg));
//                transitions.at(thisSeg).isUsed = true;
//                usedTransitions.push_back(thisSeg);
//            }//while (!qUnprocessed->empty())
//            //qDebug() << "Candidate ready...";
//            //HEURISTICS FOR ADDING THIS CANDIDATE AS A ROBOT CANDIDATE
//            if ( max_x - min_x >= 0 &&                                               // width  is non-zero
//                 max_y - min_y >= 0 &&                                               // height is non-zero
//                 (float)(max_x - min_x) / (float)(max_y - min_y) <= max_aspect &&    // Less    than specified landscape aspect
//                 (float)(max_x - min_x) / (float)(max_y - min_y) >= min_aspect &&    // greater than specified portrait aspect
//                 segCount >= min_transitions                                    // greater than minimum amount of transitions to remove noise
//                 )
//            {
//                //qDebug() << "CANDIDATE FINISHED::" << segCount << " transitions, aspect:" << ( (float)(max_x - min_x) / (float)(max_y - min_y)) << ", Coords:(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << "), width: " << (max_x - min_x) << ", height: " << (max_y - min_y);
//                int max_col = 0;
//                for (int i = 0; i < (int)validColours.size(); i++)
//                {
//                    if (i != max_col && colourHistogram[i] > colourHistogram[max_col])
//                        max_col = i;
//                }
//                for(unsigned int i=0; i<candidate_transitions.size(); i++) {
//                    candidate_transitions[i].isUsed = true;
//                }
//                ObjectCandidate temp(min_x, min_y, max_x, max_y, validColours.at(max_col), candidate_transitions);
//                candidateList.push_back(temp);
//                usedTransitions.clear();
//            }
//            else {
//                while(!usedTransitions.empty()){
//                    transitions[usedTransitions.back()].isUsed = false;
//                    usedTransitions.pop_back();
//                }
//            }
//	    delete [] colourHistogram;
//        }//while(rawSegsLeft)

//    }//if (!transitions.empty())
//    return candidateList;
//}
