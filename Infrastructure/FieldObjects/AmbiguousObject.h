#pragma once

#include "Object.h"
#include <vector>
using namespace std;

class AmbiguousObject : public Object{
	private:
        vector<int> PossibleObjectIDs;
	public:
//		AmbiguousObject();
        AmbiguousObject(int id = -1, const std::string& initName = "Unknown");
		~AmbiguousObject();
        vector<int> getPossibleObjectIDs() const {return PossibleObjectIDs;}
		void addPossibleObjectID(int ID);
        void setPossibleObjectIDs(vector<int> VectorOfIDs);
        bool isObjectAPossibility(int ID);
        void setVisibility(bool newIsVisible);

        /*!
        @brief Output streaming operation.
        @param output The output stream.
        @param p_loc The source localisation data to be streamed.
        */
        friend std::ostream& operator<< (std::ostream& output, const AmbiguousObject& p_amb);

        /*!
        @brief Input streaming operation.
        @param input The input stream.
        @param p_kf The destination localisation data to be streamed to.
        */
        friend std::istream& operator>> (std::istream& input, AmbiguousObject& p_amb);

        //! width of an obstacle in angular terms
        float arc_width;
};
