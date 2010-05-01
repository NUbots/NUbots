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
                vector<int> getPossibleObjectIDs(){return PossibleObjectIDs;}
		void addPossibleObjectID(int ID);
                void setPossibleObjectIDs(vector<int> VectorOfIDs);
                bool isObjectAPossibility(int ID);
};
