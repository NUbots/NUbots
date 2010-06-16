#include "AmbiguousObject.h"

//AMBIGUOUS FIELD OBJECT ID: Automatically set up the possible IDS

AmbiguousObject::AmbiguousObject(int id, const std::string& initName): Object(id, initName)
{

}

AmbiguousObject::~AmbiguousObject()
{

}

void AmbiguousObject::addPossibleObjectID(int ID)
{
        PossibleObjectIDs.push_back(ID);
}

void AmbiguousObject::setPossibleObjectIDs(vector<int> VectorOfIDs)
{
	PossibleObjectIDs = VectorOfIDs;
}

bool AmbiguousObject::isObjectAPossibility(int ID)
{
	bool isInVector = false;
        for(unsigned int i = 0; i < PossibleObjectIDs.size(); i++)
	{
                if(PossibleObjectIDs[i] == ID)
		{
			return true;
		}
	}
	return isInVector;
}
void AmbiguousObject::setVisibility(bool newIsVisible)
{
    isVisible = newIsVisible;
}

std::ostream& operator<< (std::ostream& output, const AmbiguousObject& p_amb)
{
    output << *static_cast<const Object*>(&p_amb);
    output << p_amb.PossibleObjectIDs.size() << ' ';
    for(int i = 0; i < p_amb.PossibleObjectIDs.size(); i++)
        output << p_amb.PossibleObjectIDs[i] << ' ';
    return output;
}

std::istream& operator>> (std::istream& input, AmbiguousObject& p_amb)
{
    input >> *static_cast<Object*>(&p_amb);
    unsigned int numEntries;
    int id;
    input >> numEntries;
    p_amb.PossibleObjectIDs.clear();
    for(int i = 0; i < numEntries; i++)
    {
        input >> id;
        p_amb.PossibleObjectIDs.push_back(id);
    }
    return input;
}
