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

    int nameLength = p_amb.name.size();
    output.write(reinterpret_cast<const char*>(&nameLength), sizeof(nameLength));
    for(int i = 0; i < nameLength; ++i)
        output.write(reinterpret_cast<const char*>(&p_amb.name[i]), sizeof(p_amb.name[i]));

    int size;
    size = p_amb.PossibleObjectIDs.size();
    output.write(reinterpret_cast<const char*>(&size), sizeof(size));
    for(int i = 0; i < size; ++i)
        output.write(reinterpret_cast<const char*>(&p_amb.PossibleObjectIDs[i]), sizeof(p_amb.PossibleObjectIDs[i]));
    return output;
}

std::istream& operator>> (std::istream& input, AmbiguousObject& p_amb)
{
    input >> *static_cast<Object*>(&p_amb);

    int nameLength;
    input.read(reinterpret_cast<char*>(&nameLength), sizeof(nameLength));
    char tempChar;
    std::string name;
    for(int i = 0; i < nameLength; ++i)
    {
        if(input.eof() || input.bad()) break;
        input.read(reinterpret_cast<char*>(&tempChar), sizeof(tempChar));
        name += tempChar;
    }
    p_amb.name = name;

    unsigned int numEntries;
    int id; 
    input.read(reinterpret_cast<char*>(&numEntries), sizeof(numEntries));
    p_amb.PossibleObjectIDs.clear();
    for(unsigned int i = 0; i < numEntries; i++)
    {
        if(input.eof() || input.bad()) break;
        input.read(reinterpret_cast<char*>(&id), sizeof(id));
        p_amb.PossibleObjectIDs.push_back(id);
    }
    return input;
}
