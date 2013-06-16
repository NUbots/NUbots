#pragma once

#include "Object.h"
#include <vector>


class AmbiguousObject : public Object
{
private:
	std::vector<int> PossibleObjectIDs;
public:
	//		AmbiguousObject();
	AmbiguousObject(int id = -1, const std::string& initName = "Unknown");
	~AmbiguousObject();

	std::vector<int> getPossibleObjectIDs() const
	{
		return PossibleObjectIDs;
	}
	void addPossibleObjectID(int ID);
	void setPossibleObjectIDs(std::vector<int> VectorOfIDs);
	bool isObjectAPossibility(int ID);
	void setVisibility(bool newIsVisible);

	friend std::ostream& operator<<(std::ostream& output, const AmbiguousObject& p_amb);
	friend std::istream& operator>>(std::istream& input, AmbiguousObject& p_amb);
        
};
