#ifndef VAR_TYPES_H
#define VAR_TYPES_H

#include <string>
#include <iostream> 

namespace despot
{
	typedef bool anyValue;



	enum tDiscreteLocation
	{
		eCorridor,
		eLocationAuditoriumSide1,
		eLocationAuditoriumSide2,
		eRobotHand,
		eUnknown
	};




	struct tLocation
	{
		tDiscreteLocation discrete_location;
		anyValue actual_location;
		inline bool operator==(const tLocation& other)const{return (*this).discrete_location == other.discrete_location && (*this).actual_location == other.actual_location;};
		inline bool operator!=(const tLocation& other)const{return !(*this == other);};
		tLocation(); 
	};



 
} // namespace despot
#endif //VAR_TYPES_H