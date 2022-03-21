
#include <despot/model_primitives/iros/enum_map_iros.h> 
using namespace std;
namespace despot
{ 
	map<IrosResponseModuleAndTempEnums, std::string> enum_map_iros::vecResponseEnumToString;
	map<std::string, IrosResponseModuleAndTempEnums> enum_map_iros::vecStringToResponseEnum ;
	map<ActionType,std::string> enum_map_iros::vecActionTypeEnumToString;

	void enum_map_iros::Init()
	{
		if(enum_map_iros::vecResponseEnumToString.size() > 0)
			return; 

		enum_map_iros::vecResponseEnumToString = enum_map_iros::CreateMapResponseEnumToString();
	 enum_map_iros::vecStringToResponseEnum = enum_map_iros::CreateMapStringToEnum(enum_map_iros::vecResponseEnumToString);
	enum_map_iros::vecActionTypeEnumToString = enum_map_iros::CreateMapActionTypeEnumToString();
	}
} // namespace despot
