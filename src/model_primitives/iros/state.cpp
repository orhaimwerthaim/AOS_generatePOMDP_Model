#include <despot/model_primitives/iros/state.h> 
namespace despot {
	

    tLocation::tLocation()
    {
        discrete_location = eUnknown;
        actual_location = false;
    }





		void IrosState::SetAnyValueLinks(IrosState *state)
		{
            state->anyValueUpdateDic["state.locationCorridor.actual_location"] = &(state->locationCorridor.actual_location);
            state->anyValueUpdateDic["state.locationAuditorium_toCan1.actual_location"] = &(state->locationAuditorium_toCan1.actual_location);
            state->anyValueUpdateDic["state.locationAuditorium_toCan2.actual_location"] = &(state->locationAuditorium_toCan2.actual_location);

		}
}// namespace despot