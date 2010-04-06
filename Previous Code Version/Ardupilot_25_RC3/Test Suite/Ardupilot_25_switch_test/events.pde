
/*
	This event will be called when the switch changes
	It is up to the user how to react to a swicth change event
	options are: MANUAL, STABILIZE, FLY_BY_WIRE, AUTO, RTL, LOITER 
	see: defines.h
	
	The three switch postions can be handled by most radios.
	Adjust your seetings to make sure all three positions work.
	If you don't have a 3 postion switch, try a two position one 
	and note which case below activates in each position.
*/
void switch_event(byte switchPosition)
{
	switch(switchPosition)
	{
		case 1: // First position
		set_mode(POSITION_1);
		break;

		case 2: // middle position
		set_mode(POSITION_2);
		break;

		case 3: // last position
		set_mode(POSITION_3);
		break;
	}
}
