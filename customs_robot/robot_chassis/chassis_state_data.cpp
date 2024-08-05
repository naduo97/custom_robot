#include "chassis_state_data.h"

namespace RobotChassis{
ChassisStateData::ChassisStateData()
{
    dVoltage_ = 0.0;
    iCurrentStationID_ = 0;
    iTargetStationID_ = 0;
}

ChassisStateData::~ChassisStateData()
{

}

//--------------------------------------

ChassisControl::ChassisControl()
{

}

ChassisControl::~ChassisControl()
{

}
}
