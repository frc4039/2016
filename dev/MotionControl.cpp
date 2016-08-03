//
//  MotionControl.cpp
//  
//
//  Created by Noah Linton on 2016-07-01.
//
//

#include "MotionControl.hpp"


class MotionControl
{
private:
    //define motors somehow
    VictorSP *leftDrive4;
    VictorSP *leftDrive1;
    VictorSP *rightDrive2;
    VictorSP *rightDrive3;
    
    // Gear definitions
    
    
    // gear ratio calculation
    float gearRatio()
    {
        
    }
    
    
public:
    void setVelocity(velocity) // set velocity
    {
        leftDrive4->SetSpeed(velocity);
        leftDrive1->SetSpeed(velocity);
        rightDrive2->SetSpeed(-velocity);
        rightDrive3->SetSpeed(-velocity);
        
    }
    
};
