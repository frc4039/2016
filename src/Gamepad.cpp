#include "WPILib.h"
#include "Gamepad.h"
#include <math.h>

Gamepad::Gamepad(uint32_t usbChannel){
        gamepad=new Joystick(usbChannel);
        pressedButton = 0;
}

float Gamepad::GetLX(){
        return gamepad->GetRawAxis(1);
}
float Gamepad::GetLY(){
        return -gamepad->GetRawAxis(2);
}
float Gamepad::GetRX(){
        return gamepad->GetRawAxis(4);
}
float Gamepad::GetRY(){
        return -gamepad->GetRawAxis(5);
}
float Gamepad::GetDpadX(){
        return gamepad->GetRawAxis(2);
}
float Gamepad::GetDpadY(){
        return -gamepad->GetRawAxis(3);
}
float Gamepad::GetCombinedX(){
        return (gamepad->GetRawAxis(1)+gamepad->GetRawAxis(4))/2;
}
float Gamepad::GetAxis(uint32_t axis){
        return gamepad->GetRawAxis(axis);
}
float Gamepad::GetJoypadsX(){
        if(-gamepad->GetRawAxis(1)>gamepad->GetRawAxis(4))
                return gamepad->GetRawAxis(1);
        else if(-gamepad->GetRawAxis(1)<gamepad->GetRawAxis(4))
                return gamepad->GetRawAxis(2);
        else
                return 0;
}

bool Gamepad::GetDpadMoved(){
        if(gamepad->GetRawAxis(5)==0&&gamepad->GetRawAxis(6)==0)
                return false;
        else
                return true;
}
bool Gamepad::GetLstickMoved(){
        if(gamepad->GetRawAxis(1)==0&&gamepad->GetRawAxis(2)==0)
                return false;
        else
                return true;
}
bool Gamepad::GetRstickMoved(){
        if(gamepad->GetRawAxis(3)==0&&gamepad->GetRawAxis(4)==0)
                return false;
        else
                return true;
}
bool Gamepad::GetActive(){
        if(GetDpadMoved()||GetLstickMoved()||GetRstickMoved()){
                return true;
        }else{
                for(int i=1;i<=13;i++){
                        if(gamepad->GetRawButton(i))
                                return true;
                }
        }
        return false;
}

int Gamepad::GetLDegrees(){
	/*
	if ((gamepad->GetX() == 0) && (gamepad->GetY() == 0))
        return 0;
	else if(gamepad->GetX() >= 0 && gamepad->GetY() >= 0){ //quadrant 1
		if(gamepad->GetX() == 0) return 0;
		else return (atan(abs(gamepad->GetY())/abs(gamepad->GetX())));
	}
	else if(gamepad->GetY() >= 0 && gamepad->GetX() <= 0){ // quadrant 2
		if(gamepad->GetY() == 0) return 90;
		else return ((atan(abs(gamepad->GetX())/abs(gamepad->GetY()))) + 90);
	}
	else if(gamepad->GetY() <= 0 && gamepad->GetX() <= 0){ //quadrant 3
		if(gamepad->GetX() == 0) return 180;
		else return ((atan(abs(gamepad->GetY())/abs(gamepad->GetX()))) + 180);
	}
	else if(gamepad->GetY() <= 0 && gamepad->GetX() >= 0){
		if(gamepad->GetY() == 0) return 0;
		else return ((atan(abs(gamepad->GetX())/abs(gamepad->GetY()))) + 270);
	}
	else return 0;
	*/
	return 0;
}
float Gamepad::GetLMagnitude(){
        return sqrt(gamepad->GetRawAxis(1)*gamepad->GetRawAxis(1)+gamepad->GetRawAxis(2)*gamepad->GetRawAxis(2));
}
int Gamepad::GetRDegrees(){
        return 0;
}
float Gamepad::GetRMagnitude(){
        return sqrt(gamepad->GetRawAxis(3)*gamepad->GetRawAxis(3)+gamepad->GetRawAxis(4)*gamepad->GetRawAxis(4));
}

bool Gamepad::GetButton(uint32_t button){
        return gamepad->GetRawButton(button);
}

bool Gamepad::GetButtonTripped(uint32_t button){
        if(!gamepad->GetRawButton(button)){
                previousState[button]=false;
                return false;
        }else if(previousState[button]){
                previousState[button]=true;
                return false;
        }else{
                previousState[button]=true;
                return true;
        }
}
bool Gamepad::GetButtonReleased(uint32_t button){
        if(gamepad->GetRawButton(button)){
                previousState[button]=true;
                return false;
        }else if(!previousState[button]){
                previousState[button]=false;
                return false;
        }else{
                previousState[button]=false;
                return true;
        }
}

float Gamepad::GetThrottle(){
        if((gamepad->GetRawButton(5)&&gamepad->GetRawButton(7))||(gamepad->GetRawButton(6)&&gamepad->GetRawButton(8)))
                return .7;
        else if(gamepad->GetRawButton(5)||gamepad->GetRawButton(6))
                return .5;
        else if(gamepad->GetRawButton(7)||gamepad->GetRawButton(8))
                return .1;
        else
                return .3;
}
float Gamepad::GetLeftThrottle(){
        if(gamepad->GetRawButton(5)&&gamepad->GetRawButton(7))
                return .7;
        else if(gamepad->GetRawButton(5))
                return .5;
        else if(gamepad->GetRawButton(7))
                return .1;
        else
                return .3;
}
int Gamepad::GetPressedButton(uint32_t range1, uint32_t range2, bool remember){
        if(!remember)
                pressedButton=0;

        for(uint32_t x=range1; x <= range2; x++){ // x is initialized at range1, loop executes while x is less than range2, adds to x every time looped
                if(gamepad->GetRawButton(x)) // If button x pressed
                        pressedButton=x; // The pressed button is that one
        }
        return pressedButton; // Return pressed button
}
