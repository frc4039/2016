#include "WPILib.h"
#include <math.h>

enum GamepadButtons{
        RightPadBottom=1,
        RightPadRight=2,
        RightPadLeft=3,
        RightPadTop=4,
        TopLeftBumper=5,
        TopRightBumper=6,
        BottomLeftBumper=7,
        BottomRightBumper=8,
        CenterPadLeft=9,
        CenterPadRight=10,
        CenterPadBottom=11,
        LeftJoyButton=12,
        RightJoyButton=13
};

class Gamepad{
        private:
                Joystick *gamepad;
                int pressedButton, count[11];
                bool previousState[11];
        public:
                Gamepad(uint32_t usbChannel);

                float GetLX(); // Returns value of left joypad X axis
                float GetLY(); // Returns value of left joypad Y axis
                float GetRX(); // Returns value of right joypad X axis
                float GetRY(); // Returns value of right joypad Y axis
                float GetDpadX(); // Returns value of dpad X axis
                float GetDpadY(); // Returns value of dpad Y axis       
                float GetCombinedX();
                float GetAxis(uint32_t axis); // Returns value of requested axis
                float GetJoypadsX();

                bool GetDpadMoved();
                bool GetLstickMoved();
                bool GetRstickMoved();
                bool GetActive();

                int GetLDegrees();
                float GetLMagnitude();
                int GetRDegrees();
                float GetRMagnitude();

                bool GetButton(uint32_t button); // Returns value of requested button

                bool GetButtonTripped(uint32_t button);
                bool GetButtonReleased(uint32_t button);
                
                float GetThrottle(); // Returns value of throttle
                float GetLeftThrottle(); // Returns value of left side of throttle
                int GetPressedButton(uint32_t range1, uint32_t range2, bool remember=true); // Gets which button between two ranges is pressed
};
