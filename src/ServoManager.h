//
// Created by Tatsuya Iwai (GreySound) on 6/7/19.
//

#ifndef SPC2019_PARACHUTE_MBED_SERVOMANAGER_H
#define SPC2019_PARACHUTE_MBED_SERVOMANAGER_H

#include <mbed.h>

namespace greysound {

class ServoManager {

/**
 * MIN_VALUE, MAX_VALUE はデューティ比から求める。(0.0-1.0f)
 * 周期 20ms, 制御パルス範囲が 1.02ms-2.02ms のサーボ（FUTABA）の場合、
 * 有効範囲は 1.02/20=0.051, 2.02/20=0.101 となる
 */

//GWSサーボ PIC/STD/F: 1.02ms-2.02ms
//#define MIN_VALUE 0.051f
//#define MAX_VALUE 0.101f

//マイクロサーボ　SG92R: 0.5ms-2.4ms
#define SERVO_PERIOD 0.020f
#define MIN_VALUE 0.025f
#define MAX_VALUE 0.120f

#define PERIOD_FUTABA 0.020f // Futaba (=20ms)

    public:

        /**
         * constructor
         * @param control
         */
        explicit ServoManager(PinName control)
        {
            flag = 0;
            minValue = MIN_VALUE;
            maxValue = MAX_VALUE;
            servo = new PwmOut(control);
            servo->period(SERVO_PERIOD);
        }

        /**
         * destructor
         */
        ~ServoManager() {
            delete(servo);
        }

        //MEMO: 今のところ必要性を感じていない
        void init()
        {
            // move to "close" position
            servo->write(minValue);   // 1.02ms(0.051f) < 1.52ms(0.076f) > 2.02ms(0.101f)
            //_servo.pulsewidth(0.00152f);   // alternative to led.write, set duty cycle time in seconds
        }

        bool setPeriod(float _period)
        {
            // validation: only accepts 0 < _period <= 1.0f
            if (_period <= 0 || _period > 1.0f) {
                return false;
            }

            servo->period(_period);  // Futaba: 20ms (0.020f)

            return true;
        }

        bool setRange(float _min, float _max)
        {
            // validation
            if(_min < MIN_VALUE || _max > MAX_VALUE || _min > _max)
            {
                return false;
            }

            minValue = _min;
            maxValue = _max;

            return true;
        }

        void moveRight() // open
        {
            flag = 1;
            servo->write(minValue);
        }

        void moveLeft() { // close

            flag = 0;
            servo->write(maxValue);
        }

        bool moveTo(float _value)
        {
            if(_value < MIN_VALUE || _value > MAX_VALUE)
            {
                return false;
            }

            servo->write(_value);

            return true;
        }

        // flip servo position depends on its flag state
        void flipState()
        {
            flag == 1 ? this->moveLeft() : this->moveRight();
        }

        bool getState()
        {
            return flag;
        }

    protected:
    private:
        uint8_t flag;
        float minValue, maxValue;
        PwmOut *servo;

    };
}


#endif //SPC2019_PARACHUTE_MBED_SERVOMANAGER_H
