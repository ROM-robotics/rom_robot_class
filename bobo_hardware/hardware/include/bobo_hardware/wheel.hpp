#ifndef ROM_WHEEL_HPP
#define ROM_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
    public:

        std::string name = "";
        int encoder_counts = 0;

        double desire_velocity_rds = 0, actual_velocity_rds = 0; // radian per second

        short desire_velocity_rpm = 0, actual_velocity_rpm = 0;

        double desire_position = 0, actual_position = 0;        // radian unit

        double desire_torque = 0; double actual_torque = 0;

        float p_gain = 0.0, i_gain = 0.0, d_gain = 0.0;

        double rads_per_count = 0;  // count တချက်မှာရှိမည့် radian unit

        double rds_to_rpm_ratio = 60.0/(2*M_PI);
        double rpm_to_rds_ratio = (2*M_PI)/60.0;

        Wheel() = default;

        Wheel(const std::string &wheel_name, int counts_per_rev)
        {   
            setup(wheel_name, counts_per_rev);
        }

        void setup(const std::string &wheel_name, int counts_per_rev)
        {   // count တချက်မှာ radian ဘယ်လောက်ရှိလဲတွက်ပါတယ်။ ဥပမာ ဘီးတပါတ်မှာ 6533 counts ရှိတယ်ဆိုရင် 0.000961733 radian ရှိပါတယ်။
            name = wheel_name;
            rads_per_count = (2*M_PI)/counts_per_rev;
        }

        double getActualPosition(const int enc_counts)
        {   // လက်ရှိ ရောက်နေတဲ့ encoder count အရေအတွက် ကို radian အဖြစ်ပြောင်းလည်းတွက်ချက်တယ်။  
            encoder_counts = enc_counts;
            actual_position = encoder_counts * rads_per_count;
            return actual_position; // radian
        }

        int getActualRpm()
        {
            actual_velocity_rpm = short(actual_velocity_rds * rds_to_rpm_ratio); // calculate with constant
            return actual_velocity_rpm;
        }

        double getActualRadianPerSecond()
        {
            actual_velocity_rds = double(actual_velocity_rpm * rpm_to_rds_ratio); // calculate with constant
            return actual_velocity_rds;
        }

        short getDesireRpm()   // to MCU
        {
            return ((short)(desire_velocity_rds * rds_to_rpm_ratio));
        }
    private:
        short voltage; short min_voltage; short max_voltage;
        float current;
        float currentI;
};


#endif // bobo_hardware_WHEEL_HPP