
class PID {
public:
    float kp;
    float ki;
    float kd;
    float targetpoint;
    //Last time error
    float prevError;
    //integral
    float intergral;
    //differential
    float derivative;

    PID(float kp, float ki, float kd);

    void Set_PID(float kp, float ki, float kd);

    /**
     * pid calculation function
     * @param target
     * @param current
     * @return
     */
    float compute(float target, float current);

    /**
     *  Reset all errors: When the set speed is different from the last time
     */
    void reset();
};
