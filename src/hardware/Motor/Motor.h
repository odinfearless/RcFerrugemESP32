class Motor
{
public:
    Motor(unsigned int pinA, unsigned int pinB, unsigned int pinPwm, unsigned int maxLimitPwm, unsigned int pwmDeadZone);
    void forward();
    void backward();
    void setValue(int val);   
    unsigned int getPinA();
    unsigned int getPinB();
    unsigned int getPinPwm();
    void begin();

private:
    unsigned int pinA;
    unsigned int pinB;
    unsigned int pinPwm;
    unsigned int minLimitPwm;
    unsigned int maxLimitPwm;
    unsigned int pwmDeadZone;
};
