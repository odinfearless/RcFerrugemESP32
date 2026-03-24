
class SonarSensor
{
public:
    SonarSensor(unsigned int trigPin, unsigned int echoPin);   
    unsigned int getTrigPin();
    unsigned int getEchoPin();   
    int read();
    void begin();

private:
    unsigned int trigPin;
    unsigned int echoPin;  
};
