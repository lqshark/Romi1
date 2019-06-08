#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class Line_Sensor
{
  public:
    //Constructor
    Line_Sensor(int pin);
    //Calibrate
    void calibrate();
    //Return the uncalibrated value from the sensor
    int read_raw();
    //Return the calibrated value from the sensor
    int read_calibrated();
    float average = 0;
    
  private:
  
    int pin;
    float sensor_now;
    float sum = 0;
    
    float min_value = 0;
    float max_value = 1000;
    float read_standard;
    /*
     * Add any variables needed for calibration here
     */
    
};

Line_Sensor::Line_Sensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

int Line_Sensor::read_raw()
{
  return analogRead(pin);
}

void Line_Sensor::calibrate()
{
  /*
   * Write code to calibrate your sensor here
   */
   for(int i=0;i<500;i+=1)
   {
    sensor_now = analogRead(pin);
    sum += sensor_now;
   }
   average = sum/500;
   sum = 0;
}

int Line_Sensor::read_calibrated()
{
  /*
   * Write code to return a calibrated reading here
   */
   read_standard = analogRead(pin)-average;
   return read_standard;
}


#endif
