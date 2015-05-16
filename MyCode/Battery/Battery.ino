int sensorpin = 4;                 // analog pin used to connect the sharp sensor
int val = 0;                 // variable to store the values from sensor(initially zero)
float distance = 0; 
float voltage = 0;

void setup()
{
  Serial.begin(9600);               // starts the serial monitor
}
 
void loop()
{
  
  for (int i = 0; i<6; i++)
  {
  val = analogRead(sensorpin);   // reads the value of the sharp sensor
  voltage = (val/1023) * 5;
  distance = 27.0570*pow(voltage,-1.1811);//pow(((val*(5/1023.0)*0.001221)/16.251),1.1765);
  Serial.print("Sensor ");
  Serial.print(sensorpin);
  Serial.print(" analog ");
  Serial.print(val);
  Serial.print(" voltage ");
  Serial.print(voltage);
  Serial.print(" distance ");
  Serial.println(distance);            // prints the value of the sensor to the serial monitor
  delay(1000);                    // wait for this much time before printing next value
  }

}
