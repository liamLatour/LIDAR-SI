/*TSL1401R

  Linear Array Sensor TSL1401
  -128 x 1 Linear Sensor Array 200 DPI

  -Sketch used with the Arduino Uno.
  Data is Send through I2C to the master arduino
  One frame (128x1 image) has the following format:

      843      this value is discarded.
      234      value of the pixel 1
      242      value of the pixel 2
       .         .
       .         .
      245      value of the pixel 128

  -Reading only one pixel in each scan.
  -The image is completed after 1+128 scans.(first is discarded)
  -First scan is repeated to permit a complete integration period).

  -For more information refer to the comments.docx file.
*/

#include <Wire.h>
#include <digitalWriteFast.h>
#include <avdweb_AnalogReadFast.h>

#define CLK     2 // clock output on pin 2
#define SI      3 // "start integrating" signal on pin 3
#define VOUT    A0 // pixel intensity in the analog channel 0
#define PIXELS  128 // number of pixels in the sensor

int indexTable[] = {
  34, 35, 36, 36, 37, 39, 41, 42, 43, 45,
  46, 48, 49, 53, 55, 56, 60, 65, 67, 68,
  70, 74, 81, 86, 90, 97, 104, 111, 128, 140,
  158, 184, 215, 250, 300, 315, 330, 345, 370, 400
  }; //distance in cm for each index value

int intDelay = 50; // integration period
int Value[PIXELS]; // pixel intensity values
int distance; // measured distance

void setup() {
  pinModeFast(CLK, OUTPUT);
  pinModeFast(SI, OUTPUT);
  digitalWriteFast(CLK, LOW);
  digitalWriteFast(SI, LOW);

  Wire.begin(2); // joins I2C bus with adress #2
  Wire.onRequest(requestEvent); // registers request event

  Serial.begin(115200);
  
}

void requestEvent() { // executed when master asks for a distance measurement
  Wire.write(distance); // sends the measured distance over the I2C bus
  Wire.write((distance >> 8)); // 
}

void loop() {
  int iIndex = 0; // interpolated index
  int maxIndex = 0; // saves max index value
  int maxP = 0; // saves max pixel value
  unsigned int total = 0; // used to calculate average
  int BmaxP = 0; // value of the pixel before the maximum
  int AmaxP = 0; // value of the pixel after the maximum

  readPixel(0); // the first reading will be discarded to remove noise from the pixels
  
  delayMicroseconds(intDelay);

  for (int pixel = 0; pixel < PIXELS; pixel++) { // acquires one frame
    readPixel(pixel);
    total += Value[pixel];

    if (maxP < Value[pixel]) { // finds the peak in the frame and saves its index
      maxP = Value[pixel];
      maxIndex = pixel;
    }
  }

  //Serial.println(float(maxP) / (float(total)/float(PIXELS)));

  if(float(maxP) / (float(total)/float(PIXELS)) < 1.05){
    distance = 400;
  }
  
  BmaxP = Value[(maxIndex - 1)];
  AmaxP = Value[(maxIndex + 1)];

  if (BmaxP > AmaxP) { // if the pixel before has more light than the one after the beam is closer to it and the distance is lower
    iIndex = ((maxIndex - 0.5) * 2); // result is multiplied by 2 to make it an int, array indices can't be floats
  }
  else if (BmaxP < AmaxP) { // if the pixel after has more light than the one before the beam is closer to it and the distance is higher
    iIndex = ((maxIndex + 0.5) * 2); // same as before
  }
  else { // the peak is centered and therefore no adjustments are required
    iIndex = maxIndex * 2; // as the two above return a number twice as high this one needs to do the same
  }
/*
  if (iIndex < 95) { // this clamps the measured distance so it stays within the table's range
    iIndex = 95;
  }
  else if (iIndex > 134) {
    iIndex = 134;
  }
*/
  float tempDistance = (0.0042*pow(iIndex,3) - 1.2997*pow(iIndex,2) + 136.24*iIndex - 4755.4)/2.0 - 4;//indexTable[(iIndex - 95)]
  if(tempDistance > 25 && tempDistance < 200){
    distance = tempDistance;
    //Serial.println(tempDistance);
  }
}

//------------------ Operates the camera module -----------------//
void readPixel(int pixel) { // acquires pixel number "pixel"
  digitalWriteFast(CLK, LOW); // initialises the acquisition procedure
  digitalWriteFast(SI, HIGH);
  digitalWriteFast(CLK, HIGH);
  digitalWriteFast(SI, LOW);


  for (int i = 0; i < pixel; i++) { // moves the SI pulse in the camera's shift register to the correct pixel
    digitalWriteFast(CLK, LOW);
    digitalWriteFast(CLK, HIGH);
  }

  Value[pixel] = analogReadFast(VOUT); // reads the pixel's value and stores it

  delayMicroseconds(8); // added to stabilize the reading
  
  for (int i = 0; i <= (PIXELS - pixel); i++) { // clocks the SI pulse out of the camera's shift register
    digitalWriteFast(CLK, LOW);
    digitalWriteFast(CLK, HIGH);
  }
}
