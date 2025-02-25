const int redPhotodiodePin = A0;
const int irPhotodiodePin1 = A1;
const int irPhotodiodePin2 = A2;
//const int irPhotodiodePin3 = A3;

int rawRed = 0;
int rawIR1 = 0;
int rawIR2 = 0;
//int rawIR3 = 0;

float redVoltage = 0;
float irVoltage1 = 0;
float irVoltage2 = 0;
//float irVoltage3 = 0;


void setup() {
 // put your setup code here, to run once:
 Serial.begin(9600); //baud rate
}

void loop() {
 // put your main code here, to run repeatedly:

 //we can have 2 photodiodes, one for red LED and 3 for IR LEDs
 rawRed = analogRead(redPhotodiodePin);
 rawIR1 = analogRead(irPhotodiodePin1);
 rawIR2 = analogRead(irPhotodiodePin2);
 //rawIR3 = analogRead(irPhotodiodePin3);


 //for testing
// Serial.print("Red photodiode: ")
// Serial.println(rawRed)
// Serial.print("IR photodiode: ")
// Serial.println(rawIR)

 redVoltage = (rawRed / 1023.0) * 3.3; //convert raw analog value to voltage
 irVoltage1 = (rawIR1 / 1023.0) * 3.3;
 irVoltage2 = (rawIR2 / 1023.0) * 3.3;
 //irVoltage3 = (rawIR3 / 1023.0) * 5.0;


 //for testing
// Serial.print("Red voltage: ")
// Serial.println(redVoltage);
// Serial.print("IR voltage: ")
// Serial.println(irVoltage)

 //to send to Matlab
 Serial.print(redVoltage);
 Serial.print(",");
 Serial.print(irVoltage1);
 Serial.print(",");
 Serial.println(irVoltage2);
 //Serial.print(",");
 //Serial.println(irVoltage3);

 delay(5); //sampling interval --> 200 Hz sampling rate




}
