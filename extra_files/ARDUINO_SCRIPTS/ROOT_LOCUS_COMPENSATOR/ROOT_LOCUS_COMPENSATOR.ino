
// Task handles
TaskHandle_t Task1;

// Define GPIO pins for encoder A and B channels
#define ENC_A_PIN 39
#define ENC_B_PIN 36

//motor control pins
int IN1 = 5;
int IN2 = 4;
int enablePin = 15;

// Define PWM parameters
const int freq = 5000; // PWM frequency in Hz
const int resolution = 8; // PWM resolution in bits (8, 10, 12, 15 bits are supported)

// Define variables to store encoder counts and direction
volatile int encoderCount = 0;

//speed calculations
double radians_ = 0.0;
double current_position = 0.0;
double velocity = 0.0;

//Low pass filter data
double yn_1 = 0, xn_1 = 0;

// Sample time in seconds
const float delta_t = 0.01;

//ANALOG SENSOR INPUT
const int analogInPin = 2;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;  // value read from the pot
double outputValue = 0;  // value output to the PWM (analog out)

//G_C COMPENSATOR
double G_c_output[2] = { 0.00, 0.9066};
double G_c_input[2] = { 238, -204.34};

double u_gc[3] = { 0.0, 0.0, 0.0};
double y_gc[3] = { 0.0, 0.0, 0.0};


// Interrupt Service Routine (ISR) for encoder A pin
void IRAM_ATTR encoderISR() {
  // Read the state of the A and B channels
  int bState = digitalRead(ENC_B_PIN);

    // If A is high, check B to determine direction
    if (bState == HIGH) {
      encoderCount--; // Counterclockwise
    } else {
      encoderCount++; // Clockwise
    }
  }

 //Tasks 
void feedtask(void *pvParameters);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Initialize PWM channel
  ledcSetup(0, freq, resolution);
  ledcAttachPin(enablePin, 0);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode( IN1, OUTPUT);
  pinMode( IN2, OUTPUT);
  digitalWrite( IN1, HIGH); 
  digitalWrite( IN2, HIGH); 

  // Set encoder pins as inputs
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  // Attach interrupt to the encoder A pin
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  // Create tasks
  //xTaskCreate( feedtask, "feedtask1", 1000, NULL, 1, &Task1);
}

void loop() 
{
  // put your main code here, to run repeatedly:
   //OBTAINING THE REFERENCE
   // read the analog in value:
    sensorValue = analogRead(analogInPin);
    outputValue = sensorValue * (6.5 / 4095.0);
    

    //OBTAINING THE DATA FEEDBACK
    //filtering the data 
    double yn = 0.969 * yn_1 + 0.0155 * velocity + 0.0155 * xn_1;
    xn_1 = velocity;
    yn_1 = yn;

    //velocity time
    current_position = ((float)encoderCount / 500) * 3.147;
    velocity = (current_position - radians_)/(0.01);   
    radians_ = current_position; 
 
    //ERROR
    double ref = 1.4;
    double error = ref - yn;

    //GC_Controller   
     //double G_c_output[2] = { 0.00, 0.9066};
     //double G_c_input[2] = { 238, -204.34};
     u_gc[0] = error;
     y_gc[1] = y_gc[0];
     y_gc[0] = G_c_output[1]*y_gc[1] + G_c_input[0]*u_gc[0] + G_c_input[1]*u_gc[1];
     u_gc[1] = u_gc[0];

    
      int feed = (int)y_gc[0];
      ledcWrite(0, feed);
        if(255 > 0)
        {
           digitalWrite(IN1, HIGH);
           digitalWrite(IN2, LOW);
        }
        else if(255 < 0)
        {
           digitalWrite(IN2, HIGH);
           digitalWrite(IN1, LOW);
        }
    
    //OUTPUTTING THE DATA
    //Serial.print(feed); 
    //Serial.print(" ");
    Serial.print(ref); 
    Serial.print(" ");
    //Serial.print(error); 
    //Serial.print(" ");
    Serial.println(yn);

    delay(10);
}

