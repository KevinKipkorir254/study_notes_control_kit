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

/*.......................REPLACE CONTROLLER INITITALIZATION HERE.........................................*/
//DATA STORAGE
double state_space[2] = { 0.0, 0.0};
double state_space_total = 0.0;
double gains_[2] = { 102.0, 10.0746}; 
double error_integral = 0.0;

/*.......................REPLACE CONTROLLER INITITALIZATION HERE.........................................*/


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

    while (!Serial) {
    // Do nothing, just wait
  }
  
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

  unsigned long startTime = millis();

  // Keep printing '0' for 5 seconds
  while (millis() - startTime < 5000) { // 5000 milliseconds = 5 seconds
    Serial.println(0);
    delay(100); // Delay a bit between prints (adjust as needed)
  }

  // Attach interrupt to the encoder A pin
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);
  delay(5000);

  // Create tasks
  //xTaskCreate( feedtask, "feedtask1", 1000, NULL, 1, &Task1);
}

void loop() 
{
  // put your main code here, to run repeatedly:
    //OBTAINING THE DATA FEEDBACK
    //filtering the data 
    double yn = 0.969 * yn_1 + 0.0155 * velocity + 0.0155 * xn_1;
    xn_1 = velocity;
    yn_1 = yn;

    //velocity time
    current_position = ((float)encoderCount / 500) * 3.147;
    velocity = (current_position - radians_)/(0.01);   
    radians_ = current_position; 
    


/*.........................................REPLACE HERE FOR THE CONTROLLER..........................................................*/
    //computing the controller values
    //replacing the values
    //computing the error
    double ref = 2.5;
    double error = ref - yn;
    state_space[0] = yn;
    state_space[1] = yn - yn_1;
    state_space_total = ((error*gains_[0]));
    
    double Khat_result = state_space[1] * gains_[1];

    double controller_output = state_space_total - Khat_result;
/*.........................................REPLACE HERE FOR THE CONTROLLER..........................................................*/
    
      int feed = (int)controller_output;//random K value
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
    
    Serial.println(yn);

    delay(10);
}

