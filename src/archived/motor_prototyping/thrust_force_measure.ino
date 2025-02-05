// Full thrust-stand script for measuring motor thrust force

#include <Servo.h>
#include <HX711.h>

// Pin definitions
#define LOADCELL_DOUT_PIN  3
#define LOADCELL_SCK_PIN   2
#define ESC_PIN            9

// Create objects
HX711 scale;
Servo esc;

// Calibration factor for the load cell
float calibration_factor = -410; // Adjust as needed (separate calibration program required)

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize ESC
  esc.attach(ESC_PIN);
  Serial.println("Starting ESC calibration...");
  
  // ESC calibration routine (initializing with neutral signal)
  esc.write(93); // Replace 93 with your ESC's neutral value if needed
  delay(1000);   // Wait for the ESC to confirm arming (listen for beeps)
  Serial.println("ESC armed and ready!");

  // Run the motor at a neutral value initially
  esc.writeMicroseconds(1500); // Neutral throttle
  delay(2000); // Allow motor to stabilize

  // Initialize HX711 (load cell)
  Serial.println("Initializing load cell...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(); // Initialize with default scale
  scale.tare();      // Reset scale to zero
  Serial.println("Scale tared. Remove all weight from scale.");

  // Print zero factor for debugging
  long zero_factor = scale.read_average();
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);

  Serial.println("After readings begin, place known weight on scale.");
  Serial.println("Press + or a to increase calibration factor.");
  Serial.println("Press - or z to decrease calibration factor.");
  Serial.println("Please wait while motor ramps up...");

  // Ramp up motor speed from 1000 to 1890
  for (int i = 1000; i <= 1890; i++) {
    esc.writeMicroseconds(i);
    delay(20);
  }
  Serial.println("Thrust stand is ready at full throttle!");
}

void loop() {
  esc.writeMicroseconds(1890);
  esc.writeMicroseconds(1890);
  // Apply the current calibration factor
  scale.set_scale(calibration_factor);

  // Read and print thrust values from load cell
  if (scale.is_ready()) {
    float thrust = scale.get_units(10); // Average of 10 readings
    Serial.print("Thrust: ");
    Serial.print(thrust, 2); // Print thrust with one decimal place
    Serial.print(" grams (g)"); // tolerance is +-1g (due to fluctuations)
    Serial.print(" | Calibration factor: ");
    Serial.println(calibration_factor);
  } else {
    Serial.println("Load cell not ready!");
  }
  esc.writeMicroseconds(1890);

  // Check for user input to adjust calibration factor
  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a') {
      calibration_factor += 10; // Increase calibration factor
      Serial.println("Calibration factor increased.");
    } else if (temp == '-' || temp == 'z') {
      calibration_factor -= 10; // Decrease calibration factor
      Serial.println("Calibration factor decreased.");
    }
  }
  esc.writeMicroseconds(1890);

  delay(100); // Delay for readable output
}