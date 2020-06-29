// Yasteer Sewpersad 
// Mood Lighting Sketch

/* Materials Required:
 *  1x Potentiometer 
 *  3x Pushbuttons
 *  1x RGB LED
 *  3x 300 Ohm Resistors
 *  1x Uno
 */

int Potentiometer_Measurement = 0;
unsigned int Scaled_Pot_Measurement = 0;
unsigned int RED,GREEN,BLUE; // Variables will hold duty cycles for each colour. 
bool isRedPressed, isGreenPressed, isBluePressed;

void setup() {
  Serial.begin(9600);\
  
  // ADC Setup
  ADMUX =  (0 << REFS1) | (1 << REFS0); // Use 5V reference, left adjust ADCH register,  and use default Channel 0 for input
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Turn on the ADC & use a prescaler of 128. The crystal oscillator uses a frequency = 16Mz so Sampling Frequency = 16Mz/128 = 125kHz
  
  // Define Pushbutton Input Pins --> Will control intensity of each colour. 
  PCICR  |= (1<<PCIE2); // Enable Pin Change Interrupts on PORTD as the push buttons will only be pressed once in a while
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20); // Specify pins.
  
  // Define RGB Pins
  DDRB = DDRB | 0b00001110; // Set three output pins and keep the others as input. 
}

void loop() {
  
  if(isRedPressed == true)
  {
    RED = Read_ADC();
  }
  else if(isGreenPressed == true)
  {
    GREEN = Read_ADC();
  }
  else if(isBluePressed == true)
  {
    BLUE = Read_ADC();
  }
  
  // Generate PWM Signals
  
  // Set the values of the RGB LED
  PORTB |= (1 << PORTB1);
}

ISR (PCINT2_vect) { // Interrupts detect rising or falling edges. Pushbuttons are prone to bouncing which can cause erroneaous readings so this will be corrected in software. 
  if((PIND & B00000100)){ // Button was pressed.
    isRedPressed = !isRedPressed; // Toggle state.
  }
  if(PIND & B00001000){
    isGreenPressed = !isGreenPressed;
  }
  if(PIND & B00010000){
    isBluePressed = !isBluePressed;
  }
}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  Potentiometer_Measurement = ADC; // ADC contains the combined 10 bit value of the ADCH & ADCL registers.
  Scaled_Pot_Measurement  = ADC * 64; // Scales a 10-bit ADC value into a 16-bit range. This will allow us to feed it into a 16-bit timer and make use of the entire range of values. 
  return Scaled_Pot_Measurement;
}
