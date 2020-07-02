// Yasteer Sewpersad 
// Mood Lighting Sketch

/* Materials Required:
 *  1x Potentiometer 
 *  3x Pushbuttons
 *  1x RGB LED --> Common Anode --> Requires Inverted PWM.
 *  3x 300 Ohm Resistors
 *  1x Uno
 */

int Potentiometer_Measurement = 0;
unsigned int Scaled_Pot_Measurement = 0;
unsigned int RED,GREEN,BLUE; // Variables will hold duty cycles for each colour. 
bool isRedPressed, isGreenPressed, isBluePressed;

void setup() {
  Serial.begin(9600);
  isRedPressed = false;
  isGreenPressed = false;
  isBluePressed = false;
  
  // ADC Setup
  ADMUX =  (0 << REFS1) | (1 << REFS0); // Use 5V reference, left adjust ADCH register,  and use default Channel 0 for input
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Turn on the ADC & use a prescaler of 128. The crystal oscillator uses a frequency = 16Mz so Sampling Frequency = 16Mz/128 = 125kHz
  
  // Define Pushbutton Input Pins --> Will control intensity of each colour. 
  PCICR  |= (1<<PCIE2); // Enable Pin Change Interrupts on PORTD as the push buttons will only be pressed once in a while
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT22) | (1 << PCINT23); // Specify pins PD4, PD6, PD7.
  sei();

  // Setup Timers --> They don't allow you to write to the OCR register and write to the PWM Pin so will have to use 3 timers to control each colour. --------> CHECK FOR A BETTER WAY!
  TCNT0 = 0;
  DDRD = DDRD | B00100000;   // Output PWM PD5
  TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM01) | (1 << WGM00); // Put Timer/Counter into Fast PWM mode(inverting) with TOP value set by OCRA register. Use Pins 5 & 6 as PWM outputs. 
  TCCR0B = (1 << WGM02)  | (1 << CS02)   | (0 << CS01)   | (1 << CS00); // 1024 prescale.

  TCNT1 = 0;
  DDRB  = DDRB | B00000100;  // Output PWM PB2
  TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (0 << WGM10); 
  TCCR1B = (0 << ICNC1)  | (0 <<ICES1)   | (1 << WGM13)  | (1 << WGM12)  | (1 << CS12)  | (0 << CS11)  | (1 << CS10); // 1024 prescale.

  TCNT2 = 0;
  DDRD = DDRD | B00001000; // Output PWM PD3
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21) | (1 << WGM20); 
  TCCR2B = (1 << WGM22)  | (1 << CS22)   | (1 << CS21)   | (1 << CS20); // 1024 prescale.
}

void loop() {
  if(isBluePressed == true)
  {
    BLUE = (Read_ADC()/256); // Convert to an 8-bit number. 
    if(BLUE > 255) BLUE = 255;
    if(BLUE < 0) BLUE = 0;
    OCR0A = 255 - BLUE;
  }
  
  if(isGreenPressed == true)
  {
    GREEN = Read_ADC();
    if(GREEN > 65535) GREEN = 65535;
    if(GREEN < 0) GREEN = 0;
    ICR1 = 65535 - GREEN;
  }
  
  if(isRedPressed == true)
  {
    RED = (Read_ADC()/256);
    if(RED > 255) RED = 255;
    if(RED < 0) RED = 0;
    OCR2A = 255 - RED;
  }
 
}

ISR (PCINT2_vect) { // Interrupts detect rising or falling edges. 
  // Pushbuttons are prone to bouncing which can cause erroneaous readings so this must be corrected in software or externally via a debouncing circuit. 
  if((PIND & B00010000)){ // Button was pressed.
    isRedPressed = !isRedPressed; // Toggle state.
  }
  if(PIND & B01000000){
    isGreenPressed = !isGreenPressed;
  }
  if(PIND & B10000000){
    isBluePressed = !isBluePressed;
  }
}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  Scaled_Pot_Measurement  = ADC * 64; // Scales a 10-bit ADC value into a 16-bit range.
  return Scaled_Pot_Measurement;
}
