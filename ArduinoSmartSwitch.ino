#include <EEPROM.h>
/*
  Smart Switch Solid State

  1.      Pinos

    Botão Programação               - D 13
    Potenciometro Pino Central      - A 2
    Debug Light                     - D 10
    SS Relay CH1                    - D 2
    SS Relay CH2                    - D 3
    SS Relay CH3                    - D 4
    SS Relay CH4                    - D 5
    SS Relay CH5                    - D 6
    SS Relay CH6                    - D 7
    SS Relay CH7                    - D 8
    SS Relay CH8                    - D 9

  2.      Memory Map

    Potentiometer Min Value         -  0x00         (2 bytes)
    Potentiometer Max Value         -  0x02         (2 bytes)
    Quantity of Programs            -  0x04         (1 byte)
    Programs (10 programs)          -  0x05~0x0E    (1 byte)
    Invert Potentiometer            -  0x0F         (1 byte - 0 no 1 yes)

  3.      Observations

    a. Relay turn lights off if the PIN is set to HIGH
    b. CH 4 at pin 7
    b. CH 3 at pin 6
    b. CH 2 at pin 5
    b. CH 1 at pin 4
    b. CH 5 at pin 9

  4.      Led Signals

    State                                          Long      Brief
    --------------------------------------------------------------------
    normal mode                                      1         2
    program_mode_programs                            1         4
    program mode switch: PROGRAM mode                -         4
    program mode switch: CALIBRATION mode            -         6
    program mode switch: INVERT mode                 -         8
    reset arduino                                    -         20(very fast)

  5. Intructions

    Em Modo Normal (-..)
    - Pressione o botão 1 vez rapidamente para reiniciar o dispositivo (A luz pisca rapidamente 20 vezes)
    - Pressione o botão pode 4~5 segundos para entrar no modo de programação
    - Pressione o botão pode 2~3 segundos mostrar os programas gravados

    Em Moco de Programação : Adição de Programas (-....)
    - Para cada programa, defina o valor de cada canal usando o potenciometro e defina o valor do
      canal pressionando rapidamente o botão. A luz pisca indicando qual o bit foi definido
    - Pressione o botão 4~5 segundos para ir para o modo calibração
    - Pressione o botão 10 segundos para ir para o modo normal

    Em Moco de Programação : Modo de Calibração (-......)
    - Mova o potenciometro o máximo para a posição OFF para definir o limite mínimo e pressione o botão. A luz pisca uma vez indicando
      que o limite mínimo foi definido
    - Mova o potenciometro o máximo para a posição ON para definir o limite mínimo e pressione o botão. A luz pisca duas vez indicando
      que o limite mínimo foi definido
    - Segure o botão de 2~3 segundos para abrir o modo Inverter potenciometro
    - Segure o botão de 4~5 segundos para voltar para o modo normal

    Em Moco de Programação : Modo de Calibração (-........)
    - Mova o potenciometro para esquerda ou direita para ligar ou desligar o inversor de potenciometro. O led ligado indica SIM INVERTER
      potenciometro e o led desligado significa NÂO INVERTER (Modo Normal). Pressione o botão para gravar. O led pisca pisca duas vezes
      para sinalizar que o valor foi gravado
    - Segure o botão de 4~5 segundos para voltar para o modo normal

*/

// ================================
// Fixed Definitions
// ================================
#define LED_PIN A0
#define BUTTON_PIN 13
#define POTENTIOMETER_PIN 2
#define RELAY_CH_1 4      
#define RELAY_CH_2 5           
#define RELAY_CH_3 6          
#define RELAY_CH_4 7          
#define RELAY_CH_5 9 
#define RELAY_CH_6 10  
#define RELAY_CH_7 11  
#define RELAY_CH_8 12

#define MEM_POT_CAL_MIN 0
#define MEM_POT_CAL_MAX 2
#define MEM_QTY_PROGRAMS 4
#define MEM_PROG_INIT 5
#define MEM_INV_POT 15

#define MAX_POTENTIOMETER_VALUE 31
#define POTENTIOMETER_DEBUMP 10

// ================================
// Global Variables
// ================================

// General
int chipDelay = 5;                        // Prevent arduino to run at light speed
int buttonCount = 0;                      // Count how many time the button was held
enum                                      // States                                        
{                                         
  normal ,
  program_mode_programs,                                
  program_mode_calibration,
  program_mode_invert_potentiometer,
} currentState;
bool isProgramMode = false;
int btnDelaySmall[] = {5,300};            // Instant
int btnDelayMedium[] = {301,600};         // 2 Seconds
int btnDelayLong[] = {601,20000};         // 4 Seconds
bool calIsStart = true;
bool invertPotentiometer = false;
byte defaultPrograms[] = {
  B00000000, 
  B11111111
};
byte lastReadValue = 0;
byte lastValue = 0;

bool currentProgramBitValue = false;
byte currentProgramBit = 0;
bool currentProgramBitsArray[8] = {false,false,false,false,false,false,false,false};

// ================================
// Setup
// ================================
void setup(){  
  Serial.begin(9600);
  setupPins();
  setupDisplayEpromValues();
  setupInitialPrograms();
  
  currentState = normal;
}

void setupPins(){
  Serial.println(">>>> Hardware Setup <<<<");
  
  // LED
  Serial.print("LED > Output > D"); Serial.println(LED_PIN);
  pinMode(LED_PIN,OUTPUT);

  // Botão
  Serial.print("BUTTON > Input PullUP > D"); Serial.println(BUTTON_PIN);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  // Potentiometer
  Serial.print("POTENTIOMETER > Input > A"); Serial.println(POTENTIOMETER_PIN);
  pinMode(POTENTIOMETER_PIN,INPUT);

  // Solid State Pins
  Serial.print("SSR CH 1 > Output > D"); Serial.println(RELAY_CH_1);
  pinMode(RELAY_CH_1,OUTPUT);
  Serial.print("SSR CH 2 > Output > D"); Serial.println(RELAY_CH_2);
  pinMode(RELAY_CH_2,OUTPUT);
  Serial.print("SSR CH 3 > Output > D"); Serial.println(RELAY_CH_3);
  pinMode(RELAY_CH_3,OUTPUT);
  Serial.print("SSR CH 4 > Output > D"); Serial.println(RELAY_CH_4);
  pinMode(RELAY_CH_4,OUTPUT);
  Serial.print("SSR CH 5 > Output > D"); Serial.println(RELAY_CH_5);
  pinMode(RELAY_CH_5,OUTPUT);
  Serial.print("SSR CH 6 > Output > D"); Serial.println(RELAY_CH_6);
  pinMode(RELAY_CH_6,OUTPUT);
  Serial.print("SSR CH 7 > Output > D"); Serial.println(RELAY_CH_7);
  pinMode(RELAY_CH_7,OUTPUT);
  Serial.print("SSR CH 8 > Output > D"); Serial.println(RELAY_CH_8);
  pinMode(RELAY_CH_8,OUTPUT);

  // Turn all the ligths off
  digitalWrite( RELAY_CH_1,HIGH );
  digitalWrite( RELAY_CH_2,HIGH );
  digitalWrite( RELAY_CH_3,HIGH );
  digitalWrite( RELAY_CH_4,HIGH );
  digitalWrite( RELAY_CH_5,HIGH );
  digitalWrite( RELAY_CH_6,HIGH );
  digitalWrite( RELAY_CH_7,HIGH );
  digitalWrite( RELAY_CH_8,HIGH );  
}

void setupDisplayEpromValues(){
  Serial.println(">>>> EEPROM Values <<<<");

  Serial.print("0x00 > MEM_POT_CAL_MIN  > "); Serial.println(readFromEeprom2Bytes(MEM_POT_CAL_MIN));
  Serial.print("0x02 > MEM_POT_CAL_MAX  > "); Serial.println(readFromEeprom2Bytes(MEM_POT_CAL_MAX));
  Serial.print("0x0F > MEM_INV_POT > "); Serial.println(readFromEeprom(MEM_INV_POT));
  Serial.print("0x04 > MEM_QTY_PROGRAMS > "); Serial.println(readFromEeprom(MEM_QTY_PROGRAMS));

  for(int i = 5 ; i < 15; i++){
      Serial.print("0x");Serial.print(i);Serial.print(" > PROGRAM > ");Serial.println(readFromEeprom(i));
  }
}

void setupInitialPrograms(){
  if( readFromEeprom(MEM_QTY_PROGRAMS) == 0 ){
    Serial.println(">>>> Writing Initial Programs <<<<");

    writeToEeprom(5,defaultPrograms[0]);
    Serial.print("0x05 > B00000000 > "); Serial.println(defaultPrograms[0]);
    writeToEeprom(6,defaultPrograms[1]);
    Serial.print("0x06 > B11111111 > "); Serial.println(defaultPrograms[1]);

    writeToEeprom(4,2);
    Serial.print("0x04 > MEM_QTY_PROGRAMS > 2");
  }
}

// ================================
// Loop
// ================================
void loop(){
  updateButton();

  if( isProgramMode and currentState == program_mode_programs ){
    updateProgram();
  }
  else if( isProgramMode and currentState == program_mode_invert_potentiometer ){
    updateInvert();
  }
  else{
    updateNormal();
  }

  
  delay(chipDelay);
}

void updateNormal(){
  if( lastValue != readPotentiometer(true) ){
    lastValue = readPotentiometer(true);
    writeToSolidStateRelayModule(getProgram(lastValue));
  }
}

void updateButton(){
  
  bool buttonIsPressed = digitalRead(BUTTON_PIN);

  if( !buttonIsPressed and buttonCount > 0 ){
     buttonPressed(buttonCount);
  }

  if( buttonIsPressed )buttonCount += 1;
  else buttonCount = 0;
}

void updateButtonCalibration(){
  int value = analogRead( POTENTIOMETER_PIN );
  if( calIsStart ){

    writeToEeprom2Bytes(MEM_POT_CAL_MIN,value);
    
    Serial.print(">> CALIBRATION MIN : ");Serial.println(value);
    calIsStart = !calIsStart;
    blinkLed(1);
  }else{

    writeToEeprom2Bytes(MEM_POT_CAL_MAX,value);
    
    Serial.print(">> CALIBRATION MAX : ");Serial.println(value);
    calIsStart = !calIsStart;
    blinkLed(2);
  }
}

void updateInvert(){
  int val = readPotentiometer(false);

  bool isNo = val <= MAX_POTENTIOMETER_VALUE/2;

  if( isNo ){
    digitalWrite(LED_PIN,LOW);
    invertPotentiometer = false;
  }else{
    digitalWrite(LED_PIN,HIGH);
    invertPotentiometer = true;
  }
  
}

void updateButtonInvertPotentiometer(){
  if( invertPotentiometer ){
    Serial.println(">> Potentiometer Invert Mode Set to ON");
    setInvertPotentiometer(1);
    blinkLed(2);
  }else{
    Serial.println(">> Potentiometer Invert Mode Set to OFF");
    setInvertPotentiometer(0);
    blinkLed(2);
  }
}

void updateProgram(){

  int val = readPotentiometer(false);
  bool isNo = val <= MAX_POTENTIOMETER_VALUE/2;

  if( isNo ){
    writeToSolidStateRelayModulePort(currentProgramBit,isNo);
    currentProgramBitValue = isNo;
  }else{
    writeToSolidStateRelayModulePort(currentProgramBit,isNo);
    currentProgramBitValue = isNo;
  }

//  if( lastReadValue != readPotentiometer(false) ){
//    // Serial.println(lastReadValue);
//    writeToSolidStateRelayModule(readPotentiometer(false));
//    lastReadValue = readPotentiometer(false);
//  }
  
}

void updateButtonProgram(){

  byte quantityPrograms = getQuantityPrograms();

  if( quantityPrograms < 10 ){
    if( currentProgramBit <= 7 ){
        // Set bit in array
        currentProgramBitsArray[currentProgramBit] = !currentProgramBitValue;
        currentProgramBit += 1;
        blinkLed(currentProgramBit);
        Serial.println(">> SET BIT");
    }else{
        // add program

        byte value = 0;

        for( int i = 0 ; i < 7 ; i++ ){
          bitWrite(value,i,currentProgramBitsArray[i]);  
        }        
        
        Serial.println(">> New Program Saved");
        setQuantityPrograms(quantityPrograms+1);
        setProgram(quantityPrograms-1,value);
        blinkLedQuick(10);

        currentProgramBitValue = false;
        currentProgramBit = 0;
        currentProgramBitsArray[0] = false;
        currentProgramBitsArray[1] = false;
        currentProgramBitsArray[2] = false;
        currentProgramBitsArray[3] = false;
        currentProgramBitsArray[4] = false;
        currentProgramBitsArray[5] = false;
        currentProgramBitsArray[6] = false;
        currentProgramBitsArray[7] = false;

        setProgram(0,0);
        setProgram(quantityPrograms,255);
    }  
  }

  
  
//  byte quantityPrograms = getQuantityPrograms();
//  
//  if( quantityPrograms < 10 ){
//    setQuantityPrograms(quantityPrograms+1);
//
//    byte value = map(
//      analogRead(POTENTIOMETER_PIN),
//      getMinPotentiometer(),
//      getMaxPotentiometer(),
//      0,
//      MAX_POTENTIOMETER_VALUE
//      );
//
//      setProgram(quantityPrograms-1,value);
//    
//  }
//
//  setProgram(0,0);
//  setProgram(quantityPrograms,255);
  
}

void resetPrograms(){
  for(int i = MEM_PROG_INIT ; i < MEM_PROG_INIT+10 ; i++){
    writeToEeprom(i,0);
  }
  
  writeToEeprom(MEM_PROG_INIT+1,255);  
  writeToEeprom(MEM_QTY_PROGRAMS,2);
}
// ================================
// Interface and Actions
// ================================
void switchLightsOff(){
  digitalWrite( RELAY_CH_1,HIGH );
  digitalWrite( RELAY_CH_2,HIGH );
  digitalWrite( RELAY_CH_3,HIGH );
  digitalWrite( RELAY_CH_4,HIGH );
  digitalWrite( RELAY_CH_5,HIGH );
  digitalWrite( RELAY_CH_6,HIGH );
  digitalWrite( RELAY_CH_7,HIGH );
  digitalWrite( RELAY_CH_8,HIGH );
}

void blinkLedQuick(int quantity){
  for(int i = 0 ; i < quantity ; i++){
    digitalWrite(LED_PIN,HIGH);
    delay(50);
    digitalWrite(LED_PIN,LOW);
    delay(50);
  }
  digitalWrite(LED_PIN,LOW);
}

void blinkLed(int quantity){
  for(int i = 0 ; i < quantity ; i++){
    digitalWrite(LED_PIN,HIGH);
    delay(150);
    digitalWrite(LED_PIN,LOW);
    delay(150);
  }
  digitalWrite(LED_PIN,LOW);
}

void blinkLedLong(int quantity){
  for(int i = 0 ; i < quantity ; i++){
    digitalWrite(LED_PIN,HIGH);
    delay(300);
    digitalWrite(LED_PIN,LOW);
    delay(300);
  }
  digitalWrite(LED_PIN,LOW);
}

void buttonPressed(int pressTime){
  if( pressTime > btnDelaySmall[0] and pressTime <= btnDelaySmall[1] ){
    Serial.println(">> BTN > Short");
    buttonPressedSmall();
  }
  if( pressTime > btnDelayMedium[0] and pressTime <= btnDelayMedium[1] ){
    buttonPressedMedium();
    Serial.println(">> BTN > Medium");
  }
  if( pressTime > btnDelayLong[0] and pressTime <= btnDelayLong[1] ){
    buttonPressedLong();
    Serial.println(">> BTN > Long");
  } 
}

void buttonPressedSmall(){
  if( isProgramMode ){
     if( currentState == program_mode_calibration ){
      updateButtonCalibration();
     }else if(currentState == program_mode_invert_potentiometer){
      updateButtonInvertPotentiometer();   
     }else if(currentState == program_mode_programs){
      updateButtonProgram();
     }
  }else{
    blinkLedQuick(20);
    asm volatile ("  jmp 0");
  }
}

void buttonPressedMedium(){
  if( isProgramMode ){

    if( currentState == program_mode_programs ){
      
      Serial.println(">> Program Mode ON : CALIBRATION");
      currentState = program_mode_calibration;

      blinkLedLong(1);
      blinkLed(6);
    }
    else if(currentState == program_mode_calibration){
      Serial.println(">> Program Mode ON : INVERT");
      currentState = program_mode_invert_potentiometer;

      blinkLedLong(1);
      blinkLed(8);
    }
    else if(currentState == program_mode_invert_potentiometer){
      Serial.println(">> Program Mode ON : PROGRAMS");
      currentState = program_mode_programs;

      blinkLedLong(1);
      blinkLed(4);
    }
    
  }else{
    Serial.println(">> Show Programs");
    blinkLed(4);
    showPrograms();  
  }
}

void buttonPressedLong(){
  
  if( !isProgramMode ){
    currentState = program_mode_programs;
    currentProgramBitValue = false;
    currentProgramBit = 0;
    switchLightsOff();
    currentProgramBitsArray[0] = false;
    currentProgramBitsArray[1] = false;
    currentProgramBitsArray[2] = false;
    currentProgramBitsArray[3] = false;
    currentProgramBitsArray[4] = false;
    currentProgramBitsArray[5] = false;
    currentProgramBitsArray[6] = false;
    currentProgramBitsArray[7] = false;
    isProgramMode = true;

    Serial.println(">> Program Mode ON : PROGRAMS");

    resetPrograms();
    blinkLedLong(1);
    blinkLed(4);
  }else{
    currentState = normal;
    isProgramMode = false;

    Serial.println(">> Program Mode OFF : NORMAL");

    blinkLedLong(1);
    blinkLed(2);
  }
  
}

void writeToEeprom(int addr, byte value){
  EEPROM.write(addr, value);
  Serial.print(">> EEPROM Write Adress : ");Serial.print(addr);Serial.print(" Value : ");Serial.println(value);

}

void writeToEeprom2Bytes(int addr, int value){
  byte hi,lo;
  hi = highByte(value);
  lo = lowByte(value);

  EEPROM.write(addr, hi);
  EEPROM.write(addr+1, lo);
  
  Serial.print(">> EEPROM Write Adress : ");Serial.print(addr);Serial.print(" Value : ");Serial.println(value);
}

byte readFromEeprom(int addr){
  return EEPROM.read(addr);
}

int readFromEeprom2Bytes(int address){
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

byte readPotentiometer(bool isProgramQuantities){
  if( getInvertPotentiometer() == 0 ){
    if( !isProgramQuantities ){
      byte val = map(
          analogRead(POTENTIOMETER_PIN),
          getMinPotentiometer(),
          getMaxPotentiometer()-POTENTIOMETER_DEBUMP,
          0,
          MAX_POTENTIOMETER_VALUE
      );
      if( val > MAX_POTENTIOMETER_VALUE ){
        val = MAX_POTENTIOMETER_VALUE;
      }
      return val;
    }
  
    byte val = map(
        analogRead(POTENTIOMETER_PIN),
        getMinPotentiometer(),
        getMaxPotentiometer()-POTENTIOMETER_DEBUMP,
        0,
        getQuantityPrograms()
    );
    if( val > getQuantityPrograms()-1 ){
      val = getQuantityPrograms()-1;
    }
  
    return val;  
  }else{
    if( !isProgramQuantities ){
      byte val = map(
          analogRead(POTENTIOMETER_PIN),
          getMinPotentiometer(),
          getMaxPotentiometer()-POTENTIOMETER_DEBUMP,
          MAX_POTENTIOMETER_VALUE,
          0
      );  
      if( val > MAX_POTENTIOMETER_VALUE ){
        val = MAX_POTENTIOMETER_VALUE;
      }
      return val;
    }
  
    byte val = map(
        analogRead(POTENTIOMETER_PIN),
        getMinPotentiometer(),
        getMaxPotentiometer()-POTENTIOMETER_DEBUMP,
        getQuantityPrograms(),
        0
    );
    if( val > getQuantityPrograms()-1 ){
      val = getQuantityPrograms()-1;
    }
  
    return val;
  }
  
  
}

void writeToSolidStateRelayModule(byte value){
  Serial.print(">> Write TO SSR : ");Serial.println(value);

  byte notValue = ~value;
  
  bool bit1 = bitRead(notValue,0);
  bool bit2 = bitRead(notValue,1);
  bool bit3 = bitRead(notValue,2);
  bool bit4 = bitRead(notValue,3);
  bool bit5 = bitRead(notValue,4);
  bool bit6 = bitRead(notValue,5);
  bool bit7 = bitRead(notValue,6);
  bool bit8 = bitRead(notValue,7);

  digitalWrite(RELAY_CH_1,bit1 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_2,bit2 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_3,bit3 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_4,bit4 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_5,bit5 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_6,bit6 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_7,bit7 ? HIGH : LOW); 
  digitalWrite(RELAY_CH_8,bit8 ? HIGH : LOW);  

  delay(500);
}

void writeToSolidStateRelayModulePort(byte portIndex, bool value){

  // Serial.print(">> Write TO SSR ");Serial.print(" Port :");Serial.print(portIndex);Serial.print(" Value : ");Serial.println(value);
  bool notValue = ~value;

  switch( portIndex ){
    case 0:
      digitalWrite(RELAY_CH_1,value ? HIGH : LOW); 
      break;
    case 1:
      digitalWrite(RELAY_CH_2,value ? HIGH : LOW); 
      break;
    case 2:
      digitalWrite(RELAY_CH_3,value ? HIGH : LOW); 
      break;
    case 3:
      digitalWrite(RELAY_CH_4,value ? HIGH : LOW); 
      break;
    case 4:
      digitalWrite(RELAY_CH_5,value ? HIGH : LOW); 
      break;
    case 5:
      digitalWrite(RELAY_CH_6,value ? HIGH : LOW); 
      break;
    case 6:
      digitalWrite(RELAY_CH_7,value ? HIGH : LOW); 
      break;
    case 7:
      digitalWrite(RELAY_CH_8,value ? HIGH : LOW); 
      break;
  }

  delay(10);
  
}

void showPrograms(){
  for( int i = 0 ; i < getQuantityPrograms() ; i++ ){
     writeToSolidStateRelayModule(getProgram(i));
     delay(2000);
  }
  switchLightsOff();
}
// ================================
// Gets and Sets
// ================================

int getMinPotentiometer(){ return readFromEeprom2Bytes(MEM_POT_CAL_MIN); }

void setMinPotentiometer(int value){ writeToEeprom2Bytes(MEM_POT_CAL_MIN,value); }

int getMaxPotentiometer(){ return readFromEeprom2Bytes(MEM_POT_CAL_MAX); }

void setMaxPotentiometer(int value){ writeToEeprom2Bytes(MEM_POT_CAL_MAX,value); }

byte getQuantityPrograms(){ return readFromEeprom(MEM_QTY_PROGRAMS); }

void setQuantityPrograms(byte value){ writeToEeprom(MEM_QTY_PROGRAMS,value); }

byte getProgram(byte programNumber){
  return readFromEeprom(MEM_PROG_INIT+programNumber);
}

void setProgram(byte programNumber, byte value){
  writeToEeprom(MEM_PROG_INIT+programNumber,value);
}

byte getInvertPotentiometer(){ return readFromEeprom(MEM_INV_POT); }

void setInvertPotentiometer(byte value){ writeToEeprom(MEM_INV_POT,value); }
