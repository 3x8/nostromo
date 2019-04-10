#include "main.h"

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
COMP_HandleTypeDef hcomp1;
TIM_HandleTypeDef htim1, htim2, htim3, htim15;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

uint16_t step = 1;
uint16_t dir_reversed = 0;
// 1 = quad mode, 2 = crawler <-> thruster mode,  3 = rc car mode,  4 = car mode with auto reverse after stop
uint16_t vehicle_mode = 1;
uint16_t bi_direction = 0;

// for complementary pwm , 0 for diode freewheeling
uint16_t slow_decay = 1;
// apply full motor brake on stop
uint16_t brake = 1;
uint16_t start_power = 150;
uint16_t prop_brake, prop_brake_active;
uint16_t prop_brake_strength = 300;

int adjusted_input;

int dshotcommand = 0;
uint8_t calcCRC;
uint8_t checkCRC;
//uint16_t error = 0;

uint16_t sine_array[20] = {80, 80, 90, 90, 95, 95,95, 100, 100,100, 100, 100, 100, 95, 95, 95, 90, 90, 80, 80};

uint16_t tempbrake = 0;

// increase divisor to decrease advance
uint16_t advancedivisor = 8;
//char advancedivisorup = 3;
//char advancedivisordown = 3;

int thiszctime = 0;
int lastzctime = 0;
int sensorless = 0;
int commutation_interval = 0;
// set proportianal to commutation time. with advance divisor
int advance = 0;
int blanktime;
int waitTime = 0;
char filter_level = 1;
char compit = 0;
int filter_delay = 2;

//debug
int control_loop_count;

int zctimeout = 0;
// depends on speed of main loop
int zc_timeout_threshold = 2000;

int signaltimeout = 0;
int signal_timeout_threshold = 10000;

int tim2_start_arr = 9000;

int duty_cycle = 100;

//ToDo enum
int pwm = 1;
int floating = 2;
int lowside = 3;

int bemf_counts;

int forward = 1;
int rising = 1;
int running = 0;
int started = 0;
char armed = 0;
int armedcount = 0;

int input_buffer_size = 64;
int smallestnumber = 20000;
uint32_t dma_buffer[64];
int propulse[4] = {0,0,0,0};
int dpulse[16] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int input = 0;
int newinput =0;

int voltageraw = 0;
int currentraw = 0;
uint32_t ADC1ConvertedValues[2] = {0,0};
int timestamp;

char dshot = 0;
char proshot = 0;
char multishot = 0;
char oneshot42 = 0;
char oneshot125 = 0;
char servoPwm = 0;

char inputSet = 0;


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// phaseB qfnf051 , phase A qfp32
#ifdef MP6531
void phaseA(int newPhase)
#endif
#ifdef FD6288
void phaseB(int newPhase)
#endif
{
  if (newPhase == pwm) {
    if(!slow_decay  || prop_brake_active) {
      LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
    } else {
      LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
  }

  if (newPhase == floating) {
    LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
  }

  if (newPhase == lowside) {
    LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
  }

}

// phase c qfn , phase b qfp
#ifdef MP6531
void phaseB(int newPhase)
#endif
#ifdef FD6288
void phaseC(int newPhase)
#endif
{
  if (newPhase == pwm) {
    if (!slow_decay || prop_brake_active) {
      LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
    } else {
      LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
  }

  if (newPhase == floating) {
    LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
  }

  if (newPhase == lowside) {
    LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
  }
}

// phaseA qfn , phase C qfp
#ifdef MP6531
void phaseC(int newPhase)
#endif
#ifdef FD6288
void phaseA(int newPhase)
#endif
{
  if (newPhase == pwm) {
    if (!slow_decay || prop_brake_active) {
      LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
    } else {
      LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
  }

  if (newPhase == floating) {
    LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
  }

  if (newPhase == lowside) {
    LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
  }

}

void comStep(int newStep) {
	//A-B
  if (newStep == 1) {
    phaseA(pwm);
    phaseB(lowside);
    phaseC(floating);
  }
  // C-B
  if (newStep == 2) {
    phaseA(floating);
    phaseB(lowside);
    phaseC(pwm);
  }
  // C-A
  if (newStep == 3) {
    phaseA(lowside);
    phaseB(floating);
    phaseC(pwm);
  }
  // B-A
  if (newStep == 4) {
    phaseA(lowside);
    phaseB(pwm);
    phaseC(floating);
  }
  // B-C
  if (newStep == 5) {
    phaseA(floating);
    phaseB(pwm);
    phaseC(lowside);
  }
  // A-C
  if (newStep == 6) {
    phaseA(pwm);
    phaseB(floating);
    phaseC(lowside);
  }
}

void allOff() {
  phaseA(floating);
  phaseB(floating);
  phaseC(floating);
}

void fullBrake() {
  phaseA(lowside);
  phaseB(lowside);
  phaseC(lowside);
}

// duty cycle controls braking strength
// will turn off lower fets so only high side is active
void proBrake() {
  phaseA(pwm);
  phaseB(pwm);
  phaseC(pwm);
}


void changeCompInput() {
  // c floating
  if (step == 1 || step == 4) {
    hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
  }
  // a floating
  if (step == 2 || step == 5) {
#ifdef MP6531
    // if f051k6  step 2 , 5 is dac 1 ( swap comp input)
    hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
#endif
#ifdef FD6288
    hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
#endif
  }
  // b floating
  if (step == 3 || step == 6) {
#ifdef MP6531
    hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
#endif
#ifdef FD6288
    hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
#endif
  }
  if (rising) {
    // polarity of comp output reversed
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  }else{
    // falling bemf
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  }

  while (HAL_COMP_Init(&hcomp1) != HAL_OK);
}


void commutate() {
  if (forward == 1) {
    step++;
    if (step > 6) {
      step = 1;
    }
    if (step == 1 || step == 3 || step == 5) {
      rising = 1;                                // is back emf rising or falling
    }
    if (step == 2 || step == 4 || step == 6) {
      rising = 0;
    }
  }
  if (forward == 0) {
    step--;
    if (step < 1) {
      step = 6;
    }
    if (step == 1 || step == 3 || step == 5) {
      rising = 0;
    }
    if (step == 2 || step == 4 || step == 6) {
      rising = 1;
    }
  }

  if (input > 47) {
    comStep(step);
  }
  changeCompInput();
// TIM2->CNT = 0;
// TIM2->ARR = commutation_interval;
}


// for forced commutation -- open loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
  }
}

void startMotor() {
  uint16_t decaystate = slow_decay;
  sensorless = 0;
  if (running == 0) {
    HAL_COMP_Stop_IT(&hcomp1);
    slow_decay = 1;

    commutate();
    commutation_interval = tim2_start_arr- 3000;
    TIM3->CNT = 0;
    running = 1;
    while (HAL_COMP_Start_IT(&hcomp1) != HAL_OK);
  }

  slow_decay = decaystate;    // return to normal
  sensorless = 1;
  bemf_counts = 0;

}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  timestamp = TIM3->CNT;
  LED_ON(LED0);

  if (compit > 200) {
    HAL_COMP_Stop_IT(&hcomp1);
    //error = 1;
    return;
  }
  compit +=1;
  while (TIM3->CNT - timestamp < filter_delay);

  if (rising) {
    // advancedivisor = advancedivisorup;
    for (int i = 0; i < filter_level; i++) {
      if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH) {
        return;
      }
    }

  } else {
    // advancedivisor = advancedivisordown;
    for (int i = 0; i < filter_level; i++) {
      if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW) {
        return;
      }
    }

  }
  thiszctime = timestamp;
  TIM3->CNT = 0;
  HAL_COMP_Stop_IT(&hcomp1);

  zctimeout = 0;

  // TEST!   divide by two when tracking up down time independant
  commutation_interval = (commutation_interval + thiszctime) / 2;

  advance = commutation_interval / advancedivisor;
  waitTime = commutation_interval /2    - advance;
  blanktime = commutation_interval / 4;

  if(tempbrake) {
    HAL_COMP_Stop_IT(&hcomp1);
    return;
  }
  if (sensorless) {
    while (TIM3->CNT  < waitTime) {
    }

    compit = 0;
    commutate();
    while (TIM3->CNT  < waitTime + blanktime) {
    }
  }

  lastzctime = thiszctime;
  bemf_counts++;

  while (HAL_COMP_Start_IT(&hcomp1) != HAL_OK);
}

void playStartupTune(){
  TIM1->PSC = 75;
  TIM1->CCR1 = 5;
  TIM1->CCR2 = 5;
  TIM1->CCR3 = 5;
  comStep(2);
  HAL_Delay(100);
  TIM1->PSC = 50;
  HAL_Delay(100);
  TIM1->PSC = 25;
  HAL_Delay(100);
  allOff();
  TIM1->PSC = 0;
}

void playInputTune(){
  TIM1->PSC = 100;
  TIM1->CCR1 = 5;
  TIM1->CCR2 = 5;
  TIM1->CCR3 = 5;
  comStep(2);
  HAL_Delay(100);
  TIM1->PSC = 50;
  HAL_Delay(100);
  allOff();
  TIM1->PSC = 0;
}

void getADCs(){
  voltageraw = ADC1ConvertedValues[0];
  currentraw = ADC1ConvertedValues[1];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  getADCs();
}

void detectInput(){
  smallestnumber = 20000;
  dshot = 0;
  proshot = 0;
  multishot = 0;
  oneshot42 = 0;
  oneshot125 = 0;
  servoPwm = 0;
  int lastnumber = dma_buffer[0];
  for ( int j = 1; j < input_buffer_size; j++) {

    if((dma_buffer[j] - lastnumber) < smallestnumber) { // blank space
      smallestnumber = dma_buffer[j] - lastnumber;
    }
    lastnumber = dma_buffer[j];
  }

  if ((smallestnumber > 3)&&(smallestnumber < 22)) {
    dshot = 1;
  }

  if ((smallestnumber > 40 )&&(smallestnumber < 80)) {
    proshot = 1;
    TIM15->PSC=1;
    TIM15->CNT = 0xffff;

  }
  if ((smallestnumber > 100 )&&(smallestnumber < 400)) {
    multishot = 1;
  }
//	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
//		oneshot42 = 1;
//	}
//	if ((smallestnumber > 3000 )&&(smallestnumber < 7000)){
//		oneshot125 = 1;
//	}
  if (smallestnumber > 2000) {
    servoPwm = 1;
    TIM15->PSC = 47;
    TIM15->CNT = 0xffff;
  }

  if (smallestnumber == 0) {
    inputSet = 0;
  }else{

    inputSet = 1;

    HAL_Delay(50);
    // playInputTune();
  }
  HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64);
}

void computeProshotDMA(){
  //debug
  LED_ON(LED1);

  int lastnumber = dma_buffer[0];
  for ( int j = 1; j < 9; j++) {
    if(((dma_buffer[j] - lastnumber) > 1500) && ((dma_buffer[j] - lastnumber) < 50000)) { // blank space
      if ((dma_buffer[j+7] - dma_buffer[j])<10000) {
        //			for ( int i = 0; i < 8; i+= 2){
        //
        //			propulse[i>>1] =map((dma_buffer[j+i+1] - dma_buffer[j+i]),48, 141, 0, 15);
        //			}

        //		for ( int i = 0; i < 8; i+= 2){
        //			 propulse[i>>1] = ((dma_buffer[j+i+1] - dma_buffer[j+i]) - 46)*11>>6;
        //		}
        for (int i = 0; i < 4; i++) {
          propulse[i] = (((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2])) - 23)/3;
        }

        calcCRC = ((propulse[0]^propulse[1]^propulse[2])<<3
                   |(propulse[0]^propulse[1]^propulse[2])<<2
                   |(propulse[0]^propulse[1]^propulse[2])<<1
                   |(propulse[0]^propulse[1]^propulse[2]));
        checkCRC = (propulse[3]<<3 | propulse[3]<<2 | propulse[3]<<1 | propulse[3]);
      }

      if (calcCRC == checkCRC) {
        //debug
        LED_ON(LED2);
        int tocheck = ((propulse[0]<<7 | propulse[1]<<3 | propulse[2]>>1));
        if (tocheck > 2047 || tocheck < 0) {
          break;
        }else{
          if(tocheck > 47) {
            newinput = tocheck;
            dshotcommand = 0;
          }
          if ((tocheck <= 47)&& (tocheck > 0)) {
            newinput = 0;
            dshotcommand = tocheck;  //  todo
          }
          if (tocheck == 0) {
            newinput = 0;
            dshotcommand = 0;
          }
        }
      }
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeMSInput(){
  int lastnumber = dma_buffer[0];

  for ( int j = 1; j < 2; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0)) {
      newinput = map((dma_buffer[j] - lastnumber),243,1200, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeOS125Input(){
  int lastnumber = dma_buffer[0];

  for (int j = 1; j < 2; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) < 12300) && ((dma_buffer[j] - lastnumber) > 0)) {
      newinput = map((dma_buffer[j] - lastnumber),6500,12000, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeOS42Input(){
  int lastnumber = dma_buffer[0];
  for ( int j = 1; j < 2; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) < 4500) && ((dma_buffer[j] - lastnumber) > 0)) {
      newinput = map((dma_buffer[j] - lastnumber),2020, 4032, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeServoInput(){
  int lastnumber = dma_buffer[0];
  for ( int j = 1; j < 3; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) >1000 ) && ((dma_buffer[j] - lastnumber) < 2010)) {
      newinput = map((dma_buffer[j] - lastnumber), 1090, 2000, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}


void computeDshotDMA(){
  int lastnumber = dma_buffer[0];

  for ( int j = 1; j < input_buffer_size; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) > 50) && ((dma_buffer[j] - lastnumber) < 65000)) {
      for (int i = 0; i < 16; i++) {
        dpulse[i] = ((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2]) / 13) - 1;
      }

      uint8_t calcCRC = ( (dpulse[0]^dpulse[4]^dpulse[8]) << 3
                         |(dpulse[1]^dpulse[5]^dpulse[9]) << 2
                         |(dpulse[2]^dpulse[6]^dpulse[10]) << 1
                         |(dpulse[3]^dpulse[7]^dpulse[11])
                         );
      uint8_t checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);

      int tocheck = (
            dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | dpulse[3] << 7
          | dpulse[4] << 6 | dpulse[5] << 5 | dpulse[6] << 4 | dpulse[7] << 3
          | dpulse[8] << 2 | dpulse[9] << 1 | dpulse[10]);

      if(calcCRC == checkCRC) {
        if (tocheck > 47) {
          newinput = tocheck;
          dshotcommand = 0;
        }
      }
      if ((tocheck <= 47) && (tocheck > 0)) {
        newinput = 0;
        dshotcommand = tocheck;    // todo
      }
      if (tocheck == 0) {
        newinput = 0;
        dshotcommand = 0;
      }

      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void transferComplete(){
// TIM15->CNT = 1;
// compit = 0;
  signaltimeout = 0;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);

  if (inputSet == 1) {
    if (dshot == 1) {
      computeDshotDMA();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64);
    }

    if (proshot == 1) {
      computeProshotDMA();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 16);
    }

    if  (servoPwm == 1) {
      computeServoInput();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
    }

    if  (multishot) {
      computeMSInput();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);

    }

    if  (oneshot125) {
      computeOS125Input();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);

    }

    if  (oneshot42) {
      computeOS42Input();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
    }
  }
}


void changeDutyCycleWithSin() {
  if (!rising) {
    // last ten elements in sin array
    duty_cycle = (duty_cycle * sine_array[((TIM2->CNT*10)/TIM2->ARR)+9])/100;
  }else{
    // first ten elements in sin array
    duty_cycle = (duty_cycle * sine_array[(TIM2->CNT*10)/TIM2->ARR])/100;
  }

  TIM1->CCR1 = duty_cycle;
  TIM1->CCR2 = duty_cycle;
  TIM1->CCR3 = duty_cycle;
}

void zc_found_routine(){
  zctimeout = 0;

  thiszctime = TIM3->CNT;

  if (thiszctime < lastzctime) {
    lastzctime = lastzctime - 65535;
  }

  if (thiszctime > lastzctime) {
//		if (((thiszctime - lastzctime) > (commutation_interval * 2)) || ((thiszctime - lastzctime < commutation_interval/2))){
//		//	commutation_interval = (commutation_interval * 3 + (thiszctime - lastzctime))/4;
//			commutation_interval = (commutation_interval + (thiszctime - lastzctime))/2;
//		}else{
    commutation_interval = (thiszctime - lastzctime);       // TEST!   divide by two when tracking up down time independant
    //	}
    advance = commutation_interval / advancedivisor;
    waitTime = commutation_interval /2 - advance;
  }
  if (sensorless) {
    while (TIM3->CNT - thiszctime < waitTime) {
    }
    commutate();
    while (TIM3->CNT - thiszctime < waitTime + blanktime) {
    }
  }

  lastzctime = thiszctime;
}


int main(void) {
  HAL_Init();
  systemClockConfig();

  configValidateOrReset();
  configRead();

  ledInit();
  systemDmaInit();
  systemAdcInit();
  systemComparator1Init();
  systemTimer1Init();
  systemTimer2Init();
  systemTimer3Init();
  systemTimer15Init();
  watchdogInit(2000);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);

  // HAL_Delay(500);
  playStartupTune();

  while (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK);
  while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64) != HAL_OK);
	//while (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK);
  while (HAL_COMP_Start_IT(&hcomp1) != HAL_OK);



  //ToDo
  if (vehicle_mode == 1) {                    // quad single direction
    vehicle_mode = escConfig()->vehicle_mode;
    dir_reversed = escConfig()->dir_reversed;
    bi_direction = escConfig()->bi_direction;
  }
  if (vehicle_mode == 2) {                   // crawler or thruster
    bi_direction = 1;
    slow_decay = 1;                       // for complementary pwm , 0 for diode freewheeling
    brake = 1;                           // apply full motor brake on stop
    start_power = 150;

  }
  if (vehicle_mode == 3) {                 // rc car 50 percent brake on reverse.
    bi_direction = 1;
    slow_decay = 0;                       // for complementary pwm , 0 for diode freewheeling
    brake = 0;                           // apply full motor brake on stop
    // start_power = 150;
    prop_brake = 1;
    prop_brake_strength = 900;
  }
  if (vehicle_mode == 4) {                 // rc car 50 percent brake on reverse.
    bi_direction = 1;
    slow_decay = 0;                         // for complementary pwm , 0 for diode freewheeling
    brake = 0;                             // apply full motor brake on stop
    // start_power = 150;
    prop_brake = 1;
    prop_brake_strength = 800;
  }

  if(bi_direction) {
    newinput = 1001;
    //	start_power = 175;
  }

  // set duty cycle to 50 out of 768 to start.
  TIM1->CCR1 = 1;
  TIM1->CCR2 = 1;
  TIM1->CCR3 = 1;
  TIM1->CCR4 = 800;

  // main loop
  while (true) {
    watchdogFeed();

    LED_OFF(LED0);
    LED_OFF(LED1);
    LED_OFF(LED2);

    compit = 0;

    control_loop_count++;
    if (control_loop_count > 1) {
      control_loop_count = 0;

//  1-5: beep (1= low freq. 5 = high freq.)
//  6: ESC info request (FW Version and SN sent over the tlm wire)
//  7: rotate in one direction
//  8: rotate in the other direction
//  9: 3d mode off
//  10: 3d mode on
//  11: ESC settings request (saved settings over the TLM wire) (planed but not there yet)
//  12: save Settings

      if (dshotcommand > 0) {
        if (dshotcommand == 1) {
          playStartupTune();
        }
        if (dshotcommand == 2) {
          playInputTune();
        }
        if (dshotcommand == 21) {
          forward =  dir_reversed;
        }
        if (dshotcommand == 20) {         // forward = 1 if dir_reversed = 0
          forward = 1 - dir_reversed;
        }
        if (dshotcommand == 7) {
          dir_reversed = 0;

        }
        if (dshotcommand == 8) {
          dir_reversed = 1;
        }
        if (dshotcommand == 9) {
          bi_direction = 0;
          armed = 0;

        }
        if (dshotcommand == 10) {
          bi_direction = 1;
          armed = 0;
        }
        if (dshotcommand == 12) {
          escConfig()->vehicle_mode = vehicle_mode;
          escConfig()->dir_reversed = dir_reversed;
          escConfig()->bi_direction = bi_direction;
          configWrite();
          // reset esc, iwdg timeout
          while(true);
        }
      }

      if (bi_direction == 1 && (proshot == 0 && dshot == 0)) {
        //char oldbrake = brake;
        if ( newinput > 1100 ) {
          if (forward == dir_reversed) {
            adjusted_input = 0;
            prop_brake_active = 1;
            forward = 1 - dir_reversed;
            //	HAL_Delay(1);
          }

          if (prop_brake_active == 0) {
            adjusted_input = (newinput - 1050)*3;
            //	tempbrake = 0;
            //	}
          }
        }

        if (newinput < 800) {
          if (forward == (1 - dir_reversed)) {
            prop_brake_active = 1;
            adjusted_input = 0;
            forward = dir_reversed;
            //	HAL_Delay(1);
          }

          if (prop_brake_active == 0) {
            adjusted_input = (800 - newinput) * 3;
          }
          //	tempbrake = 0;
        }

        if (zctimeout >= zc_timeout_threshold) {
          //	adjusted_input = 0;
          if (vehicle_mode != 3) {                // car mode requires throttle return to center before direction change
            prop_brake_active = 0;
          }
          bemf_counts = 0;
        }

        if (newinput > 800 && newinput < 1100) {
          adjusted_input = 0;
          prop_brake_active = 0;
        }

      } else if((proshot || dshot ) && bi_direction) {
        if ( newinput > 1097 ) {

          if (forward == dir_reversed) {
            forward = 1 - dir_reversed;
            bemf_counts =0;
          }
          adjusted_input = (newinput - 1100) * 2 + 100;
        } if ( newinput <= 1047 &&  newinput > 0) {
          if(forward == (1 - dir_reversed)) {
            bemf_counts =0;
            forward = dir_reversed;
          }
          adjusted_input = (newinput - 90) * 2;
        }
        if ((newinput > 1047 && newinput < 1098 ) || newinput <= 120) {
          adjusted_input = 0;
        }
      } else {
        adjusted_input = newinput;
      }

      if (adjusted_input > 2000) {
        adjusted_input = 2000;
      }

      if (adjusted_input - input > 25) {
        input = input + 5;
      } else {
        input = adjusted_input;
      }

      if (adjusted_input <= input) {
        input = adjusted_input;
      }
    }

    advancedivisor = map((commutation_interval),100,5000, 2, 20);
    if (inputSet == 0) {
      HAL_Delay(10);
      detectInput();
    }

    if (!armed) {
      if ((inputSet == 1)&&(input == 0)) {
        armedcount++;
        HAL_Delay(1);
        if (armedcount > 1000) {
          armed = 1;
          playInputTune();
        }
      }
      if (input > 1) {
        armedcount = 0;
      }
    }

    if ((input > 47)&&(armed == 1)) {
      prop_brake_active = 0;
      started = 1;

      duty_cycle = input / 2 - 10;

      if (bemf_counts < 15) {
        if(duty_cycle < 70) {
          duty_cycle=70;
        }
        if (duty_cycle > 400) {
          duty_cycle=400;
        }
      }

      if (running) {
        if (duty_cycle > 998 ) {                                          // safety!!!
          duty_cycle = 998;
        }
        if (duty_cycle < 44) {
          duty_cycle = 44;
        }

        // set duty cycle to 50 out of 768 to start.
        TIM1->CCR1 = duty_cycle;
        TIM1->CCR2 = duty_cycle;
        TIM1->CCR3 = duty_cycle;
        //	TIM1->CCR4 = duty_cycle;
      }
    }

    signaltimeout++;
    if (signaltimeout > signal_timeout_threshold ) {
      input = 0;
      armed = 0;
      armedcount = 0;
      //error = 1;
      //	  duty_cycle = 0;          //mid point
    }

    if (input <= 47) {
      //	sensorless = 0;
      started = 0;
      if ( !brake && !prop_brake_active) {
        allOff();
      }
      duty_cycle = 0;
      if(brake || tempbrake) {
        fullBrake();
        duty_cycle = 0;
        //HAL_COMP_Stop_IT(&hcomp1);
      }

      if(prop_brake && prop_brake_active) {
        //prop_brake_active = 1;
        duty_cycle = prop_brake_strength;
        proBrake();
      }

      // set duty cycle to 50 out of 768 to start.
      TIM1->CCR1 = duty_cycle;
      TIM1->CCR2 = duty_cycle;
      TIM1->CCR3 = duty_cycle;

      if (commutation_interval > 30000) {
        HAL_COMP_Stop_IT(&hcomp1);
        //prop_brake_active = 0;
      }

    }

    if (bemf_counts < 100 || commutation_interval > 10000) {
      filter_delay = 15;
      filter_level = 10;
    } else {
      filter_level = 3;
      filter_delay = 3;
    }

    if(commutation_interval < 200 && duty_cycle > 500) {
      filter_delay = 1;
      filter_level = 0;
    }

    if (started == 1) {
      if (running == 0) {
        //allOff();
        zctimeout = 0;
        // safety on for input testing
        startMotor();
      }
    }

    if (duty_cycle < 300) {
      zc_timeout_threshold = 4000;
    }else{
      zc_timeout_threshold = 2000;
    }

    zctimeout++;                                            // move to started if
    if (zctimeout > zc_timeout_threshold) {
      //prop_brake_active = 0;
      sensorless = 0;
      HAL_COMP_Stop_IT(&hcomp1);

      running = 0;
      //		commutation_interval = 0;
      zctimeout = zc_timeout_threshold + 1;
      duty_cycle = 0;
    }
  }
}
