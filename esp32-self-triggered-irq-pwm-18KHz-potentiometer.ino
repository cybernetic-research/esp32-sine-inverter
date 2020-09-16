/**
 * @file      esp32-self-triggered-irq-pwm-18KHz-potentiometer.ino
 * @author    cybernetic-research
 * @brief     A simple open loop SPWM generator intended to be used to 
 *            generate ac at 50 or 60Hz. This program outputs SPWM at 18KHz for 
 *            50hz, or 21600Hz for 60Hz. the PWM output is wired back into the
 *            chip as a rising edge interrupt allowing a new PWM value to be
 *            output every edge transition. An ADC is used to read a 
 *            potentiometer allowing the PWM output voltage to be raised or
 *            lowered (because we scale the sine lookup table).
 *            This program is meant to be used on ESP32 boards  
 * @version   0.1
 * @date      2020-09-16
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <stdint.h>

const byte led_gpio = 32;   // the PWM pin the LED is attached to
int32_t brightness = 0;     // how bright the LED is
const int potPin = 27;      // Potentiometer is connected to GPIO 27 (Analog ADC1_CH6) 
int32_t  potValue = 0;      // variable for storing the potentiometer value


uint8_t  pwm=1;
int32_t PWM_FEEDBACK_PIN = 18;

#define MODULATING_FREQ   18000     // <-- 18KHz is 360 x 50 Hz for American 60Hz use number 21600 instead
#define MAX_STEPS         359       //  one pwm update per degree of mains sine

//8-bit representation of a sinewave scaled to 0->255
int32_t sinetable[]=
{
127  ,
129 ,
131 ,
134 ,
136 ,
138 ,
140 ,
143 ,
145 ,
147 ,
149 ,
151 ,
154 ,
156 ,
158 ,
160 ,
162 ,
164 ,
166 ,
169 ,
171 ,
173 ,
175 ,
177 ,
179 ,
181 ,
183 ,
185 ,
187 ,
189 ,
191 ,
193 ,
195 ,
196 ,
198 ,
200 ,
202 ,
204 ,
205 ,
207 ,
209 ,
211 ,
212 ,
214 ,
216 ,
217 ,
219 ,
220 ,
222 ,
223 ,
225 ,
226 ,
227 ,
229 ,
230 ,
231 ,
233 ,
234 ,
235 ,
236 ,
237 ,
239 ,
240 ,
241 ,
242 ,
243 ,
243 ,
244 ,
245 ,
246 ,
247 ,
248 ,
248 ,
249 ,
250 ,
250 ,
251 ,
251 ,
252 ,
252 ,
253 ,
253 ,
253 ,
254 ,
254 ,
254 ,
254 ,
254 ,
254 ,
254 ,
255 ,
254 ,
254 ,
254 ,
254 ,
254 ,
254 ,
254 ,
253 ,
253 ,
253 ,
252 ,
252 ,
251 ,
251 ,
250 ,
250 ,
249 ,
248 ,
248 ,
247 ,
246 ,
245 ,
244 ,
243 ,
243 ,
242 ,
241 ,
240 ,
239 ,
237 ,
236 ,
235 ,
234 ,
233 ,
231 ,
230 ,
229 ,
227 ,
226 ,
225 ,
223 ,
222 ,
220 ,
219 ,
217 ,
216 ,
214 ,
212 ,
211 ,
209 ,
207 ,
205 ,
204 ,
202 ,
200 ,
198 ,
196 ,
195 ,
193 ,
191 ,
189 ,
187 ,
185 ,
183 ,
181 ,
179 ,
177 ,
175 ,
173 ,
171 ,
169 ,
166 ,
164 ,
162 ,
160 ,
158 ,
156 ,
154 ,
151 ,
149 ,
147 ,
145 ,
143 ,
140 ,
138 ,
136 ,
134 ,
131 ,
129 ,
127 ,
125 ,
123 ,
120 ,
118 ,
116 ,
114 ,
111 ,
109 ,
107 ,
105 ,
103 ,
100 ,
98  ,
96  ,
94  ,
92  ,
90  ,
88  ,
85  ,
83  ,
81  ,
79  ,
77  ,
75  ,
73  ,
71  ,
69  ,
67  ,
65  ,
63  ,
61  ,
59  ,
58  ,
56  ,
54  ,
52  ,
50  ,
49  ,
47  ,
45  ,
43  ,
42  ,
40  ,
38  ,
37  ,
35  ,
34  ,
32  ,
31  ,
29  ,
28  ,
27  ,
25  ,
24  ,
23  ,
21  ,
20  ,
19  ,
18  ,
17  ,
15  ,
14  ,
13  ,
12  ,
11  ,
11  ,
10  ,
9 ,
8 ,
7 ,
6 ,
6 ,
5 ,
4 ,
4 ,
3 ,
3 ,
2 ,
2 ,
1 ,
1 ,
1 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
1 ,
1 ,
1 ,
2 ,
2 ,
3 ,
3 ,
4 ,
4 ,
5 ,
6 ,
6 ,
7 ,
8 ,
9 ,
10  ,
11  ,
11  ,
12  ,
13  ,
14  ,
15  ,
17  ,
18  ,
19  ,
20  ,
21  ,
23  ,
24  ,
25  ,
27  ,
28  ,
29  ,
31  ,
32  ,
34  ,
35  ,
37  ,
38  ,
40  ,
42  ,
43  ,
45  ,
47  ,
49  ,
50  ,
52  ,
54  ,
56  ,
58  ,
59  ,
61  ,
63  ,
65  ,
67  ,
69  ,
71  ,
73  ,
75  ,
77  ,
79  ,
81  ,
83  ,
85  ,
88  ,
90  ,
92  ,
94  ,
96  ,
98  ,
100 ,
103 ,
105 ,
107 ,
109 ,
111 ,
114 ,
116 ,
118 ,
120 ,
123 ,
125 ,
};



uint8_t readPot =0;
int32_t ctr=0;

/**
 * @brief IRAM_ATTR
 * An interrupt service routine that runs whenever a rising edge is detected on
 * the feedback I/O pin. We generate a PWM pulse at 18KHz for 50Hz, or 21.6Khz 
 * for a 60Hz pulse. the PWM output pin is fed back into the chip as an 
 * interrupt and we retrigger from it.
 * Every time an interrupt is received we load in the next PWM value from the 
 * sine table. this allows each degree of the 50 or 60Hz waveform to have a 
 * separate pulse associated with it  
 */
void IRAM_ATTR isr() {
  int32_t val = 1;
  val = (sinetable[ctr] * potValue) / 255 ; //scale the sine tablke 
  ctr+=1;
  if(ctr>MAX_STEPS)
  {
    ctr=0;
    readPot = 1;
  }
  if(val==0)
  {
    val = 1;
  }
  ledcWrite(0, val); // set the brightness of the LED
 
}






/**
 * @brief setup
 * the setup routine runs once when you press reset:
 */
void setup() {
  ledcAttachPin(led_gpio, 0); // assign a led pins to a channel

  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, MODULATING_FREQ, 8);                 // 18 kHz PWM, 8-bit resolution
  pinMode(PWM_FEEDBACK_PIN, INPUT_PULLDOWN);
  attachInterrupt(PWM_FEEDBACK_PIN, isr, RISING);  //this is wired to the PWM output 
  ledcWrite(0, pwm);                               //signal to mosfet gate
  Serial.begin(115200);
}





/**
 * @brief loop
 * Shall read ADC value and allow user to modify PWM amplitude by means of
 * potentiometer. We scale the SPWM in fact, allowing a synthesised lower voltage
 */
int32_t t = 0;
void loop() {
  
  if(readPot)
  {
    readPot = 0;
    int p = analogRead(potPin);  //https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
    t+=1;
    if(20==t)
    {
     Serial.printf("ADCM %u\n", potValue);
     t=0;
    }
    potValue = p/16;
    if(potValue<1)
    {
      potValue = 0;
    }
    if(potValue>255)
    {
      potValue = 255;
    }
  }
}
