/*
  2024-04-03 initiale version ausm bunker
  2024-04-04 klappen servo aus 5s nach letzter poti aenderrung
*/

// Klappensteuerung
#define KLAPPEN_STEL_POTI       A0 // SVP, ADC1_0  1280..4095
#define KLAPPEN_STEL_POTI_MIN 1140
#define KLAPPEN_STEL_POTI_MAX 4095
#define KLAPPEN_SERVO           19 // P19
#define KLAPPEN_SERVO_MIN       70 // 50 // 45 // 160 // 0
#define KLAPPEN_SERVO_MAX        0 //160
#define KLAPPEN_SERVO_TIMEOUT    5 // kommando fuer 5s nach jeder aenderrung des STEL-poti's

#define ZUENDUNG                23 // P23, KLEMME 13
#define ZUENDUNG_IMMER_AN        0

// Heizungssteuerung
#define HEIZUNGS_INTERVAL       60 // 60 // 30 sekunden
#define HEIZUNGS_TRIG           22 // pin P22
#define HEIZUNGS_POTI           A6 // pin P34, ADC1_6
#define HEIZUNGS_POTI_MIN     1140
#define HEIZUNGS_POTI_MAX     4095
#define HEIZUNGS_MAX           100 // 100 -> 100% heizung -> 0V die ganze zeit!
#define HEIZUNGS_MIN             0 // 0 -> 0% heizung -> 3v3 die ganze zeit! -> ventil zieht strom!

#define LED_LINKS               16 // pin
#define LED_RECHTS              17 // pin

#define ZV_BLINK                18 // pin
#define ZV                       5 // pin

#define ZV_AUF_LED_TIME         10 // sekunden
#define ZV_ZU_LED_INTERVAL     400 // millisekunden
#define ZV_ZU_LED_COUNT          3 // pulse

#define TEST_LED1               13 // pin

#define ADC_MIN                  0
#define ADC_MAX               4095

#include <ESP32Servo.h>
#define DEBUG
#ifdef DEBUG
#  define dbg_val(a) do { Serial.print(#a " "); Serial.println(a); } while(0)
#  define dbg_val_ts(a) do { Serial.print(millis()); Serial.print(" "); Serial.print(#a " "); Serial.println(a); } while(0)
#else
#  define dbg_val(a)
#  define dbg_val_ts(a)
#endif

Servo klappen_servo;
bool klappen_servo_on = false;
uint32_t klappen_servo_start_time = 0;
int last_servo_cmd = 0;

bool zuendung_an = false;

int heizungs_duty_cycle = HEIZUNGS_MAX;
uint32_t heizung_interval_start = 0; // millis
void heizung_an() {
  digitalWrite(HEIZUNGS_TRIG, false); // ventilaus, spule ist stromlos
  heizungs_duty_cycle = HEIZUNGS_MAX;
  heizung_interval_start = millis();
}

void heizung_aus() {
  digitalWrite(HEIZUNGS_TRIG, true); // ventil schalten, spule zieht strom!
  heizungs_duty_cycle = HEIZUNGS_MIN;
  heizung_interval_start = millis();
}

void leds(bool links, bool rechts) {
  digitalWrite(LED_LINKS, links);
  digitalWrite(LED_RECHTS, rechts);
}

#define LED_FREQ 5000

void setup() {
  Serial.begin(115200);

  pinMode(TEST_LED1, OUTPUT);
  //ledcSetup(0, LED_FREQ, 0);
  //ledcAttachPin(TEST_LED1, 0);


  pinMode(ZUENDUNG, INPUT);
  pinMode(ZV_BLINK, INPUT);
  pinMode(ZV, INPUT);

  pinMode(KLAPPEN_SERVO, OUTPUT); // attach only on zuendung-on event

  pinMode(LED_LINKS, OUTPUT);
  pinMode(LED_RECHTS, OUTPUT);
  leds(false, false); // beide aus

  pinMode(HEIZUNGS_TRIG, OUTPUT);
  heizung_an();
}

uint32_t elapsed_since_u32(uint32_t last, uint32_t now) {
  if(now >= last)
    return now - last;
  return 0xffffffff - last + now;
}

void standby_led() {
  static uint32_t last_time = 0;
  uint32_t now = micros(); // unsigned long is 32bit here
  uint32_t since_last = elapsed_since_u32(last_time, now);
  static bool toggle = false;
  if(since_last < 1e6)
    return;
  toggle = !toggle;
  last_time = now;
  digitalWrite(TEST_LED1, toggle);
  /*
  // dim-up
  if(since_last > 1e6) {
    last_time = now;
    since_last = 0;
    toggle = !toggle;
  }
  uint8_t duty = (uint8_t)(since_last * 255 / 1e6);
  if(toggle)
    duty = 255 - duty;
  //dbg_val(duty);
  ledcWrite(0, duty);
  */
}

#define map_poti(v, poti_prefix, out_prefix) \
    map( \
      constrain(v, poti_prefix ## _MIN, poti_prefix ## _MAX),  \
      poti_prefix ## _MIN, poti_prefix ## _MAX, \
      out_prefix ## _MIN, out_prefix ## _MAX)
#define map_poti_inv(v, poti_prefix, out_prefix) \
    map( \
      constrain(v, poti_prefix ## _MIN, poti_prefix ## _MAX),  \
      poti_prefix ## _MIN, poti_prefix ## _MAX, \
      out_prefix ## _MAX, out_prefix ## _MIN)

void klappen_servo_aus() {
	if(klappen_servo_on == false)
		return;
	klappen_servo.detach();
	klappen_servo_on = false;
	dbg_val(klappen_servo_on);
}

void klappensteuerung() {
	int stel_poti = analogRead(KLAPPEN_STEL_POTI);
	//dbg_val(stel_poti);
	int servo_cmd = map_poti(stel_poti, KLAPPEN_STEL_POTI, KLAPPEN_SERVO);

	uint32_t now = millis();
	if(servo_cmd != last_servo_cmd) {
		last_servo_cmd = servo_cmd;
		klappen_servo_start_time = now;
	} else {
		// no new command
		if(klappen_servo_on) {
			// check for timeout
			uint32_t since_start = elapsed_since_u32(klappen_servo_start_time, now);
			if (since_start > (1e3 * KLAPPEN_SERVO_TIMEOUT))
				klappen_servo_aus();
		}
		if(!klappen_servo_on)
			return;
	}

	// send new servo command!
	if(klappen_servo_on == false) {
		klappen_servo.attach(KLAPPEN_SERVO);
		klappen_servo_on = true;
		dbg_val(klappen_servo_on);
	}

	klappen_servo.write(servo_cmd);
	//dbg_val(servo_cmd);
}

void heizungssteuerung() {
  int heizungs_poti = analogRead(HEIZUNGS_POTI);
  //dbg_val(heizungs_poti);
  heizungs_duty_cycle = map_poti_inv(heizungs_poti, HEIZUNGS_POTI, HEIZUNGS);
  //dbg_val(heizungs_duty_cycle);
  bool trig;
  uint32_t schalt_zeitpunkt = (uint32_t)((float)heizungs_duty_cycle * HEIZUNGS_INTERVAL * 1e3 / 100);
  uint32_t now = millis(); // unsigned long is 32bit here
  uint32_t since_interval_start = elapsed_since_u32(heizung_interval_start, now);
  if(since_interval_start > HEIZUNGS_INTERVAL * 1000) {
    since_interval_start = 0;
    heizung_interval_start = now;
  }
  if(since_interval_start < schalt_zeitpunkt)
    trig = false; // heizung an, ventil aus, spule ist stromlos
  else
    trig = true; // heizung aus, ventil an, spule ist zieht strom
  digitalWrite(HEIZUNGS_TRIG, trig); 
}

uint32_t led_cmd = 0; // 0 -> nothing, 1 -> stay on for 20s, 2 -> 3 pulses
uint32_t led_cmd_start = 0;
uint32_t led_pulse_cnt = 0;
bool led_pulse_state = false;
void zv_auf() {
  Serial.println("ZV auf");
  led_cmd = 1; // stay on
  led_cmd_start = millis();
}
void zv_zu() {
  Serial.println("ZV zu");
  led_cmd = 2;
  led_cmd_start = millis();
  led_pulse_cnt = ZV_ZU_LED_COUNT;
  led_pulse_state = true; // on
}
void zv_idle() {
  if(led_cmd == 0) {
    leds(false, false); // beide aus
    return;
  }
  uint32_t since_start = elapsed_since_u32(led_cmd_start, millis());
  if(led_cmd == 1) { // 20s an
    leds(true, true); // beide an
    if(since_start > (ZV_AUF_LED_TIME * 1e3))
      led_cmd = 0; // cmd done
  } else if(led_cmd == 2) { // 3 pulsed
    leds(led_pulse_state, led_pulse_state); // beide an
    if(since_start > ZV_ZU_LED_INTERVAL) {
      if(led_pulse_state)
        led_pulse_cnt --;
      led_pulse_state = !led_pulse_state;
      led_cmd_start = millis();
      if(led_pulse_cnt == 0)
        led_cmd = 0; // pulses done
    }
  }
}
bool last_blink = false;
bool last_zv = false; // eine seite vom stellmotor
bool do_msr_zv = false;
uint32_t msr_zv_start = 0;
void zentralverriegellung(int ms) {
  uint32_t start = millis();
  while(true) {
    uint32_t now = millis();
    uint32_t since_start = elapsed_since_u32(start, now);
    if(since_start > ms)
      break;

    // poll pins
    bool blink = digitalRead(ZV_BLINK);
    zuendung_an = ZUENDUNG_IMMER_AN || digitalRead(ZUENDUNG);
    if(blink != last_blink) {
      dbg_val_ts(blink);
      delay(50);
      last_blink = blink;

      if(zuendung_an == false && blink == false && do_msr_zv == false) {
        msr_zv_start = millis();
        do_msr_zv = true;
      }
    }
    bool zv = digitalRead(ZV);
    if(zv != last_zv) {
      dbg_val_ts(zv);
      last_zv = zv;
    }

    if(do_msr_zv && elapsed_since_u32(msr_zv_start, now) > 200) {
      do_msr_zv = false;
      if(zuendung_an == false) {
        if(zv == false)
          zv_auf();
        else
          zv_zu();
      }
    }

    zv_idle();
  }
  //Serial.println("timeout");
}

bool last_zuendung_an = false;
void loop() {  
  zuendung_an = ZUENDUNG_IMMER_AN || digitalRead(ZUENDUNG);
  //dbg_val(zuendung_an);

  if (zuendung_an != last_zuendung_an) {
    dbg_val(zuendung_an);
    last_zuendung_an = zuendung_an;
    if(zuendung_an)
	    klappen_servo_start_time = millis();
    else
	    klappen_servo_aus();
  }

  if(zuendung_an) {
    digitalWrite(TEST_LED1, true);
    //ledcWrite(0, 255); // status led full on

    klappensteuerung();
    heizungssteuerung();

    //delay(100);
  } else {
    standby_led();
    heizung_an();
    zentralverriegellung(50);
  }
}
