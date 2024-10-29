#define PIN_PUMP 12
#define PIN_VALVE 11

#define PIN_FRONT_ENABLE 10
#define PIN_FRONT_FORWARD 8
#define PIN_FRONT_BACKWARD 9

#define PIN_REAR_ENABLE 7
#define PIN_REAR_FORWARD 6
#define PIN_REAR_BACKWARD 5

#define BUFF_SIZE 256

char buff[BUFF_SIZE];

static volatile int pump_on = false;
static volatile int valve_open = false;
static volatile double motor_speed = 0;

void setup()
{
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);

  pinMode(PIN_FRONT_ENABLE, OUTPUT);
  pinMode(PIN_FRONT_FORWARD, OUTPUT);
  pinMode(PIN_FRONT_BACKWARD, OUTPUT);

  pinMode(PIN_REAR_ENABLE, OUTPUT);
  pinMode(PIN_REAR_FORWARD, OUTPUT);
  pinMode(PIN_REAR_BACKWARD, OUTPUT);

}

// the loop function runs over and over again forever
void loop()
{
  // delay(1000);                      // wait for a second
  // delay(1000);                      // wait for a second

  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // digitalWrite(PIN_PUMP, HIGH);
  // delay(1000);
  // digitalWrite(PIN_PUMP, LOW);
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000);
  // forward(255);
  // delay(1000);

  // digitalWrite(PIN_PUMP, (int) (!pump_on));
  // digitalWrite(PIN_VALVE, (int) (!valve_open));

  if (pump_on) {
    turn_pump_on();
  } else {
    turn_pump_off();
  }

  if (valve_open) {
    set_valve_open();
  } else {
    set_valve_close();
  }

  serialEvent();
}

void serialEvent()
{
  if (Serial.available()) {
    char command = Serial.peek();
    String str = Serial.readStringUntil('\n');
    Serial.println(command);
    Serial.println(str);

    switch (command) {
      case 'p':
        memset(buff, '\0', sizeof(char));
        str.toCharArray(buff, BUFF_SIZE);
        sscanf(buff, "p %d", &pump_on);
        break;

      case 'v':
        memset(buff, '\0', sizeof(char));
        str.toCharArray(buff, BUFF_SIZE);
        sscanf(buff, "v %d", &valve_open);
        break;


      case 'f':
        memset(buff, '\0', sizeof(char));
        str.toCharArray(buff, BUFF_SIZE);
        sscanf(buff, "f %lf", &motor_speed);
        forward(motor_speed);

      case 'b':
        memset(buff, '\0', sizeof(char));
        str.toCharArray(buff, BUFF_SIZE);
        sscanf(buff, "b %lf", &motor_speed);
        backward(motor_speed);

      default:
        break;
    }
  }
}

void forward(int speed) {
  digitalWrite(PIN_FRONT_FORWARD, HIGH);
  digitalWrite(PIN_FRONT_BACKWARD, LOW);
  analogWrite(PIN_FRONT_ENABLE, speed);

  digitalWrite(PIN_REAR_FORWARD, HIGH);
  digitalWrite(PIN_REAR_BACKWARD, LOW);
  analogWrite(PIN_REAR_ENABLE, speed);
}

void backward(int speed) {
  digitalWrite(PIN_FRONT_FORWARD, LOW);
  digitalWrite(PIN_FRONT_BACKWARD, HIGH);
  analogWrite(PIN_FRONT_ENABLE, speed);

  digitalWrite(PIN_REAR_FORWARD, LOW);
  digitalWrite(PIN_REAR_BACKWARD, HIGH);
  analogWrite(PIN_REAR_ENABLE, speed);
}

void turn_pump_on() {
  digitalWrite(PIN_PUMP, LOW);
}

void turn_pump_off() {
  digitalWrite(PIN_PUMP, HIGH);
}

void set_valve_open() {
  digitalWrite(PIN_VALVE, LOW);
}

void set_valve_close() {
  digitalWrite(PIN_VALVE, HIGH);
}