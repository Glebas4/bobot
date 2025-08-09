#include <Arduino.h>

const bool FORWARD = true;
const bool CW = true;
const bool BACKWARD = false;
const bool CCW = false;
const float kP = 1;


class MOTOR
{
  public:
    volatile long count;
    volatile float rounds, total;
    uint8_t speed, enc_pin, _pin1, _pin2;; 
    bool dir, set_speed;  

    MOTOR(uint8_t in_1, uint8_t in_2, uint8_t encoder_pin)
    {
      _pin1 = in_1;
      _pin2 = in_2;
      enc_pin = encoder_pin;
    }
      
    void stop()
    {
      //this->on(!this->dir, 255);
      digitalWrite(_pin1, 1);
      digitalWrite(_pin2, 1);
    }

    void off()
    {
      digitalWrite(_pin1, 0);
      digitalWrite(_pin2, 0);
    }

    void on()
    {
      if(this->dir)
        {
          analogWrite(_pin1, this->speed);
          digitalWrite(_pin2, 0);
        }
      else
        {
          analogWrite(_pin2, this->speed);
          digitalWrite(_pin1, 0);
        }
    }

    void reverse()
    {
      uint8_t n = _pin1;
      _pin1 =_pin2;
      _pin2 = n;
    }

};


class SONAR 
{
  public:
    uint8_t trigger, echo;

    SONAR(uint8_t trig_pin, uint8_t echo_pin)
    {
      trigger = trig_pin;
      echo = echo_pin;
    }

    int read() 
    {
      digitalWrite(trigger, 0);
      delayMicroseconds(2);
      digitalWrite(trigger, 1);

      delayMicroseconds(10);
      digitalWrite(trigger, 0);

      int duration = pulseIn(echo, 1);
      int dist = duration / 58.2;

      Serial.println(dist);
      return dist;
}

};


MOTOR motor_R(11, 10, 3);
MOTOR motor_L(5, 6, 2);

MOTOR *R = &motor_R;
MOTOR *L = &motor_L;


SONAR sonar(7, 8);

void stab(MOTOR* m, unsigned int imp)
{ 
  if(m->set_speed)
    {
      //unsigned int s_Time, c_Time; 
      //c_Time = millis(); 
      //if(c_Time - s_Time >= 1)
        //{
          int pulse, P, D, I, last_P;

          pulse = pulseIn(m->enc_pin, HIGH);

          P = (pulse-imp)*kP; //разница необходимых и действительных оборотов
          //d = (p-last_p) / 0.001;
          //last_P = P;
          //I = I + P*0.01;
    
          //int sum = I + P;

          m->speed = constrain(P, 0, 255);
          m->on();

          //unsigned int s_Time = millis();
        //}
    }
}




void enc(MOTOR* m)
{
  m->count++;
  m->rounds = m->count/273.0;

  if(m->total)
    {
        if(m->rounds >= m->total) 
          {
            m->stop(); 
            m->set_speed = false; 
            m->speed = 0;
            m->count = 0;
            m->rounds = 0;
            m->total = 0; 
          }
    }

}


void turn(bool dir, int degrees)
{
  float total = (degrees * 0.129)/40.192;

  motor_R.total = motor_L.total = total;
  motor_L.speed = motor_R.speed = 90;

  if(dir) 
    {
      motor_L.dir = FORWARD;
      motor_R.dir = BACKWARD;
    }
  else 
    { 
      motor_R.dir = FORWARD;
      motor_L.dir = BACKWARD;
    }

  motor_L.on();
  //motor_R.on(motor_R.dir, motor_R.speed);
}


void enc_R()
{
  enc(R);
}

void enc_L()
{
  enc(L);
}


void setup() 
{

  Serial.begin(250000);
  
  attachInterrupt(digitalPinToInterrupt(3), enc_R, RISING);
  attachInterrupt(digitalPinToInterrupt(2), enc_L, RISING);

  motor_L.total = 0;
  motor_R.total = 0;

  //motor_L.speed=100;
  //motor_R.speed=100;
  
  //motor_L.dir = FORWARD;
  //motor_R.dir = FORWARD;
  
  //motor_R.set_speed = true;
  //motor_L.set_speed = true;

  //delay(3000);
  
  //motor_R.on();
  //motor_L.on();

  //motor_L.on(motor_L.dir, 255);

}


void loop() 
{ 
  //int dist = sonar.read();
  //Serial.println(dist);

  if (Serial.available()>1) 
      {
        char data[3]; 
        Serial.readBytes(data, 3);
      
        uint8_t key = data[0];
        uint8_t cmd = data[1];
        uint8_t val = data[2];

        switch(key)
              {
                case 0: //Right motor manual control
                      switch(cmd)
                      {
                        case 0: motor_L.dir = BACKWARD; motor_L.speed = val; motor_L.on(); break;
                        case 1: motor_L.dir = FORWARD; motor_L.speed = val; motor_L.on(); break;
                        case 2: motor_L.stop(); break;       
                        default: break;
                      }

                      break;

              
                case 1: //Left motor manual control
                      switch(cmd)
                      {
                        case 0: motor_R.dir = BACKWARD; motor_R.speed = val; motor_R.on(); break;
                        case 1: motor_R.dir = FORWARD; motor_R.speed = val; motor_R.on(); break;
                        case 2: motor_R.stop(); break;
                        default: break;
                      }

                      break;

                case 2:
                      motor_L.dir = motor_R.dir = FORWARD;
                      motor_L.speed = motor_R.speed = val;
                      motor_R.on(); 
                      motor_L.on();
                      break;

                case 3:
                      motor_R.dir = motor_L.dir = BACKWARD;
                      motor_R.speed = motor_L.speed = val;
                      motor_R.on(); 
                      motor_L.on();
                      break;

                case 4:
                      switch(cmd)
                      {
                        case 0: motor_R.total = val; break;
                        case 1: motor_L.total = val; break;
                        case 2: motor_R.total = motor_R.total = val; break;
                      }

                case 5:
                      turn(cmd, val); break;
                      
                case 6: //Stop motors
                      motor_L.stop(); 
                      motor_R.stop(); 
                      break;

                default: break;         
              }
      }  

  //stab(R, 1500);
  //stab(L, 1500);

}
