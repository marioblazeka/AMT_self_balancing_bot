# AMT_self_balancing_bot
održavanje ravnoteže te brza reakcija i odgovor na vanjske sile
za izradu aplikacije upotrijebljen je MIT App Inventor, besplatna mrežna platforma kojiomogućava brzo razvijanje mobilnih aplikacija

![image](https://github.com/marioblazeka/AMT_self_balancing_bot/assets/147752707/b1a6a29a-dadf-495e-946c-c4372cf11dd9)

Arduino programski kôd

#include &lt;SoftwareSerial.h&gt;  // Za Bluetooth komunikaciju
#include &quot;I2Cdev.h&quot; // Komunikacija uređaja
#include &lt;PID_v1.h&gt;  // Biblioteka za PID
#include &quot;MPU6050_6Axis_MotionApps20.h&quot;  // Biblioteka iz jrowberg
#include &lt;Wire.h&gt; //Komunikacija s I2C/TWI uređajevima

//slanje vrijednosti na modul L298N
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 3
MPU6050 mpu;
String naredba;
char skretanje = &#39;P&#39;;  //defaultna vrijednost kretanja
SoftwareSerial Bluetooth(5, 6);  //RX pin, TX pin
boolean BTpovezan = false;
bool isTurning = false;
unsigned long turningStartTime;   //trajanje okretanja
const unsigned long turningDuration = 1200;
const byte BTpin = 12;    // Pin za provjeru stanja Bluetooth veze
bool dmpSpreman = false;  //Digital Motion Processor (DMP), koristi
se za dohvaćanje podataka povezanih s kretanjem, postavi true ako je
success
uint8_t mpuIntStatus;     // sadrži bajt statusa prekida iz MPU-a
uint8_t devStatus;        //status uređaja
uint16_t velicinaPaketa;  // očekivana veličina DMP paketa (zadano
je 42 bajta)
uint16_t fifoCount;       //brojac za bajtove u FIFO
uint8_t fifoBuffer[64];
Quaternion q;         //predstavlja orijentaciju uređaja u
kvaternionskom formatu. Kvaternioni su matematički prikaz rotacija u
trodimenzionalnom prostoru.
VectorFloat gravity;  //predstavlja vektor gravitacije. Sadrži
vrijednosti gravitacijske sile duž X, Y i Z osi.
float ypr[3];         //je niz koji pohranjuje izračunate kutove
skretanja, nagiba i prevrtanja uređaja. Ovi kutovi predstavljaju
rotaciju uređaja oko tri osi. YAW PITCH, ROLL ANGLE

double zeljenaVrijednost = -2.30;  //koristena kao setpoint
//Predstavlja centar stabilnosti koji robot pokušava održati.
//Postavljanjem na -2,30, specificira se željeni kut nagiba (u
stupnjevima) pri kojem se robot treba balansirati.
//PID vrijednosti podešavanje
double Kp = 31.2;  //proporcionalna engl. Proportional Gain
double Kd = 0.98;  //derivacijska engl. Derivative Gain
double Ki = 190;   //integralna engl. Integral Gain
//ulazne i izlazne vrijednosti
double input, output;  //input-predstavlja trenutni kut nagiba
//output-je upravljački signal koji pokreće motore da održe
ravnotežu
//na temelju razlike između input i željeneVrijednosti
PID pid(&amp;input, &amp;output, &amp;zeljenaVrijednost, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false;  // Indikator kada se MPU
Interrupt PIN postavlja u stanje HIGH
void dmpPodaciSpremni() {
  mpuInterrupt = true;
}
void setup() {
  Serial.begin(115200);   //USB baudrate fixirana na 115200 bps
  Bluetooth.begin(9600);  //bluetooth radi na tome baud rate-u
  while (!BTpovezan) {
    if (digitalRead(BTpin) == HIGH) {
      BTpovezan = true;
    }
  }
  initMPU();
  initMotors();
}
void loop() {
  obradaPodataka();
  BluetoothUpravljanje();
  ispisVrijednost();
}

void initMPU() {
  Serial.println(F(&quot;Inicijalizacija I2C uređaja...&quot;));
  Wire.begin();
  mpu.initialize();
  // provjera veze
  Serial.println(F(&quot;Testiranje povezanosti uređaja...&quot;));
  Serial.println(mpu.testConnection() ? F(&quot;MPU6050 povezivanje
uspješno&quot;)
                                      : F(&quot;MPU6050 povezivanje
neuspješno&quot;));
  //Pokreće DMP (Digital Motion Processor) za dobivanje podataka
senzora
  devStatus = mpu.dmpInitialize();
  //Default gyrooffset za mpu6050 modul
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpPodaciSpremni, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpSpreman = true;
    velicinaPaketa = mpu.dmpGetFIFOPacketSize();
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
}
void BluetoothUpravljanje(){
  if (Bluetooth.available()) {
    naredba = Bluetooth.readString();
  switch (naredba.charAt(0))
    {
      case &#39;R&#39;: // Right, desno
        if (!isTurning) // Provjeri ako se ne rotira vec
        {
          skretanje = &#39;R&#39;;
          isTurning = true;
          turningStartTime = millis(); // Zabiljezi trenutno vrijeme
        }
        break;
      case &#39;L&#39;: // Left, lijevo
        if (!isTurning) //
        {
          skretanje = &#39;L&#39;;
          isTurning = true;
          turningStartTime = millis();
        }
        break;
      default:
        skretanje = &#39;P&#39;; // Balansiranje
        break;
    }
  }
  if (isTurning &amp;&amp; (millis() - turningStartTime &gt;= turningDuration))
{
 
    isTurning = false; // Resetiraj
    skretanje = &#39;P&#39;; // Nastavi balansiranje
    Naprijed();
  }
}
void initMotors() {
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(3, OUTPUT);
  analogWrite(11, LOW);
  analogWrite(10, LOW);
  analogWrite(9, LOW);
  analogWrite(3, LOW);
}
void obradaPodataka() {
  if (!dmpSpreman) return;
  while (!mpuInterrupt &amp;&amp; fifoCount &lt; velicinaPaketa) {
    pid.Compute();
    //ako je na kutu većim od 45 prestaje raditi
    if (input &gt; -45 &amp;&amp; input &lt; 45) {
      if (output &gt; 0)  //output veći od 0
      {
        Naprijed();           //rotacija kotača unaprijed
      } else if (output &lt; 0)  //output manji od 0
      {
        Natrag();  //rotacija kotača natrag
      }
    } else {
      Zaustavi();  //kotači se ne vrte
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus &amp; 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus &amp; 0x02) {
    while (fifoCount &lt; velicinaPaketa) {
      fifoCount = mpu.getFIFOCount();
    }
    mpu.getFIFOBytes(fifoBuffer, velicinaPaketa);
    fifoCount -= velicinaPaketa;
    mpu.dmpGetQuaternion(&amp;q, fifoBuffer);
    mpu.dmpGetGravity(&amp;gravity, &amp;q);
    mpu.dmpGetYawPitchRoll(ypr, &amp;q, &amp;gravity);
    input = ypr[1] * 180 / M_PI;  //vrši se pretvorba u stupnjeve
  }
}

void Naprijed() {
  if (skretanje == &#39;R&#39;) {
    analogWrite(3, map(output, 0, 70, 0, 60));  // Postepeno povećaj
brzinu motora
    analogWrite(10, 0);
  } else if (skretanje == &#39;L&#39;) {
    analogWrite(3, 0);
    analogWrite(10, map(output, 0, 70, 0, 60));  // Postepeno
povećaj brzinu motora
  } else                                          // Naprijed
  {
    analogWrite(3, output);
    analogWrite(10, output);
  }
  analogWrite(9, 0);
  analogWrite(11, 0);
}
void Natrag() {
  if (skretanje == &#39;R&#39;) {
    analogWrite(11, map(output * -1, 0, 70, 0, 60));  // Postepeno
povećaj brzinu motora
    analogWrite(9, 0);
  } else if (skretanje == &#39;L&#39;) {
    analogWrite(11, 0);
    analogWrite(9, map(output * -1, 0, 70, 0, 60));  // Postepeno
povećaj brzinu motora
  } else {
    analogWrite(9, output * -1);
    analogWrite(11, output * -1);
  }
  analogWrite(3, 0);
  analogWrite(10, 0);
}

void Zaustavi() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}
void ispisVrijednost() {
  Serial.print(input);
  Serial.print(&quot;\t&quot;);
  Serial.println(zeljenaVrijednost);
}




![image](https://github.com/marioblazeka/AMT_self_balancing_bot/assets/147752707/61fb4cea-59cd-40bf-8b42-66c833632c08)


![image](https://github.com/marioblazeka/AMT_self_balancing_bot/assets/147752707/c6b358e2-512d-4c9f-9c43-e00f8246ebff)


https://photos.app.goo.gl/SDppAcjDebctiuBS6

https://photos.app.goo.gl/rTajonfmqY6gzyHn7


