/*
 * Oregon Senseur ultrason - HC-SR04 sur ATtiny85 avec watchdog
 *  OregonHCSR04_ATtiny85_Wdt.ino
 * Fonctions Oregon reprises de http://connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
 * L'ATtiny85 doit être configuré en hhorloge 8 MHz pour respecter les timing RF.
 *
 * Modifier la constante "HAUTEURBAC" en fonction du niveau à mesurer dans les limites du HC-SR04.
 * Modifier la constante "OREGON_ID" si plusieurs modules montés ou pour éviter les conflits avec d'autre sondes.
 */
 

/* 
 * Version pour ATtiny85
 * ATMEL ATtiny85 / ARDUINO
 * 
 *                   +-\/-+
 *   Ain0 (D5) PB5  1|    |8  Vcc
 *   Ain3 (D3) PB3  2|    |7  PB2 (D2) Ain1
 *   Ain2 (D4) PB4  3|    |6  PB1 (D1) pwm1
 *             GND  4|    |5  PB0 (D0) pwm0
 *                   +----+

*/


// ----------------------------------------------------------------------------
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


// Déclaration pour Tx RF Oregon
#define OREGON_ID  0xBB                         // ID sonde, doit être unique
#define TX_PIN 0							    // TX RF, broche 5 ATtiny85
#define SEND_HIGH() digitalWrite(TX_PIN, HIGH)
#define SEND_LOW() digitalWrite(TX_PIN, LOW)

const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME*2;
byte thn132n ; 
float temp ;
byte  batt ;

// Déclaration pour HC-SR04
#define echoPin 4           // broche Echo, broche 3 ATtiny85
#define trigPin 3           // broche Trigger (declenchement), broche 2 ATtiny85

#define HAUTEURBAC 90
int maximumRange = 200;     // distance Maximale acceptée (en cm)
int minimumRange = 0;       // distance Minimale acceptée (en cm)
long duration, distance;    // Durée utilisé pour calculer la distance

volatile byte wdt_overflow_count = 0;
boolean sendok = true ;


// ----------------------------------------------------------------------------
// Fonctions bas niveau Oregon
// ----------------------------------------------------------------------------
/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void) 
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void) 
{
   SEND_LOW();
   delayMicroseconds(TIME);
   SEND_HIGH();
   delayMicroseconds(TWOTIME);
   SEND_LOW();
   delayMicroseconds(TIME);
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data) 
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data) 
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 

 
// ----------------------------------------------------------------------------
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size)
{
    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}
 
/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
	if ( thn132n ) sendQuarterLSB(0x00);									// A modifier pour ATtiny85
	else
	{
		byte POSTAMBLE[]={0x00};
		sendData(POSTAMBLE, 1);  
	}
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}
 
 
// ----------------------------------------------------------------------------
/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type) 
{
  data[0] = type[0];
  data[1] = type[1];
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel) 
{
    data[2] = channel;
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID) 
{
  data[3] = ID;
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setTemperature(byte *data, float temp) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
 
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
 
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
 
  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;
 
  // Set temperature float part
  data[4] |= (tempFloat << 4);
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setHumidity(byte* data, byte hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;
 
  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}
 
// ----------------------------------------------------------------------------
/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
	if ( thn132n )														// A modifier pour ATtiny85
	{
		int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
		data[6] |=  (s&0x0F) << 4;     data[7] =  (s&0xF0) >> 4;
	}
	else data[8] = ((Sum(8, data) - 0xa) & 0xFF);
}
 
 
// -------------------------------------------------------------------------------------------
// Fonction principale d'envoie trame Oregon
// -------------------------------------------------------------------------------------------
void sendOregonTNHN132N(float temperature, byte batt)
{
	thn132n = 1 ; 			// Sonde TNHN132N
	// Buffer for Oregon message
	byte OregonMessageBuffer[8];

	byte ID[] = {0xEA,0x4C};	// Create the Oregon message for a temperature only sensor (TNHN132N)
	setType(OregonMessageBuffer, ID);
	setChannel(OregonMessageBuffer, 0x20);
	setId(OregonMessageBuffer, OREGON_ID);

	setBatteryLevel(OregonMessageBuffer, batt); 		// 0 : low, 1 : high
	setTemperature(OregonMessageBuffer, temperature);
 
	// Calculate the checksum
	calculateAndSetChecksum(OregonMessageBuffer);
 
	// Send the Message over RF
	sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
	// Send a "pause"
	SEND_LOW();
	delayMicroseconds(TWOTIME*8);
	// Send a copie of the first message. The v2.1 protocol send the
	// message two time 
	sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
 
	SEND_LOW();
}


// ----------------------------------------------------------------------------
// Interruption watchdog, configuré pour réveiller le proc tous les 8s (maximum)
// ----------------------------------------------------------------------------
ISR(WDT_vect) 
{
	sleep_disable() ;			        // Clear the SE (sleep enable) bit. 
	if (++wdt_overflow_count > 15)		// Int. WDT tous 8 secondes x 15 => 120s 
	{
		sendok = true ;
		wdt_overflow_count = 0;
	}
}

// ----------------------------------------------------------------------------
// Mesure la distance avec le HC-SR04, calcule la différence par rapport à la hauteur totale de la cuve
// ----------------------------------------------------------------------------
long getDistance()
{
          digitalWrite(trigPin, LOW); 
          delayMicroseconds(2); 

          digitalWrite(trigPin, HIGH);
          delayMicroseconds(10); 
 
          digitalWrite(trigPin, LOW);

          // Attend que la broche Echo passe au niveau HAUT 
          // retourne la durée
          duration = pulseIn(echoPin, HIGH);
 
          //Calculer la distance (en cm, basé sur la vitesse du son).
          distance = duration/58.2;
 
          // Si la distance mesurée est HORS des valeurs acceptables
          if (distance >= maximumRange || distance <= minimumRange)
          {                                                         
            distance = HAUTEURBAC ;             // Retourner HAUTEURBAC si hors limite.
          }
          
          return HAUTEURBAC - distance ;        // Retourne Hauteur bac à sel - distance mesurée entre le haut du bac et le sel.
}

// ----------------------------------------------------------------------------
void setup()
{
	pinMode(TX_PIN, OUTPUT);
	SEND_LOW();   

        // Activer les broches HCSR04
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
        
    // Configure le watchdog
	// Clear the reset flag.
	MCUSR &= ~(1<<WDRF);
  
	// In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
	WDTCR |= (1<<WDCE) | (1<<WDE);

	// set new watchdog timeout prescaler value 
	WDTCR = 1<<WDP0 | 1<<WDP3; 			// 8.0 seconds
  
	// Enable the WD interrupt (note no reset).
	WDTCR |= _BV(WDIE);
	
	// Use the Power Down sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	
}
 
 
// ----------------------------------------------------------------------------
void loop()
{
	if (sendok)
	{          
          // Lecture distance
          temp = getDistance() ;
          batt = 1 ;
          
          // Envoie valeur sur RF.
          sendOregonTNHN132N(temp, batt) ;
	  sendok = false ;
	}
	sleep_mode() ;				// Mise en veille. 
}
// ----------------------------------------------------------------------------
