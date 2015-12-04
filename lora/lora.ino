#include <SoftwareSerial.h>
#include <Akeru.h>
#include <TinyGPS.h>

#define SERIAL_BAUD 9600
#define SIGFOX_LED_PIN 13
#define SIGFOX_POWER 3

#define BOOT_DELAY 500 // ms before initializing modem
#define LOOP_DELAY 18000 // milliseconds between each loop

/* GPS pins */ 
#define RX_PIN 2
#define TX_PIN 3 

SoftwareSerial SoftSerial(RX_PIN, TX_PIN);
TinyGPS gps;

/* MQ-2 */
const int   gas_pin = A0;
#define GAS_CONST 42.73
#define NUM_READS 11

/* PM */
#define PM_PIN 9
unsigned long sample_time = 30000;

typedef struct  s_data{
    int     gas; //2bytes
    int     pm; //2bytes
    long    lat; //4bytes
    long    lon; //4bytes
}               t_data;

t_data          g_data;

boolean ft_sendMessage(void);
boolean ft_getGPS(void);
boolean ft_getGas(void);
boolean ft_getPM(void);

void    setup(void)
{
    Serial.begin(SERIAL_BAUD);
    pinMode(SIGFOX_LED_PIN, OUTPUT);
    digitalWrite(SIGFOX_LED_PIN, LOW);

    delay(BOOT_DELAY);  // let the akeru board wake up gently

    Akeru.begin();
    Akeru.setPower(SIGFOX_POWER);
    SoftSerial.begin(SERIAL_BAUD);

    pinMode(PM_PIN, INPUT);
    
    for(int i = 0; i < 3; ++i)
    {
        digitalWrite(SIGFOX_LED_PIN, HIGH);
        delay(500);
        digitalWrite(SIGFOX_LED_PIN, LOW);
        delay(500);
    }
}

void    loop(void)
{
    if (ft_getGPS())
    {
        ft_getGas();
        ft_getPM();
        ft_sendMessage();
        delay(LOOP_DELAY);
    }
}

boolean ft_sendMessage(void)
{
    Akeru.begin();
    delay(500);
    if (!Akeru.isReady())
        return(false);
    digitalWrite(SIGFOX_LED_PIN, HIGH);
    if (!Akeru.send(&g_data, sizeof(g_data)))
    {
        digitalWrite(SIGFOX_LED_PIN, LOW);
        return(false);
    }
    digitalWrite(SIGFOX_LED_PIN, LOW);
    return(true);
}

boolean ft_getGPS(void)
{
    unsigned long   fix_age;

    SoftSerial.listen();
    while(SoftSerial.available())
    {
        gps.encode(SoftSerial.read());
    }
    gps.get_position(&g_data.lat, &g_data.lon, &fix_age);
    if (fix_age == TinyGPS::GPS_INVALID_AGE || fix_age > 5000)
    {
        Serial.println("no fix or no valid data");
        return(false);
    }
    else
    {
        Serial.println("valid data obtained");
    }
    return(true);
}

boolean ft_getGas(void)
{
    float   sensor_volt;
    float   Rs_gas;
    float   ratio;
    int     sensor_value;
    int     i;

    i = 0;
    while (i < NUM_READS)
    {
        sensor_value = analogRead(gas_pin);
        i++;
        delay(500);
    }
    sensor_value = analogRead(gas_pin);
    sensor_volt = (float)sensor_value / (1024 * 5.0);
    Rs_gas = (5.0 - sensor_volt) / sensor_volt;
    ratio = (Rs_gas / GAS_CONST) * 100;
    g_data.gas = (int)ratio;
    return(true);
}

boolean ft_getPM(void)
{
    unsigned long   start_time;
    unsigned long   lpo;
    float           ratio;
    float           concentration;
    bool            check;

    check = false;
    while (check == false)
    {
        lpo = 0;
        start_time = millis();
        while ((millis() - start_time) <= sample_time)
        {
            lpo += pulseIn(PM_PIN, LOW);
        }
        ratio = lpo / (sample_time * 10.0);
        concentration = (1.1 * pow(ratio, 3)) - (3.8 * pow(ratio, 2)) + (520 * ratio) + 0.62;
        Serial.println(concentration);
        if (concentration >= 0.0 && concentration <= 8000.0)
        {
            concentration *= 100;
            g_data.pm = (int)concentration;
            check = true;
        }
    }
    return (true);
}
