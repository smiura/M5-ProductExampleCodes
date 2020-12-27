#ifndef _GPSANALYSE_H_
#define _GPSANALYSE_H_

#include <Arduino.h>
#include "utility/Task.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//#define DEBUG_GPS

typedef struct RMC
{
    String pamstr[13];  
    String Utc;         //1
    char State;         //2
    double Latitude;    //3
    char  LatitudeMark;  //4
    double Longitude;   //5
    char LongitudeMark;  //6
    float TrackSpeed;   //7
    float TrackAngle;   //8
    String Date;        //9
    float Magnetic;     //10
    char Declination;    //11
    int mode;           //12
    String Sum;
}RMC_t;


typedef struct GSA
{
    String pamstr[50];
    char mode2;
    int mode1;
    int PINMap[12];
    float PDOP;
    float HDOP;
    float VDOP;
    String Sum;

}GSA_t;

typedef struct GPSSatellite
{
    bool flag;
    int id;
    int elevation;
    int Azimuth;
    int SNR;
}GPSSatellite_t;

typedef struct GSV
{
    String pamstr[128];
    int size;
    int Number;
    int SatelliteSize;
    GPSSatellite_t Satellite[32];
    String sum;
}GSV_t;

/*
typedef struct GPGGA
{
}GPGGA_t;

typedef struct GPGGA
{
}GPGGA_t;


typedef struct GPGLL
{

}GPGLL_t;
*/

class GPSAnalyse : public Task
{
private:
    
    /* data */
    String _GPS_Str;
    HardwareSerial *_serial;
    SemaphoreHandle_t _xSemaphore = NULL;
    char *GPSReadBuff;

    void Analyse();

    void AnaRMC(String str);
    void AnaGAS(String str);
    void AnaGSV(String str);

    void run(void *data);

    RMC_t _s_RMC;
    GSA_t _s_GAS;
    GSV_t _s_GSV;

public:
    GPSAnalyse();
    ~GPSAnalyse();
    void setSerialPtr(HardwareSerial &serial);
    void upDate();


    RMC_t s_RMC;
    GSA_t s_GAS;
    GSV_t s_GSV;
};






#endif
