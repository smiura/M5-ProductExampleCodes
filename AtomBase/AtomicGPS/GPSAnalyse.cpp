#include "GPSAnalyse.h"

GPSAnalyse::GPSAnalyse()
{
    _GPS_Str.clear();
}


void GPSAnalyse::setSerialPtr(HardwareSerial &serial)
{
    _serial = &serial;
    _xSemaphore = xSemaphoreCreateMutex();
}

GPSAnalyse::~GPSAnalyse()
{

}

void GPSAnalyse::run(void *data)
{
    GPSReadBuff = (char*)calloc(1024,sizeof(char));
    Serial.println("GPS Task");
    while (1)
    {
        if (_serial->available())
        {
            
            memset(GPSReadBuff, 1024, sizeof(char));
            _serial->readBytes(GPSReadBuff, _serial->available());
            _GPS_Str.concat(GPSReadBuff);
            xSemaphoreTake(_xSemaphore, portMAX_DELAY);
            Analyse();
            xSemaphoreGive(_xSemaphore);
            delay(1);
        }
        else
        {
            delay(5);
        }
    }
}

void GPSAnalyse::upDate()
{
    xSemaphoreTake(_xSemaphore, portMAX_DELAY);
    memcpy(&s_RMC,&_s_RMC,sizeof(RMC_t));
    memcpy(&s_GAS,&_s_GAS,sizeof(GSA_t));
    memcpy(&s_GSV,&_s_GSV,sizeof(GSV_t));
    xSemaphoreGive(_xSemaphore);
}

void GPSAnalyse::AnaRMC(String str)
{
    str = str.substring(str.indexOf("RMC"), str.length());
    if (str.indexOf('*') != -1)
    {
        int indexsub = str.indexOf('*');
        String sumstr = str.substring(indexsub + 1, str.length());
        str = str.substring(0, indexsub);
        //Serial.println(sumstr);
    }

    int index = 0, last_index = 0, pamindex = 1;

    while (str.indexOf(',', index) != -1)
    {
        index = str.indexOf(',', index + 1);
        last_index = str.indexOf(',', index + 1);
        //Serial.printf("index:%d,%d\n", index, last_index);
        if (index != -1)
        {
            last_index = (last_index == -1) ? str.length() : last_index;
            if ((last_index - index) > 1)
            {
                String pamstr = str.substring(index + 1, last_index);
#ifdef DEBUG_GPS
                Serial.printf("%d:", pamindex);
                Serial.println(pamstr);
#endif
                _s_RMC.pamstr[pamindex] = pamstr;
            }
            else
            {
                _s_RMC.pamstr[pamindex].clear();
            }

            pamindex++;
        }
    }

    _s_RMC.Utc = _s_RMC.pamstr[1];
    if (_s_RMC.pamstr[2].length() != 0)
    {
        _s_RMC.State = _s_RMC.pamstr[2].charAt(0);
    }

    {
        _s_RMC.Latitude = _s_RMC.pamstr[3].substring(0, 2).toDouble() + _s_RMC.pamstr[3].substring(2, 4).toDouble() / 60 + _s_RMC.pamstr[3].substring(5, 9).toDouble() / 600000;
    }
    //_s_RMC.Latitude        = _s_RMC.pamstr[3].toDouble();
    _s_RMC.LatitudeMark = _s_RMC.pamstr[4].charAt(0);
    {
        _s_RMC.Longitude = _s_RMC.pamstr[5].substring(0, 3).toDouble() + _s_RMC.pamstr[5].substring(3, 5).toDouble() / 60 + _s_RMC.pamstr[5].substring(6, 10).toDouble() / 600000;
    }
    //_s_RMC.Longitude       = _s_RMC.pamstr[5].toDouble();
    _s_RMC.LongitudeMark = _s_RMC.pamstr[6].charAt(0);
    _s_RMC.TrackSpeed = _s_RMC.pamstr[7].toFloat();
    _s_RMC.TrackAngle = _s_RMC.pamstr[8].toFloat();
    _s_RMC.Date = _s_RMC.pamstr[9];
    _s_RMC.Magnetic = _s_RMC.pamstr[10].toFloat();
    _s_RMC.Declination = _s_RMC.pamstr[11].charAt(0);
    _s_RMC.mode = _s_RMC.pamstr[12].charAt(0);
}

void GPSAnalyse::AnaGAS(String str)
{
    str = str.substring(str.indexOf("GSA"), str.length());

    if (str.indexOf('*') != -1)
    {
        int indexsub = str.indexOf('*');
        String sumstr = str.substring(indexsub + 1, str.length());
        str = str.substring(0, indexsub);
        //Serial.println(sumstr);
    }

    int index = 0, last_index = 0, pamindex = 1;
    while (str.indexOf(',', index) != -1)
    {
        index = str.indexOf(',', index + 1);
        last_index = str.indexOf(',', index + 1);
        if (index != -1)
        {
            last_index = (last_index == -1) ? str.length() : last_index;
            if ((last_index - index) > 1)
            {
                String pamstr = str.substring(index + 1, last_index);
#ifdef DEBUG_GPS
                Serial.printf("%d:", pamindex);
                Serial.println(pamstr);
#endif
                _s_GAS.pamstr[pamindex] = pamstr;
            }
            else
            {
                _s_GAS.pamstr[pamindex].clear();
            }
            pamindex++;
        }
    }

    _s_GAS.mode2 = _s_GAS.pamstr[1].charAt(0);
    _s_GAS.mode1 = _s_GAS.pamstr[2].toInt();
    for (size_t i = 0; i < 12; i++)
    {
        _s_GAS.PINMap[i] = _s_GAS.pamstr[3 + i].toInt();
    }
    _s_GAS.PDOP = _s_GAS.pamstr[15].toFloat();
    _s_GAS.HDOP = _s_GAS.pamstr[16].toFloat();
    _s_GAS.VDOP = _s_GAS.pamstr[17].toFloat();
}

void GPSAnalyse::AnaGSV(String str)
{
    //Serial.println(str);
    str = str.substring(str.indexOf("GSV"), str.length());

    if (str.indexOf('*') != -1)
    {
        int indexsub = str.indexOf('*');
        String sumstr = str.substring(indexsub + 1, str.length());
        str = str.substring(0, indexsub);
        //Serial.println(sumstr);
    }

    int index = 0, last_index = 0, pamindex = 1;
    while (str.indexOf(',', index) != -1)
    {
        index = str.indexOf(',', index + 1);
        last_index = str.indexOf(',', index + 1);
        if (index != -1)
        {
            last_index = (last_index == -1) ? str.length() : last_index;
            if ((last_index - index) > 1)
            {
                String pamstr = str.substring(index + 1, last_index);
#ifdef DEBUG_GPS
                Serial.printf("%d:", pamindex);
                Serial.println(pamstr);
#endif
                _s_GSV.pamstr[pamindex] = pamstr;
            }
            else
            {
                _s_GSV.pamstr[pamindex].clear();
            }
            pamindex++;
        }
    }
    int SatelliteSize = (pamindex - 4) / 4;
    //Serial.printf("Number%d\n", SatelliteSize);

    _s_GSV.size = _s_GSV.pamstr[1].toInt();
    _s_GSV.Number = _s_GSV.pamstr[2].toInt();
    _s_GSV.SatelliteSize = _s_GSV.pamstr[3].toInt();

    if (_s_GSV.Number == 1)
    {
        for (size_t i = 0; i < 32; i++)
        {
            _s_GSV.Satellite[i].flag = false;
        }
    }
    for (size_t i = 0; i < SatelliteSize; i++)
    {
        int id = _s_GSV.pamstr[4 + (i * 4) + 0].toInt();
        if( id >= 32 ) continue;
        if(( 7 + (i * 4)) > 50 )
        {
            break;
        }
        _s_GSV.Satellite[id].elevation = _s_GSV.pamstr[4 + (i * 4) + 1].toInt();
        _s_GSV.Satellite[id].Azimuth = _s_GSV.pamstr[4 + (i * 4) + 2].toInt();
        _s_GSV.Satellite[id].SNR = (_s_GSV.pamstr[4 + (i * 4) + 3].length() == 0) ? -1 : _s_GSV.pamstr[4 + (i * 4) + 3].toInt();
        _s_GSV.Satellite[id].flag = true;
    }

    if (_s_GSV.Number == _s_GSV.size)
    {
        for (size_t i = 0; i < 32; i++)
        {
            if (_s_GSV.Satellite[i].flag == true)
            {
                Serial.printf("ID %d:%d,%d,%d\n", i, _s_GSV.Satellite[i].elevation, _s_GSV.Satellite[i].Azimuth, _s_GSV.Satellite[i].SNR);
            }
        }
    }
}

void GPSAnalyse::Analyse()
{
    while (_GPS_Str.indexOf('\r') != -1)
    {
        int index = _GPS_Str.indexOf('\r');
        String str = _GPS_Str.substring(0, index);
        _GPS_Str = _GPS_Str.substring(index + 5, _GPS_Str.length());

        str.trim();

        //Serial.println(str);

        if (str.indexOf("RMC") != -1)
        {
            AnaRMC(str);
        }
        else if (str.indexOf("VTG") != -1)
        {
        }
        else if (str.indexOf("GGA") != -1)
        {

        }
        else if (str.indexOf("GSA") != -1)
        {
            //Serial.print("GSA:");
            //AnaGAS(str);
        }
        else if (str.indexOf("GSV") != -1)
        {
            //Serial.print("GPGSV:");
            AnaGSV(str);
        }
        else if (str.indexOf("GPGLL") != -1)
        {
        }
    }
}
