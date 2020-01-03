#include <Socket.h>
#include <Uart.h>
#include <UbloxGPS.h>
#include <sstream>
#include <iostream>
#include <iomanip>

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static inline bool is_base64(unsigned char c)
{ return (isalnum(c) || (c == '+') || (c == '/')); }

std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len)
{
    std::string ret;
    int i = 0, j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    while (in_len--)
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 3; j++) char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

        for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];
        while ((i++ < 3)) ret += '=';
    }
    return ret;
}

void convertTime(long long input, long long& ms, long long& s, long long& m, long long& h, long long& d)
{
    ms = (input % 1000); input = (input / 1000);
    s = (input % 60); input = (input / 60);
    m = ((int)input % 60); input = (input / 60);
    h = ((int)input % 24); d = (int)(input / 24);
}

static wincomm::Uart uart;
static wincomm::UbloxGPS gps;
static bool canSendToNTRIP = false;
static int num_count = 9;

int startGPS(const std::string& ntripCaster, const std::string& auth, const std::string& devAddr)
{
    // NTRIP client parameters
    //std::string ntripCaster = "203.107.45.154";
    int ntripCasterPort = 8002;

    std::string mountpoint = "RTCM32_GGB";
    std::string appName = "NTRIP HelmetAR/1.0";
    //std::string auth = "qxntbp001:9e1553c";
    std::string auth64 = base64_encode((unsigned char*)auth.c_str(), auth.size());

    // Find and start serial GPS
    int baudrate = 38400;
    if (!uart.open(devAddr, baudrate))
    {
        std::cout << "Unable to open serial port" << std::endl;
        return 1;
    }

    gps.start(&uart);
    gps.connectNtripCaster(ntripCaster.c_str(), ntripCasterPort, mountpoint.c_str(),
                           appName.c_str(), auth64.c_str());
    canSendToNTRIP = false;
    return 0;
}
    
bool updateGPS(double t, std::ofstream& out, double &latitude, double &longitude, double &altitude, bool &fix_valid, bool &diff_valid, int &fix_mode, double &signal_score, int &num_valid_satellites)//double &v_north, double &v_east, double &v_down)
{
    bool received_flag=false;
    long long msec, seconds, minutes, hours, days;
    {
        num_count ++;
        if(num_count %10)
        {
            gps.requestStatusPoll();
            gps.requestSatellitePoll();
            gps.requestVelocityPoll();   
            num_count = 0;
        }



        // Receive GPS data
        if (gps.readFromBuffer())
        {
            if (gps.getDirtyMessageTypes() & wincomm::UbloxGPS::MSG_POSITION)
            {
                const wincomm::UbloxGPS::NavigationData& navData = gps.getLatestNavigation();
                convertTime((long long)navData.timeOfWeekMS, msec, seconds, minutes, hours, days);
                //printf("D%lld, T%02lld:%02lld:%02lld.%03lld: lat/lon/alt = %lf, %lf, %lf\n", days + 1,
                //       hours, minutes, seconds, msec, navData.latitude, navData.longitude, navData.altitude);
                latitude = navData.latitude;
                longitude = navData.longitude;
                altitude = navData.altitude;
                  //std::cout << "latitude: " << navData.latitude << ", longitude: " << navData.longitude << ", altitude: " << navData.altitude <<std::endl;
                received_flag = true;

                //out << std::setprecision(14) << t <<  " " << hours <<  " " << minutes <<  " " << seconds <<  " "
                //    << msec <<  " " << navData.latitude <<  " " << navData.longitude <<  " " << navData.altitude << "\n";
            }
            
            if (gps.getDirtyMessageTypes() & wincomm::UbloxGPS::MSG_VELOCITY)
            {
                const wincomm::UbloxGPS::VelocityData& velData = gps.getLatestVelocity();
                convertTime((long long)velData.timeOfWeekMS, msec, seconds, minutes, hours, days);
                //v_north = velData.velocityN;
                //v_east = velData.velocityE;
                //v_down = velData.velocityD;
                //printf("D%lld, T%02lld:%02lld:%02lld.%03lld: velocity-N/E/D = %lf, %lf, %lf\n", days + 1,
                //       hours, minutes, seconds, msec, velData.velocityN, velData.velocityE, velData.velocityD);
            }

            if (gps.getDirtyMessageTypes() & wincomm::UbloxGPS::MSG_SATELLITE)
            {
                //std::cout << "handle latest satellites... " << std::endl;

                const wincomm::UbloxGPS::SatelliteData& satData = gps.getLatestSatellites();
                signal_score = satData.score;
                num_valid_satellites = satData.valid_count;
                //std::cout << "satellites size: " << satData.infoList.size() << std::endl;
                //std::cout << "satellites score: " << satData.score << std::endl;
                //std::cout << "satellites valid count: " << satData.valid_count << std::endl;

                convertTime((long long)satData.timeOfWeekMS, msec, seconds, minutes, hours, days);
                //printf("D%lld, T%02lld:%02lld:%02lld.%03lld: satellites = %d\n", days + 1,
                //    hours, minutes, seconds, msec, satData.infoList.size());
            }

            if (gps.getDirtyMessageTypes() & wincomm::UbloxGPS::MSG_STATUS)
            {
                //std::cout << "getLatestStatus... " << std::endl;
                const wincomm::UbloxGPS::StatusData& statusData = gps.getLatestStatus();
                convertTime((long long)statusData.timeOfWeekMS, msec, seconds, minutes, hours, days);
                fix_valid = statusData.gpsFixValid;
                diff_valid = statusData.diffCorrectionValid;
                fix_mode = statusData.fixMode;
                //printf("D%lld, T%02lld:%02lld:%02lld.%03lld: fixMode = %d, diff = %d\n", days + 1,
                //    hours, minutes, seconds, msec, statusData.fixMode, statusData.diffCorrectionValid);
            }
            gps.clearDirtyMessageTypes();
        }

        // Receive NTRIP messages and send to device
        std::vector<unsigned char> casterData;
        gps.updateNtripCaster(casterData, canSendToNTRIP);
        if (!casterData.empty() && !canSendToNTRIP)
        {
            std::string result((char*)&(casterData[0]), casterData.size());
            if (result.find("OK") != std::string::npos)
            {
                printf("NTRIP caster ready. Now send GPGGA/GNGGA data...\n");
                canSendToNTRIP = true;
            }
        }
        //usleep(10000);
    }
    return received_flag;
}
