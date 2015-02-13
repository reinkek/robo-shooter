#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "mavlink.h"
#include <OptiTrack.h>
#include <math.h>
#include <sys/time.h>

using namespace std;

bool b_useCalibration = true;
double currentPosition[3];
double **currentRotationMatrix;
OptiTrack *objects;
int nbObjects;

class vision
{
public:

    bool update(bool real)
    {
        struct timeval now;
        long mtime, seconds, useconds;

        if (real){
            //Get Optitrack data
            for (int i=0; i<nbObjects; i++)
            {
              if (objects[i].IsEnabled()){
                //reading new values, if there is any
                objects[i].Update();

                // Getting the object position
                objects[i].GetPosition(currentPosition, b_useCalibration);

                // Getting the object orientation
                objects[i].GetRotationMatrix(currentRotationMatrix, b_useCalibration);

                //printing
//                 std::cout << "Object name: " << objects[i].GetObjectName() << std::endl;
//                 std::cout << "Position:    [" << currentPosition[0] << "," << currentPosition[1] << "," << currentPosition[2] << "]" << std::endl;

//                 std::cout << "Orientation: [" << currentRotationMatrix[0][0] << "," << currentRotationMatrix[0][1] << "," << currentRotationMatrix[0][2] << std::endl;
//                 std::cout << "              " << currentRotationMatrix[1][0] << "," << currentRotationMatrix[1][1] << "," << currentRotationMatrix[1][2] << std::endl;
//                 std::cout << "              " << currentRotationMatrix[2][0] << "," << currentRotationMatrix[2][1] << "," << currentRotationMatrix[2][2] << "]" << std::endl;

//                 cout << "Converting RollPitchYaw" << endl;

                 // Roll Pitch Yaw conversions
//                 roll = atan2(currentRotationMatrix[2][1],currentRotationMatrix[1][1]);

//                 pitch = atan2(-currentRotationMatrix[3][1],
//                                sqrt(pow(currentRotationMatrix[3][2],2)+pow(currentRotationMatrix[3][3],2)));

//                 yaw = atan2(currentRotationMatrix[3][2],currentRotationMatrix[3][3]);
//                 cout << "Getting time " << endl;

                 gettimeofday(&now, NULL);

                 seconds  = now.tv_sec;
                 useconds = now.tv_usec;
                 mtime = seconds * 1000000 + useconds;

                 usec = mtime;

                 double quadRot[3][3];
                 for(int j=0;j<3;j++){
                   quadRot[j][0] = currentRotationMatrix[j][1];
                   quadRot[j][1] = -currentRotationMatrix[j][0];
                   quadRot[j][2] = currentRotationMatrix[j][2];
                 }

                 y = currentPosition[0];
                 x = currentPosition[1];
                 z = -currentPosition[2];

                 x_raw = quadRot[0][0];
                 y_raw = quadRot[0][1];
                 z_raw = quadRot[0][2];

                 printf("%d , Time: %u, X: %f, Y: %f, Z: %f,RAW: %f, %f, %f \n",i, (uint)usec,x,y,z,x_raw,y_raw,z_raw);
              }
            }
            return true;
        }
        else {
            // send hardcoded test values\
            return true;
        }
    }

    float x,y,z;
    float x_raw,y_raw,z_raw;
	uint64_t usec;
};

int main() {
  
    std::string vrpn_server_ip = "172.24.68.48:3883"; //you should not change this ip address
    std::cout << "vrpn_server_ip:" << vrpn_server_ip << std::endl;
    
    //The name of the objects that you want to track
    std::string target_name[] = {"quad"};
    
    //computing the number of strings in the target_name
    nbObjects = sizeof( target_name ) / sizeof( std::string );
    
    //initializing the optitrack object(s)
    objects = new OptiTrack[nbObjects];
    for (int i=0; i<nbObjects; i++){
      objects[i].Init(vrpn_server_ip, target_name[i]);
      
      //if you want to use your own coordinate system T, you could load it here
      //T is a homogeneous matrix from OptiTrack coordinate system to your coordinate system
      objects[i].loadCalibrationMatrix("/home/student/group4/cs225a/OptiTrack/VisionCalib.txt");
    }
    
    currentRotationMatrix = new double*[3];
    for (int i=0; i<3; i++)
      currentRotationMatrix[i] = new double[3];
  
    vision vis;

    int fd = -1;

    while(fd == -1) // while open is unsuccessful
    {
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if(fd == -1) { // failed
            cout << "open_port: Unable to open /dev/ttyUSB0. \n";
            sleep(2);
        }
    }
    fcntl(fd, F_SETFL, 0);
    cout << "/dev/ttyUSB0 is open \n";

    // set serial port parameters
    struct termios port_settings;      // structure to store the port settings in

    cfsetispeed(&port_settings, B57600);    // set baud rates
    cfsetospeed(&port_settings, B57600);

    port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    cout << "Applying settings to port" << endl;

    tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // update parameters to pack
    uint8_t sysid = 1; // ID 1 for this quad system
    uint8_t compid = 100; // ID for fake gps/mag component
    int16_t len = 0;
    ssize_t serlen = -1;

    while(true){

        // Send Optitrack data
//        cout << "Updating" << endl;
        vis.update(true);
//        cout << "attempting to pack" << endl;
        //pack gps message
        mavlink_msg_vision_position_estimate_pack(sysid, compid, &msg,
                vis.usec, vis.x, vis.y, vis.z, vis.x_raw, vis.y_raw, vis.z_raw);
        // Copy the message to the send buffer
        len = mavlink_msg_to_send_buffer(buf, &msg);
//        cout << "attempting to send " << len << "\t";
        serlen = write(fd, buf, len);
//        cout << "sent " << serlen << endl;
        usleep(50000); // sleep 100ms for com latency

    }

    return 0;


}
