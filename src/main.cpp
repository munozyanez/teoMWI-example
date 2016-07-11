
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream

#include <yarp/os/all.h>


#include "MiddlewareInterface.h"

using namespace std;



int main()
{
    std::cout << "[error]";
    //MWI::Port imuPort;
    //INITIALISE AND CHECK YARP
    yarp::os::Network yarpNet;

    if ( !yarpNet.checkNetwork(2) )
    {
        std::cout << "[error] %s found no YARP network (try running \"yarp detect --write\")." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[success] YARP network found." << std::endl;
    }



    //Setup imu middleware port
    MWI::Port imuPort("/inertial");
    std::stringstream dataIndices, imudata;

    // i = 0,1,2 --> euler angles (deg)
     // i = 3,4,5 --> linear acceleration (m/s²)
     // i = 6,7,8 --> angular speed (deg/s)
     // i = 9,10,11 --> magnetic field (arbitrary units)
    dataIndices << 3 << " " <<  "4 5" ;// X Y Z [m/s^2]

    int indices [] = {3, 4, 5};
    std::vector<double> imuAccel (3);



    //READ SENSOR
    double a_x,a_y,a_z;

    imuPort.Read(dataIndices, imudata);
    //imuPort.ShowAllData();
    imudata >> a_x;
    imudata >> a_y;
    imudata >> a_z;

    std::cout << a_x << a_y << a_z <<std::endl;

    imuPort.Read(indices,imuAccel);

    std::cout << "vector" << imuAccel[0] << imuAccel[1] << imuAccel[2] <<std::endl;

    //Robot teo right arm
    std::stringstream robConfig;
    //YARP device
    robConfig << "device remote_controlboard" << " ";
    //To what will be connected
    robConfig << "remote /teo/rightLeg" << " ";
    //How will be called on YARP network
    robConfig << "local /local/rightLeg/" << " ";
    MWI::Robot rightLeg(robConfig);


    double jointPos=rightLeg.GetJoint(2);
    double jointVel;

    std::cout << "jointPos: " << jointPos  <<std::endl;




    double T=0.01;
    int loops = 2/T;
    double vel = 1.1;
    int jointNumber = 2;

    double lastJointPos = rightLeg.GetJoint(jointNumber);

    //rightLeg.SetJointVel(jointNumber, -vel);

    rightLeg.SetJointPos(jointNumber,-10);

    for(int i=0; i<loops; i++)
    {
        jointPos=rightLeg.GetJoint(jointNumber);
        jointVel=rightLeg.GetJointVelocity(jointNumber);

        std::cout << T << ","
              << vel << ","
              << jointPos << ","
              << jointVel << ","
              << std::endl;
        //std::cout << command << "" << std::endl;

        lastJointPos = jointPos;
        yarp::os::Time::delay(T);


    }

    //rightLeg.SetJointVel(jointNumber, vel);

    for(int i=0; i<loops; i++)
    {
        jointPos=rightLeg.GetJoint(jointNumber);
        jointVel=rightLeg.GetJointVelocity(jointNumber);
        std::cout << T << ","
              << vel << ","
              << jointPos << ","
              << jointVel << ","
              << std::endl;
        //std::cout << command << "" << std::endl;

        lastJointPos = jointPos;
        yarp::os::Time::delay(T);


    }
    //step =0;
    rightLeg.SetJointVel(jointNumber, 0);





    return 0;
}

