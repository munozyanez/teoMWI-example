
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

    MWI::Robot rightArm("teo","rightArm");
    if (rightArm.GetError()!=0)
    {
        std::cout << "MWI::Robot not available. ERROR: " << rightArm.GetError() << std::endl;
        return -1;

    }



    double jointPos=rightArm.GetJoint(2);
    std::vector<double> jointPositions;
    double jointVel;

    std::cout << "jointPos: " << jointPos  <<std::endl;




    double T=0.01;
    int loops = 2/T;
    double vel = 2;
    int jointNumber = 3;

    double lastJointPos = rightArm.GetJoint(jointNumber);

    rightArm.SetControlMode(2);
    rightArm.SetJointVel(jointNumber, vel);

    for(int i=0; i<loops; i++)
    {
        rightArm.GetJoints(jointPositions);
        jointPos=jointPositions[jointNumber];
        jointVel=rightArm.GetJointVelocity(jointNumber);

        std::cout << T << ","
              << vel << ","
              << jointPos << ","
              << jointVel << ","
              << std::endl;
        //std::cout << command << "" << std::endl;

        lastJointPos = jointPos;
        yarp::os::Time::delay(T);


    }

    rightArm.SetJointVel(jointNumber, -vel);

    for(int i=0; i<loops; i++)
    {
        rightArm.GetJoints(jointPositions);
        jointPos=jointPositions[jointNumber];
        jointVel=rightArm.GetJointVelocity(jointNumber);
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
    rightArm.SetJointVel(jointNumber, 0);

    rightArm.SetControlMode(1);
    rightArm.SetJointPos(jointNumber,60);
    yarp::os::Time::delay(3);
    rightArm.SetControlMode(2);

    for(int i=0; i<30; i++)
    {
        vel=(double)i/5;
        rightArm.SetControlMode(2);
        rightArm.SetJointVel(jointNumber, vel);

        std::cout << i << ","
              << "vel: "<< vel << ","
              << "pos: "<< rightArm.GetJoint(jointNumber) << ","
              << std::endl;
        yarp::os::Time::delay(0.3);

    }

//    for(int i=0; i<30; i++)
//    {
//        vel=(double)i/10;
//        rightArm.SetControlMode(2);
//        rightArm.SetJointVel(jointNumber, -vel);

//        std::cout << i << ","
//              << "vel: "<< vel << ","
//              << "pos: "<< rightArm.GetJoint(jointNumber) << ","
//              << std::endl;
//        yarp::os::Time::delay(0.2);

//    }
    rightArm.SetJointVel(jointNumber, 0);

    return 0;
}

