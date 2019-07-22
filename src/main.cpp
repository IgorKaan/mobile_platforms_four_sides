#include <Arduino.h>
#include <Wire.h>
#include <AX12A.h> 
#include "math.h"
#include <I2Cdev.h>
#include <MotorControl.h>
#include <MqttClient.h>
#include <PubSubClient.h>
#include <stdio.h>
#include <HardwareSerial.h>

std::string receivedData;
std::string sign;
std::string angle;
std::string move;
std::string rotate;
std::string finish;

short speed = 100;
short correctValue = 0;

uint8_t finishValue = -1;
uint8_t splitindex;
uint8_t rotateValue = -1;
uint8_t moveForwardValue = -1;
uint8_t platformNumber = 201;
uint8_t sensorId;

bool moveSide = true;

// const char* ssid = "SPEECH_405";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.61";

const char* ssid = "213_Guest";
const char* password = "11081975";
const char* mqtt_server = "192.168.1.72";

// const char* ssid = "Keenetic-SPIRAS";
// const char* password = "1122334455";
// const char* mqtt_server = "192.168.1.65";

mqttClient mqtt(ssid, password, mqtt_server);
MotorControl GyroRobot;

void callback(char* topic, byte* message, unsigned int length)
{
    char platformControlTopic[64];

    sprintf(platformControlTopic, "platforms/%d", platformNumber);
    
    if (strcmp(topic, platformControlTopic)==0) {

        receivedData = "";
        sign = "";
        angle = "";
        move = "";
        rotate = "";
        finish = "";
       
        int digit_sign;
        
        for (int i = 0; i < length; i++)
            {
                receivedData += (char)message[i]; 
            } 
            sign = receivedData[0];
            angle = receivedData.substr(1, 3);
            move = receivedData[4];
            rotate = receivedData[5];
            finish = receivedData[6];

            if (sign == "0") {
                digit_sign = -1;
            }
            else {
                digit_sign = 1;
            }

            correctValue = digit_sign * atoi(angle.c_str());
            moveForwardValue = atoi(move.c_str());
            rotateValue = atoi(rotate.c_str());
            finishValue = atoi(finish.c_str());
        } 
} 

void setup()
{
    Serial.begin(115200);
    GyroRobot = MotorControl();
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    mqtt.subscribe(platformNumber);
}

void loop()
{
    // mqtt.initClientLoop();
    // if (correctValue <= 90 && correctValue >= -90 && moveForwardValue == 1 && rotateValue == 0 && finishValue == 0) {
    //     mqtt.initClientLoop();
    //     GyroRobot.goForward(speed*0.7);        
    // }
    // else if ((correctValue > 90 || correctValue < -90) && moveForwardValue == 1 && rotateValue == 0 && finishValue == 0) {
    //     mqtt.initClientLoop();
    //     GyroRobot.goBackward(speed*0.7);        
    // } 
    // else if (finishValue == 1)
    // {
    //     GyroRobot.stopMovement();
    // }
    // else if (rotateValue == 1 && finishValue == 0) {
    //     if (correctValue > 90) {
    //         mqtt.initClientLoop();
    //         GyroRobot.turnLeft(speed*0.5);       
    //     }
    //     else if (correctValue < -90) {
    //         mqtt.initClientLoop();
    //         GyroRobot.turnRight(speed*0.5);
    //     }
    //     else if (correctValue < 90 && correctValue > 0) {
    //         mqtt.initClientLoop();
    //         GyroRobot.turnRight(speed*0.5);
    //     }
    //     else if (correctValue > -90 && correctValue < 0) {
    //         mqtt.initClientLoop();
    //         GyroRobot.turnLeft(speed*0.5);
    //     }
    // }
    
    // if (correctValue <= 90 && correctValue >= -90) {
    //     if (moveForwardValue == 1 && rotateValue == 0) {
    //         GyroRobot.goForward(speed*0.7);
    //         Serial.println("forward");
    //     }
    //     if (rotateValue == 0 && moveForwardValue == 0) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    //     if (rotateValue == 1 && moveForwardValue == 0)
    //     {
    //         //GyroRobot.stopMovement();
    //         if ((correctValue > 0) && (correctValue < 90)) {
    //             GyroRobot.turnRight(speed*0.5);
    //             Serial.println("left");
    //         }
    //         else if ((correctValue < 0) && (correctValue > -90)) {
    //             GyroRobot.turnLeft(speed*0.5);
    //             Serial.println("right");
    //         }
    //     }   
    //     else if (finishValue == 1) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    // }
    // else if (correctValue >= 90 || correctValue <= -90) {
    //     if (moveForwardValue == 1 && rotateValue == 0) {
    //         GyroRobot.goBackward(speed*0.7);
    //         Serial.println("back");
    //     }
    //     if (rotateValue == 0 && moveForwardValue == 0) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    //     if (rotateValue == 1 && moveForwardValue == 0)
    //     {
    //        //GyroRobot.stopMovement();
    //        if (correctValue >= 90) {
    //             GyroRobot.turnLeft(speed*0.5);
    //             Serial.println("left");    
    //         }
    //         else if (correctValue <= -90) {  
    //             GyroRobot.turnRight(speed*0.5);  
    //             Serial.println("right");   
    //         }
    //     }   
    //     else if (finishValue == 1) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }    
    // }

    // if (correctValue <= 45 && correctValue >= -45) {
    //     if (moveForwardValue == 1 && rotateValue == 0) {
    //         GyroRobot.goForward(speed*0.7);
    //         Serial.println("forward");
    //     }
    //     if (rotateValue == 0 && moveForwardValue == 0) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    //     if (rotateValue == 1 && moveForwardValue == 0)
    //     {
    //         //GyroRobot.stopMovement();
    //         if (correctValue > 0) {
    //             GyroRobot.turnLeft(speed*0.5);
    //             Serial.println("left");
    //         }
    //         else if (correctValue < 0) {
    //             GyroRobot.turnRight(speed*0.5);
    //             Serial.println("right");
    //         }
    //     }   
    //     else if (finishValue == 1) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    // }

    // else if (correctValue >= 135 || correctValue <= -135) {
    //     if (moveForwardValue == 1 && rotateValue == 0) {
    //         GyroRobot.goBackward(speed*0.7);
    //         Serial.println("back");
    //     }
    //     if (rotateValue == 0 && moveForwardValue == 0) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    //     if (rotateValue == 1 && moveForwardValue == 0)
    //     {
    //        //GyroRobot.stopMovement();
    //        if (correctValue > 0) {
    //             GyroRobot.turnRight(speed*0.5);
    //             Serial.println("right");    
    //         }
    //         else if (correctValue < 0) {  
    //             GyroRobot.turnLeft(speed*0.5);  
    //             Serial.println("left");   
    //         }
    //     }   
    //     else if (finishValue == 1) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }    
    // }

    // else if (correctValue > 45 || correctValue < 135) {
    //     if (moveForwardValue == 1 && rotateValue == 0) {
    //         GyroRobot.goRight(speed*0.7);
    //         Serial.println("back");
    //     }
    //     if (rotateValue == 0 && moveForwardValue == 0) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }
    //     if (rotateValue == 1 && moveForwardValue == 0)
    //     {
    //        //GyroRobot.stopMovement();
    //        if (correctValue > 0) {
    //             GyroRobot.turnRight(speed*0.5);
    //             Serial.println("right");    
    //         }
    //         else if (correctValue < 0) {  
    //             GyroRobot.turnLeft(speed*0.5);  
    //             Serial.println("left");   
    //         }
    //     }   
    //     else if (finishValue == 1) {
    //         GyroRobot.stopMovement();
    //         Serial.println("stop");
    //     }    
    // }
    
    //mqtt.pubFeedback(outputData,platformNumber);

    mqtt.initClientLoop();
    if (correctValue <= 45 && correctValue >= -45 && moveForwardValue == 1 && rotateValue == 0 && finishValue == 0) {
        mqtt.initClientLoop();
        GyroRobot.goForward(speed);        
    }
    else if ((correctValue >= 135 || correctValue <= -135) && moveForwardValue == 1 && rotateValue == 0 && finishValue == 0) {
        mqtt.initClientLoop();
        GyroRobot.goBackward(speed);        
    } 
    if (correctValue > 45 && correctValue < 135 && moveForwardValue == 1 && rotateValue == 0 && finishValue == 0) {
        mqtt.initClientLoop();
        GyroRobot.goRight(speed);        
    }
    else if ((correctValue < -45 && correctValue > -135) && moveForwardValue == 1 && rotateValue == 0 && finishValue == 0) {
        mqtt.initClientLoop();
        GyroRobot.goLeft(speed);        
    } 
    else if (finishValue == 1)
    {
        GyroRobot.stopMovement();
    }
    else if (rotateValue == 1 && finishValue == 0) {

        if (correctValue <= 45 && correctValue >= 0) {
            mqtt.initClientLoop();
            GyroRobot.turnRight(speed);
        }
        else if (correctValue >= -45 && correctValue <= 0) {
            mqtt.initClientLoop();
            GyroRobot.turnLeft(speed);
        }
        else if (correctValue >= 135) {
            mqtt.initClientLoop();
            GyroRobot.turnLeft(speed);       
        }
        else if (correctValue <= -135) {
            mqtt.initClientLoop();
            GyroRobot.turnRight(speed);
        }
        else if (correctValue > 45 && correctValue < 90) {
            mqtt.initClientLoop();
            GyroRobot.turnLeft(speed);
        }
        else if (correctValue >= 90 && correctValue < 135) {
            mqtt.initClientLoop();
            GyroRobot.turnRight(speed);
        }
        else if (correctValue < -45 && correctValue >= -90) {
            mqtt.initClientLoop();
            GyroRobot.turnRight(speed);
        }
        else if (correctValue < -90 && correctValue >= -135) {
            mqtt.initClientLoop();
            GyroRobot.turnLeft(speed);
        }
    }
}
  

