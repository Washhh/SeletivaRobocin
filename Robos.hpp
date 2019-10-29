#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include<bits/stdc++.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<unistd.h>
#include<string.h>
#include<stdlib.h>
#include<termios.h>
#include<thread>
#include<chrono>
#include<time.h>
#include "pb/messages_robocup_ssl_detection.pb.h"
#include "pb/messages_robocup_ssl_geometry.pb.h"
#include "pb/messages_robocup_ssl_wrapper.pb.h"
#include "pb/grSim_Packet.pb.h"
#include "pb/grSim_Commands.pb.h"
#include "pb/grSim_Replacement.pb.h"
#include <iostream>
#include <Eigen/Dense>

#define TempoLimite 0.3

class Robos{
    public:
        Robos();
        void set_robot();
        bool Verificar();
        bool Robot_ID();
        bool ATIVO();
        float getx();
        void kalman();
        void Perda();
        void printRobotInfo();
       

    private:
        int n = 4; // Number of states
        int m = 2; // Number of measurements
        Eigen::MatrixXd A(n, n); // System dynamics matrix
        Eigen::MatrixXd C(m, n); // Output matrix
        Eigen::MatrixXd Q(n, n); // Process noise covariance
        Eigen::MatrixXd R(m, m); // Measurement noise covariance
        Eigen::MatrixXd P(n, n); // Estimate error covariance
        bool Atualizado      = false;
        bool has_orietation  = false;
        double dt            = 1.0/60;
        double cont          = 0.0;
        int ID_ROBOT         = -1;
        int Robo_Ativo       = 0;
        double vx;
        double vy;
        float x;
        float y;
        float x_novo;
        float y_novo;
        float pixel_x;
        float pixel_y;
        float height;
        float orientation;
        float confidence;

};
Robos::Robos(SSL_DetectionRobot &robot){
    
    this.P << 
    0, 0, 0, 0, 
    0, 0, 0, 0, 
    0, 0, 0, 0, 
    0, 0, 0, 0;

    this.A << 
    1, 0, dt, 0, 
    0, 1, 0, dt, 
    0, 0, 1, 0, 
    0, 0, 0, 1;

    this.C << 
    1, 0, 0, 0, 
    0, 1, 0, 0;

    this.Q << 
    .05, .05, .0, .05,
    .05, .05, .0, .05,
    .05, .05, .0, .05,
    .05, .05, .0, .05;

    this.R <<
     5, 5,
     5, 5;

    this.ID_ROBOT = robot.robot_id();
    this.cont = clock();
    this.x = robot.x();
    this.y = robot.y();
    this.x_novo = robot.x();
    this.y_novo = robot.y();
    this.pixel_x = robot.pixel_x();
    this.pixel_y = robot.pixel_y();
    this.height = robot.height();
    this.confidence = robot.confidence();
    this.vx = 0.0;
    this.vy = 0.0;
    if(robot.has_orietation()){
        this.orientation = robot.orientation();
        this.has_orietation = true;
    }
    else{
        this.has_orietation = false;
    } 
}
void Robos::set_robot(SSL_DetectionRobot &robot){
    if(this.Robo_Ativo == 1){
        this.Robo_Ativo = 0;
        
        this.P <<
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0;

        this.x = robot.x();
        this.y = robot.y();
    }
    this.cont= clock();
    this.x_novo = robot.x();
    this.y_novo = robot.y();
    this.pixel_x = robot.pixel_x();
    this.pixel_y= robot.pixel_y();
    this.height = robot.height();
    this.confidence = robot.confidence();
    this.vx = (this.x_novo - this.x)/this.dt;
    this.vy = (this.y_novo - this.y)/this.dt;
    this.Atualizado = true;
    if(robot.has_orietation()){
        this.orientation = robot.orientation();
        this.has_orietation = true;
    } 
    else{
        this.has_orietation = false;
    }
}

bool Robos::Verificar(SSL_DetectionRobot &robot){
    
    if(Robot_ID(robot.robot_id())){
        set_robot(robot);
        return true;
    }
    return false;
}

bool Robos::Robot_ID(int ID){
    
    if(ID == ID_ROBOT){
        return true;
    }

    return false;
}

bool Robos::ATIVO(){
    if(this.Robo_Ativo == 0){
        return true;
    }
    return false;
}

float Robos::getx(){
    return this.x;
}

void Robos::kalman(){

    Eigen::MatrixXd I
    I.setIdentity();
    Eigen::VectorXd X(this.n);
    Eigen::VectorXd Y(this.m);
    X << this.x, this.y, this.vx, this.vy;
    Y << this.x_novo, this.y_novo;

    if(this.Atualizado){ // corrigir, salvar, predict

        // corrigindo meu valor predito anteriormente
        P = A*P*A.transpose() + Q;
        K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
        P = (I - K*C)*P;
        X += K * (Y - C*X);

        // Salvando
        Saida = C*X; 

        //PREDICT
        X_NEW = A*X;
        X_NEW = C*X_NEW;
        
    }
    else{ // salvar, predict

        // Salvando
        Saida = C*X;

        // PREDICT
        X_NEW = A*X;
        X_NEW = C*X_NEW;
    }

    this.x_novo = Saida(1,1);
    this.y_novo = Saida(2,1);
    this.x = X_NEW(1,1);
    this.y = X_NEW(2,1);
    this.Atualizado = false; // garantir que ele só execute correção na próxima vez caso seja entregue seu ponto x,y
}

void Robos::Perda(){ 
                
    if(((clock() - this.cont)/CLOCKS_PER_SEC) >= TempoLimite){
        cout << "o robo Azul de ID " << this.ID_ROBOT << " deu perda" << endl;
        this.Robo_Ativo = 1;
    }
        
}

// printar as informações do robo
void Robos::printRobotInfo() {
    printf("CONF=%4.2f ", this.confidence);
    printf("ID=%3d ",this.ID_ROBOT);
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",this.height,this.x_novo,this.y_novo);
    if (this.has_orietation) {
        printf("ANGLE=%6.3f ",this.orientation);
    }
    printf("RAW=<%8.2f,%8.2f>\n",this.pixel_x,this.pixel_y);
}
