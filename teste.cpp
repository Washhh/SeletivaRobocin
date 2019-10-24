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

//classe dos Robos
class Robos{
    public:
        bool Atualizado = false;
        bool Identificado = false;
        bool has_orietation;
        double cont     = 0.0;
        int ID_ROBOT    = -1;
        int Robo_Ativo  = 0;
        double dx;
        double dy;
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
//................
//Kalman
void Kalman_filter(Robos Robot){
    int n = 4; // Number of states
    int m = 2; // Number of measurements

    double dt = 1.0/60; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance
    
    // sentando as matrizes
    A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
    C << 1, 0, 0, 0, 0, 1, 0, 0;
    Q << .05, .05, .0, .05,.05, .05, .0, .05,.05, .05, .0, .05,.05, .05, .0, .05;
    R << 5, 5, 5, 5;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100, .1, .1, .1, .1,10000, 10, .1;

    //Montando o filtro

    I.setIdentity();
    Eigen::VectorXd X(n);
    Eigen::VectorXd Y(m);
    X << Robot.x, Robot.y, Robot.vx, Robot.vy;
    Y << Robot.x_novo, Robot.y_novo;

    if(Robot.Atualizado == true){ // corrigir, salvar, predict

        // corrigindo meu valor predito anteriormente
        P = A*P*A.transpose() + Q;
        K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
        //P = (I - K*C)*P;
        X += K * (Y - C*X);

        // Salvando
        Saida = C*X; 

        //PREDICT
        X_NEW = A*X; 
        
    }
    else{ // salvar, predict

        // Salvando
        Saida = C*X;

        // PREDICT
        X_NEW = A*X;
        X_NEW = C*X_NEW;
    }

    Robot.x_novo = Saida(1,1);
    Robot.y_novo = Saida(2,1);
    Robot.x = X_NEW(1,1);
    Robot.y = X_NEW(2,1);
    Robot.Atualizado = false; // garantir que ele só execute correção na próxima vez caso seja entregue seu ponto x,y
}
//FIltro de perda
void PERDA(int Cont_Indice_TIME, Robos *TIME){

        for(int x = 0; x < Cont_Indice_TIME; x++){
                
            //cout << "tempo de " << x << " : " << ((clock() - vetor[x].contador)/CLOCKS_PER_SEC) << endl;
            //cout << endl, endl;
                
            if(((clock() - TIME[x].cont)/CLOCKS_PER_SEC) >= TempoLimite){
                cout << "o robo Azul de ID " << TIME[x].ID_ROBOT << " deu perda" << endl;
                TIME[x].Robo_Ativo = 1;
            }
        }

    

}
//.......................................

// printar as informações do robo
void printRobotInfo(Robos robot, int has_orientation_valid) {
    printf("CONF=%4.2f ", robot.confidence);
    printf("ID=%3d ",robot.ID_ROBOT);
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height,robot.x_novo,robot.y_novo);
    if (has_orientation_valid == 1) {
        printf("ANGLE=%6.3f ",robot.orientation());
    }
    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x,robot.pixel_y);
}
//...................................................

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    GrSim_Client grSim_client;

    //Declarando os parametros utilizados nas manipulações de dados
    Cont_Indice_blue=0;
    Cont_Indice_yellow=0;
    Robos Blue[8];
    Robos Yellow[8];
    //................

    while(true) {
        if (client.receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
                double t_now = GetTimeSec();

                printf("-[Detection Data]-------\n");
                //Frame info:
                printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n",detection.camera_id(),detection.frame_number(),detection.t_capture());

                printf("SSL-Vision Processing Latency                   %7.3fms\n",(detection.t_sent()-detection.t_capture())*1000.0);
                printf("Network Latency (assuming synched system clock) %7.3fms\n",(t_now-detection.t_sent())*1000.0);
                printf("Total Latency   (assuming synched system clock) %7.3fms\n",(t_now-detection.t_capture())*1000.0);
                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();

                //Ball info:
                for (int i = 0; i < balls_n; i++) {
                    SSL_DetectionBall ball = detection.balls(i);
                    printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i+1, balls_n, ball.confidence(),ball.x(),ball.y());
                    if (ball.has_z()) {
                        printf("Z=%7.2f ",ball.z());
                    } else {
                        printf("Z=N/A   ");
                    }
                    printf("RAW=<%8.2f,%8.2f>\n",ball.pixel_x(),ball.pixel_y());
                }

                //Blue robot info:

                //loop de verificações e manipulação robos azuis
                for (int i = 0; i < robots_blue_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    if (robot.has_robot_id()) {
                        NewID=0; // NewID = 0 -> Novo ID

                        for(int J=0; J <Cont_Indice_blue; J++){
                            if(Blue[J].ID_ROBOT == robot.robot_id()){
                                Blue[J].cont= clock();
                                Blue[J].Robo_Ativo = 0;
                                Blue[J].x_novo = robot.x();
                                Blue[J].y_novo = robot.y();
                                Blue[J].pixel_x = robot.pixel_x();
                                Blue[J].pixel_y= robot.pixel_y();
                                Blue[J].height = robot.height();
                                Blue[J].confidence = robot.confidence();
                                Blue[J].vx = (Blue[J].x_novo - Blue[J].x)/dt
                                Blue[J].vy = (Blue[J].y_novo - Blue[J].y)/dt
                                Blue[J].Identificado = true;
                                Blue[J].Atualizado = true;
                                NewID=1;// NewID = 1 -> ID Já identificado  
                                if(robot.has_orietation()){
                                    Blue[J].orientation = robot.orientation();
                                    Blue[J].has_orietation = true;
                                } 
                                else{
                                    Blue[J].has_orietation = false;
                                }    
                            }
                        // registrar novo ID 
                            if(NewID==0){
                                Blue[Cont_Indice_blue].ID_ROBOT = robot.robot_id();
                                Blue[Cont_Indice_blue].cont = clock();
                                Blue[Cont_Indice_blue].x_novo = robot.x();
                                Blue[Cont_Indice_blue].y_novo = robot.y();
                                Blue[Cont_Indice_blue].pixel_x = robot.pixel_x();
                                Blue[Cont_Indice_blue].pixel_y = robot.pixel_y();
                                Blue[Cont_Indice_blue].height = robot.height();
                                Blue[Cont_Indice_blue].confidence = robot.confidence();
                                Blue[Cont_Indice_blue].vx = 0.0;
                                Blue[Cont_Indice_blue].vy = 0.0;
                                if(robot.has_orietation()){
                                    Blue[Cont_Indice_blue].orientation = robot.orientation();
                                    Blue[Cont_Indice_blue].has_orietation = true;
                                }
                                else{
                                    Blue[Cont_Indice_blue].has_orietation = false;
                                } 
                                Cont_Indice_blue++;
                            }
                        
                            if(robot.x() <= 0){
                                grSim_client.sendCommand(1.0, i);
                            }else{
                                grSim_client.sendCommand(-1.0, i);
                            }
                        }

                        printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);

                    }
                }
                // definir robos a serem mostrados em tela
                for(int J=0; J < Cont_Indice_blue; J++){
                    if(Blue[J].Robo_Ativo == 0){
                        // Definir se foi recebido a orientação
                        int has_orientation_valid = 0;
                        if(Blue[J].has_orietation == true){
                            has_orientation_valid = 1;
                        }
                        //
                        if(Blue[J].Identificado == true){
                            kalman_filter(Blue[J])//argumentos
                        }
                        printRobotInfo(Blue[J], has_orientation_valid);
                       
                    }
                }
                PERDA(Cont_Indice_blue, Blue)//completar Argumentos
                //....................................................

                //Yellow robot info:

                //loop de verificações e manipulação robos amarelos
                for (int i = 0; i < robots_yellow_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    if (robot.has_robot_id()) {
                        NewID=0; // NewID = 0 -> Novo ID

                        for(int J=0; J <Cont_Indice_yellow; J++){
                            if(Yellow[J].ID_ROBOT == robot.robot_id()){
                                Yellow[J].cont= clock();
                                Yellow[J].Robo_Ativo = 0;
                                Yellow[J].x_novo = robot.x();
                                Yellow[J].y_novo = robot.y();
                                Yellow[J].pixel_x = robot.pixel_x();
                                Yellow[J].pixel_y= robot.pixel_y();
                                Yellow[J].orientation = robot.orientation();
                                Yellow[J].height = robot.height();
                                Yellow[J].confidence = robot.confidence();
                                Yellow[J].dx = (Yellow[J].x_novo - Yellow[J].x)/dt
                                Yellow[J].dy = (Yellow[J].y_novo - Yellow[J].y)/dt
                                Yellow[J].Identificado = true;
                                Yellow[J].Atualizado = true;
                                Yellow
                                NewID=1;// NewID = 1 -> ID Já identificado  
                                if(robot.has_orietation()){
                                   Yellow[J].orientation = robot.orientation();
                                   Yellow[J].has_orietation = true;
                                } 
                                else{
                                    Yellow[J].has_orietation = false;
                                }      
                            }
                        // registrar novo ID 
                            if(NewID==0){
                                Yellow[Cont_Indice_yellow].ID_ROBOT = robot.robot_id();
                                Yellow[Cont_Indice_yellow].cont = clock();
                                Yellow[Cont_Indice_yellow].x = robot.x();
                                Yellow[Cont_Indice_yellow].y = robot.y();
                                Yellow[Cont_Indice_yellow].pixel_x = robot.pixel_x();
                                Yellow[Cont_Indice_yellow].pixel_y = robot.pixel_y();
                                Yellow[Cont_Indice_yellow].orientation = robot.orientation();
                                Yellow[Cont_Indice_yellow].height = robot.height();
                                Yellow[Cont_Indice_yellow].confidence = robot.confidence();
                                Yellow[Cont_Indice_yellow].dx = 0.0;
                                Yellow[Cont_Indice_yellow].dy = 0.0;
                                if(robot.has_orietation()){
                                   Yellow[J].orientation = robot.orientation();
                                   Yellow[J].has_orietation = true;
                                } 
                                else{
                                    Yellow[J].has_orietation = false;
                                } 
                                Cont_Indice_yellow++;
                            }
                        }

                        printf("-Robot(B) (%2d/%2d): ",i+1, robots_yellow_n);

                    }
                }
                // definir robos a serem mostrados em tela
                for(int J=0; J < Cont_Indice_blue; J++){
                    if(Yellow[J].Robo_Ativo == 0){
                        // Definir se foi recebido a orientação
                        int has_orientation_valid = 0;
                        if(Yellow[J].has_orietation == true){
                            has_orientation_valid = 1;
                        }
                        //
                        if(Yellow[J].Identificado == true){
                            kalman_filter(Yellow[J])//argumentos
                        }
                        printRobotInfo(Yellow[J], has_orientation_valid);
                       
                    }
                }
                PERDA(Cont_Indice_yellow, Yellow)//completar Argumentos
                //....................................................

            //see if packet contains geometry data:
            if (packet.has_geometry()) {
                const SSL_GeometryData & geom = packet.geometry();
                printf("-[Geometry Data]-------\n");

                const SSL_GeometryFieldSize & field = geom.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%d (mm)\n",field.field_length());
                printf("  -field_width=%d (mm)\n",field.field_width());
                printf("  -boundary_width=%d (mm)\n",field.boundary_width());
                printf("  -goal_width=%d (mm)\n",field.goal_width());
                printf("  -goal_depth=%d (mm)\n",field.goal_depth());
                printf("  -field_lines_size=%d\n",field.field_lines_size());
                printf("  -field_arcs_size=%d\n",field.field_arcs_size());

                int calib_n = geom.calib_size();
                for (int i=0; i< calib_n; i++) {
                    const SSL_GeometryCameraCalibration & calib = geom.calib(i);
                    printf("Camera Geometry for Camera ID %d:\n", calib.camera_id());
                    printf("  -focal_length=%.2f\n",calib.focal_length());
                    printf("  -principal_point_x=%.2f\n",calib.principal_point_x());
                    printf("  -principal_point_y=%.2f\n",calib.principal_point_y());
                    printf("  -distortion=%.2f\n",calib.distortion());
                    printf("  -q0=%.2f\n",calib.q0());
                    printf("  -q1=%.2f\n",calib.q1());
                    printf("  -q2=%.2f\n",calib.q2());
                    printf("  -q3=%.2f\n",calib.q3());
                    printf("  -tx=%.2f\n",calib.tx());
                    printf("  -ty=%.2f\n",calib.ty());
                    printf("  -tz=%.2f\n",calib.tz());

                    if (calib.has_derived_camera_world_tx() && calib.has_derived_camera_world_ty() && calib.has_derived_camera_world_tz()) {
                      printf("  -derived_camera_world_tx=%.f\n",calib.derived_camera_world_tx());
                      printf("  -derived_camera_world_ty=%.f\n",calib.derived_camera_world_ty());
                      printf("  -derived_camera_world_tz=%.f\n",calib.derived_camera_world_tz());
                    }

                }
            }
        }
    }

    return 0;
}