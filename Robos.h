#ifndef ROBOS_H
#define ROBOS_H
#include "Bibliotecas.h"
#include "Ruido.h"

class Robos{
    public:
        Robos(SSL_DetectionRobot &robot);
        void set_robot(SSL_DetectionRobot &robot);
        bool Verificar(SSL_DetectionRobot &robot);
        bool Robot_ID(int ID);
        bool ATUALIZADO();
        bool ATIVO();
        bool VALIDO();
        float getx();
        void kalman();
        void Perda();
        void printRobotInfo();
        void Iniciar_Ruido();
        void Filtro_Ruido();
        void SET_OFF_RUIDO();
        bool Ruido_Inicializado();
       

    private:
        std::Vector<Ruido> Ruido;
        int n = 4; // Number of states
        int m = 2; // Number of measurements
        Eigen::MatrixXd A(n, n); // System dynamics matrix
        Eigen::MatrixXd C(m, n); // Output matrix
        Eigen::MatrixXd Q(n, n); // Process noise covariance
        Eigen::MatrixXd R(m, m); // Measurement noise covariance
        Eigen::MatrixXd P(n, n); // Estimate error covariance
        bool Atualizado      = false;
        bool has_orietation  = false;
        bool Valido          = false;
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
#endif