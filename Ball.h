#ifndef BALL_H
#define BALL_H
#include "Robos.h"
class Ball{
    public:
        Ball(SSL_Detectionball &ball); // construtor
        void set_ball(SSL_Detectionball &ball);
        bool Verificar(SSL_Detectionball &ball);
        bool Ball_Area(int Area);
        bool get_Atualizado();
        bool get_Ativo();
        bool get_Valido();
        float getx();
        float gety();
        void kalman();
        void Perda();
        void printBallInfo();
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
        bool has_z           = false;
        bool Valido          = false;
        double dt            = 1.0/60;
        int Area_Bola        = -1; // Area da Bola
        int Bola_Ativa       = 0; // Variável para definir se o Bola está dentro do tempo limite de perda
        double vx; // componente x da velocidade da Bola
        double vy; // componente y da velocidade da Bola
        float x; // posição x prevista para a Bola
        float y; // posição y prevista para a Bola
        float x_novo; // posição x recebida pelo pacote, usada para corrigir a posição prevista do kalman
        float y_novo; // posição y recebida pelo pacote, usada para corrigir a posição prevista do kalman
        float pixel_x; // pixel x recebido
        float pixel_y; // pixel y recebido
        float z; // z recebido
        float confidence; 
}
#endif