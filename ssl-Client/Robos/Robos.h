#ifndef ROBOS_H
#define ROBOS_H
#include "../Bibliotecas.h"
#include "../Ruido/Ruido.h"

class Robos{
    public:
        Robos(SSL_DetectionRobot &robot); // construtor
        void set_robot(SSL_DetectionRobot &robot); // Atualiza as informações de um robô existente
        bool Verificar(SSL_DetectionRobot &robot); // Seta as informações atualizadas no robô cujo ID seja o mesmo recebido
        bool Robot_ID(int ID);// Retorna se o ID recebido é o ID do robô já encontrado
        bool get_Atualizado(); // Retorna o boleano do elemento Atualizado
        bool get_Ativo(); // Retorna o boleano do elemento Robo_Ativo
        bool get_Valido(); // Retorna o boleano do elemento Valido
        float getx(); // Retorna o valor de x
        float gety(); // Retorna o valor de y
        void set_off_Ativo(); // Seta o robô para inativo
        void kalman();// Executa kalman com as coordenadas do robo
        void Perda(); // Executa Perda para verificar se o robô está " sumido " a muito tempo e o desativa caso afirmativo
        void printRobotInfo(); // Mostra as informações do Robo
        void Iniciar_Ruido(); // Inicia o filtro de Ruido para verificar se o robô é válido
        void Filtro_Ruido(); // Seta o valor de Valido dependendo se o filtro acabou de executar e o robô se manteve visível, assim ele é dito como Valido
        void SET_OFF_RUIDO(); // Se o robô foi perdido enquanto o Ruido é executado, seta para não inicializado para que caso seja encontrado novamente seja setado novamente para inicializado
        bool Ruido_Inicializado(); // Retorna se o Filtro de Ruido está sendo executado
       

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
        double cont          = 0.0; // Contador para o filtro de perda
        int ID_ROBOT         = -1; // ID DO Robô 
        int Robo_Ativo       = 0; // Variável para definir se o Robô está dentro do tempo limite de perda
        double vx; // componente x da velocidade do Robô 
        double vy; // componente y da velocidade do Robô 
        float x; // posição x prevista para o Robô 
        float y; // posição y prevista para o Robô 
        float x_novo; // posição x recebida pelo pacote, usada para corrigir a posição prevista do kalman
        float y_novo; // posição y recebida pelo pacote, usada para corrigir a posição prevista do kalman
        float pixel_x; // pixel x recebido
        float pixel_y; // pixel y recebido
        float height; // altura recebida
        float orientation; // orientação recebida
        float confidence; 

};
#endif