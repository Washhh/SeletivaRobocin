#ifndef BALL_H
#define BALL_H
#include "../Bibliotecas.h"
#include "../Ruido/Ruido.h"

class Ball{
    public:
        Ball(SSL_Detectionball &ball); // construtor
        void set_ball(SSL_Detectionball &ball); // Atualiza as informações de uma bola detectada
        bool Verificar(SSL_Detectionball &ball); // Seta as informações atualizadas na bola cuja área seja compatível com a área recebida
        bool Ball_Area(int Area);// Retorna se a área recebida é compatível com a bola 
        bool get_Atualizado(); // Retorna o estado do boleano Atualizado
        bool get_Ativo(); // Retorna o estado do boleano Ativo
        bool get_Valido();  // Retorna o estado do boleano Valido
        float getx(); // Retorna o valor de x
        float gety(); // Retorna o valor de y
        void kalman(); // Executa o filtro de kalman com as coordenadas da bola
        void Perda(); // Executa o filtro de perda para verificar se a bola está "sumida" a muito tempo e desativa caso afirmativo
        void printBallInfo(); //Mostra as informações da bola
        void Iniciar_Ruido(); // Inicia o filtro de Ruido para verificar se a bola é válida
        void Filtro_Ruido(); // Seta o valor de Valido dependendo se o filtro acabou de executar e a bola se manteve visível, assim ela é dita como válida
        void SET_OFF_RUIDO(); // Se a bola foi perdida enquanto o filtro de Ruido executava, seta para NÃO INICIALIZADO para que caso seja encontrado novamente seja setado novamente para inicializado
        bool Ruido_Inicializado(); // Retorna se o filtro de Ruido está


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
        double cont          = 0.0; // Contador para o filtro de perda
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

};
#endif