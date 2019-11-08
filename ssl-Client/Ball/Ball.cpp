#include "Ball.h"
#define TempoLimite 0.3

Ball::Ball(SSL_DetectionBall &ball){ // Construtor
    
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

    this.Area_Bola = ball.area();
    this.cont = clock();
    this.x = ball.x();
    this.y = ball.y();
    this.x_novo = ball.x();
    this.y_novo = ball.y();
    this.pixel_x = ball.pixel_x();
    this.pixel_y = ball.pixel_y();
    this.confidence = ball.confidence();
    this.vx = 0.0;
    this.vy = 0.0;
    if(ball.has_z()){
        this.z = ball.z();
        this.has_z = true;
    }
    else{
        this.has_z = false;
    } 
}
void Ball::set_ball(SSL_DetectionBall &ball){ // Método que passa para a bola as informações recebidas 
    if(this.Bola_Ativa == 1){// se o robo estava inativo , reativa ele e para garantir que não é só um ruido, passa o filtro de ruido novamente
        this.Bola_Ativa = 0;
        this.Valido = false;
        Ruido.Contador();
        this.P <<
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0;

        this.x = ball.x();
        this.y = ball.y();
    }
    this.cont= clock();
    this.x_novo = ball.x();
    this.y_novo = ball.y();
    this.pixel_x = ball.pixel_x();
    this.pixel_y= ball.pixel_y();
    this.confidence = ball.confidence();
    this.vx = (this.x_novo - this.x)/this.dt;
    this.vy = (this.y_novo - this.y)/this.dt;
    this.Atualizado = true;
    if(ball.has_z()){
        this.z = ball.z();
        this.has_z = true;
    } 
    else{
        this.has_z = false;
    }
}
bool Ball::Verificar(SSL_DetectionBall &ball){ // Método para descobrir se a área recebida é a da bola verificado
    
    if(Ball_Area(ball.area())){
        set_ball(ball);
        return true;
    }
    return false;
}

bool Ball::Ball_Area(int Area){ // Método que retorna a verificação de igualdade entre a área da bola e a área recebida
    double aux = this.Area_Bola/Area;
    if(aux >= 0.95 && aux <= 1.05 ){
        return true;
    }

    return false;
}

bool Ball::get_Atualizado(){ // Método para retorno do estado da bola, se atualizado ou não
    return this.Atualizado;
}

bool Ball::get_Ativo(){ // Método para retorno do estado da bola, se ativo ou não
    if(this.Bola_Ativa == 0){
        return true;
    }
    return false;
}

bool Ball::get_Valido(){ // Método para retorno do estado da bola, se valido ou não
    return this.Valido;
}

float Ball::getx(){ // Método para retorno da posição x da bola
    return this.x_novo;
}

float Ball::gety(){ // Método para retorno da posição y da bola
    return this.y_novo;
}
void Ball::set_off_Ativo(){// Método para deixar a bola inativa
    this.Bola_Ativa = 1;
}
void Ball::kalman(){ // Método para execução do lindo e maravilho filtro de kalman

    Eigen::MatrixXd I
    I.setIdentity();
    Eigen::VectorXd X(this.n);// Matriz 4x1
    Eigen::VectorXd Y(this.m);// Matriz 2x1
    X << 
    this.x, 
    this.y, 
    this.vx, 
    this.vy;
    Y << 
    this.x_novo, 
    this.y_novo;

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

    this.x_novo = Saida(1,1); // Posição X definida como a atual
    this.y_novo = Saida(2,1); // Posição Y definida como a atual
    this.x = X_NEW(1,1); // Posição X estimada
    this.y = X_NEW(2,1); // Posição Y estimada
    this.Atualizado = false; // garantir que ele só execute correção na próxima vez caso seja entregue seu ponto x,y
}

void Ball::Perda(){ // Método para a execução do filtro de Perda que define se uma bola está ou não no tempo limite de verificação, passando desse tempo limite, ela é setada como inválida
                
    if(((double)(clock() - this.cont)/CLOCKS_PER_SEC) >= TempoLimite){
        cout << "A bola de Area " << this.Area_Bola << " deu perda" << endl;
        this.Bola_Ativa = 1;
    }
        
}


void Ball::printBallInfo() {// printar as informações da bola
    printf("CONF=%4.2f ", this.confidence);
    printf("ID=%3d ",this.Area_Bola);
    printf("POS=<%9.2f,%9.2f> ",this.x_novo,this.y_novo);
    if (this.has_z) {
        printf("ANGLE=%6.3f ",this.z);
    }
    printf("RAW=<%8.2f,%8.2f>\n",this.pixel_x,this.pixel_y);
}
void Ball::Iniciar_Ruido(){ // Método que Inicia o filtro de Ruido
    Ruido = new Ruido(); 
}
void Ball::Filtro_Ruido(){ // Método que retorna se o filtro terminou ou não 
    if(Ruido.Filtro_Ruido()){
        this.Valido = true;  
    }
}

void Ball::SET_OFF_RUIDO(){ // Método que para a execução do filtro o sentando como não inicializado
    Ruido.Filtro_OFF();
}
bool Ball::Ruido_Inicializado(){ // Método que retorna o estado do filtro, inicializado ou não
    return Ruido.Inicializado();
}