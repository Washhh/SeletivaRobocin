#ifndef RUIDO_H
#define RUIDO_H
#include <time.h>

class Ruido{
    public:
        Ruido();
        void Contador(); // Inicia os valores do filtro
        void Filtro_OFF(); // Seta o filtro para não inicializado
        bool Inicializado(); // retorna o estado atual do filtro, se ele foi iniciado ou não
        bool Filtro_Ruido(); // executa o verificação do filtro para saber se ele já terminou ou não
       
    private:
        double tempo; // tempo em que o valor do clock - temporizador tem que sair mair ou igual
        double temporizador; // variável que recebe o valor do clock no momento em que o filtro começa
        bool filtro_inicializado = false; // boelado para saber se o filtro está ou não sendo executado
};
#endif