#ifndef RUIDO_H
#define RUIDO_H
#include <time.h>

class Ruido{
    public:
        Ruido();
        void Contador();
        void Filtro_OFF();
        bool Inicializado();
        bool Filtro_Ruido();
       
    private:
        double tempo;
        double temporizador;
        bool filtro_inicializado = false;
}