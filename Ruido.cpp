#include "Ruido.h"
#include <stdio.h>

Ruido::Ruido(){
    tempo = 200.0;
    Contador();
}
void Ruido::Contador(){
    this.temporizador = clock();
    this. filtro_inicializado = true;
}

void Ruido::Filtro_OFF(){
    filtro_inicializado = false;
}
bool Ruido::Inicializado(){
    return filtro_inicializado;
}
bool Filtro_Ruido(){
    if()
    if(((double)(clock()-temporizador)/100.0) >= tempo){
        return true;
    }
    return false;
}