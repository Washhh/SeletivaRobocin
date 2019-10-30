#include "Ruido.h"
#include <stdio.h>

Ruido::Ruido(){ // Montador do filtro de Ruido
    tempo = 200.0; 
    Contador();
}
void Ruido::Contador(){ // Método usado para iniciar as variáveis necessárias para execução e verificação
    this.temporizador = clock(); 
    this. filtro_inicializado = true;
}

void Ruido::Filtro_OFF(){ // Método usado para cancelar a verificação do filtro, pois ele não vai mais está inicializado
    filtro_inicializado = false;
}
bool Ruido::Inicializado(){ // Método que retorna o estado do filtro, iniciado ou não
    return filtro_inicializado;
}
bool Filtro_Ruido(){ // Método onde é verificado se o filtro acabou ou não

    if(((double)(clock()-temporizador)/100.0) >= tempo){
        return true;
    }
    return false;
}