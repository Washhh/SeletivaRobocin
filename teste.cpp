#include<bits/stdc++.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<unistd.h>
#include<string.h>
#include<stdlib.h>
#include<stdio.h>
#include<termios.h>
#include<thread>
#include<chrono>
#include<time.h>

using namespace std;

#define tempoMaximo 0.1

int getch(){
	struct termios oldt, newt;
	int ch;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

typedef struct{
    int x;
    int y;
    double vx, vy, w;
    double contador;
} robot;

robot vetor[8];



int main(){
    char c;

    for(int x = 0; x < 8; x++) vetor[x].contador = clock();

    while(true){
        c = getch();
        if(c >= '0' && c <= '7'){

            printf("Atualizei o tempo de : %c\n", c);

            vetor[c - '0'].contador = clock(); 

            for(int x = 0; x < 8; x++){
                
                cout << "tempo de " << x << " : " << ((clock() - vetor[x].contador)/CLOCKS_PER_SEC) << endl;
                cout << endl, endl;
                
                if(((clock() - vetor[x].contador)/CLOCKS_PER_SEC) >= tempoMaximo){
                    cout << "o robo " << x << " deu perda" << endl;
                }
            }
        }
    }
}