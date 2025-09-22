#include <iostream>

int main() {
    int altura = 5; // Altura del triángulo

    for (int i = 1; i <= altura; i++) {
        // Imprimir espacios
        for (int j = 0; j < altura - i; j++) {
            std::cout << " ";
        }
        // Imprimir asteriscos
        for (int k = 0; k < (2 * i - 1); k++) {
            std::cout << "*";
        }
        std::cout << std::endl;
    }

    return 0;
}