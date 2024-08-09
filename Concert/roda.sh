#!/bin/bash

# Array de valores para alpha
alphas=(0.2 0.4 0.6 0.8)

# Array de valores para custo
custos=("50 5 1" "100 10 2" "150 15 3")

# Array de valores para revenue
revenues=(2000 1500 1000)

# Caminho do arquivo de entrada
input_file="cab.txt"

# Loop aninhado para gerar todas as combinações
for revenue in "${revenues[@]}"; do
    for custo in "${custos[@]}"; do
        for alpha in "${alphas[@]}"; do
            # Executa o programa com os parâmetros atuais
            ./condireta $input_file $alpha $revenue $custo
        done
    done
done

# Loop aninhado para gerar todas as combinações
for revenue in "${revenues[@]}"; do
    for custo in "${custos[@]}"; do
        for alpha in "${alphas[@]}"; do
            # Executa o programa com os parâmetros atuais
            ./ncondireta $input_file $alpha $revenue $custo
        done
    done
done
