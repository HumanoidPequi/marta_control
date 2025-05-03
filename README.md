# MARTA Control System

Este repositório fornece um ambiente de desenvolvimento para controle da Marta, incluindo scripts, bibliotecas e configurações otimizadas para a execução de algoritmos de controle e manipulação.

## Estrutura do Repositório

O repositório está organizado em módulos funcionais:

### common
Utilitários e funcionalidades comuns compartilhadas entre módulos:
- Operações matemáticas e vetoriais
- Interpolação linear
- Parâmetros físicos do robô

### kinematics
Implementação de cinemática inversa para o robô:
- Cinemática inversa das pernas
- Cinemática inversa dos braços
- Gerenciamento de juntas

### walking
Implementação da caminhada usando ZMP (Zero Moment Point):
- Caminhada omnidirecional
- Controlador de pernas
- Cálculos de caminhada

### actions
Ações especiais do robô:
- **kick**: Ações de chute
- **getup**: Comportamentos para levantar-se

### msgs
Definições de mensagens e ações personalizadas:
- Ação de levantamento (Raise.action)
- Outras mensagens personalizadas

## Requisitos

- Ubuntu 20.04
- ROS Noetic
- ros_control
- ros_controllers
- hardware_interface

## Contribuindo

1. Respeite a estrutura modular:
   - Utilidades comuns vão em `common`
   - Cinemática vai em `kinematics`
   - Caminhada vai em `walking`
   - Ações especiais vão em `actions`
   - Definições de mensagens vão em `msgs`

2. Mantenha dependências adequadas:
   - Todos os módulos dependem de `common`
   - `walking` depende de `kinematics`
   - `actions` depende de `kinematics`

3. Utilize os arquivos de lançamento:
   - Cada funcionalidade tem seu próprio arquivo de lançamento
   - Parâmetros são carregados de arquivos de parâmetros na pasta config