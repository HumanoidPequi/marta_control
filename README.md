## martha_control

Este repositório fornece um ambiente de desenvolvimento para controle do robô humanoide Martha, incluindo scripts, bibliotecas e configurações otimizadas para a execução de algoritmos de controle e manipulação.

## Índice

- [Controle da Martha](#martha_control)
  - [Índice](#Índice)
  - [Introdução](#Introdução)
  - [Estrutura do Repositório](#estrutura-do-repositório)
  - [Requisitos](#requisitos)
  - [Getting Started](#getting-started)
    - [Clonando o Repositório e Configurando o Workspace nativo](#clonando-o-repositório-e-configurando-o-workspace-nativo)
    - [Clonando o Repositório e Configurando o Workspace via docker](#clonando-o-repositório-e-configurando-o-workspace-via-docker)
  - [Executando o Controle](#executando-o-controle)
  - [Contribuindo](#Contribuindo)

## Introdução
Este projeto foi desenvolvido para fornecer um ambiente completo de controle para o robô Martha utilizando ROS Noetic. O ambiente inclui scripts de controle e bibliotecas necessárias, facilitando o desenvolvimento e a execução de algoritmos de controle.

## Estrutura do Repositório

- **docker/**: Contém o Dockerfile para configurar o ambiente via Docker
- **Martha_control_ws/**: Workspace contendo os pacotes de controle do robô Martha

## Requisitos

- Docker (opcional)
- Ubuntu 20.04
- Ros Noetic
- ros_control
- ros_controllers
- hardware_interface
- martha_simulation

## Getting Started

### Clonando o Repositório e Configurando o Workspace nativo

Para clonar o Workspace execute:
`git clone git@github.com:HumanoidPequi/martha_control.git`

Após a compilação, execute o source do workspace:

```
//dentro de Martha_ws 
source devel/setup.bash
```

### Clonando o Repositório e Configurando o Workspace via docker

Dentro da raiz do repositório, execute o build da imagem docker:

`docker build -t martha_control:1.0 -f docker/Dockerfile .`

Dentro da imagem docker, siga as instruções para rodar o controle do robô.

## Executando o Controle

Para executar o controle, tanto no ambiente docker quanto nativo, utilize o seguinte comando:

`roslaunch <modulo de controle> <launcher do controle>`

Este comando inicializa os controladores e configura as interfaces de hardware ou simulação para o robô Martha.

## Contribuindo

Contribuições são bem-vindas! Por favor, envie um pull request ou abra uma issue se você encontrar problemas ou tiver sugestões para melhorias.
