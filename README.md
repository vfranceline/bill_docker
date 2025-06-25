# bill\_docker

Este repositório contém a configuração e os pacotes ROS 2 necessários para simular e controlar o robô Bill em um ambiente de desenvolvimento em contêiner.

## Visão Geral

O projeto utiliza Docker para encapsular o ambiente ROS 2 e todas as suas dependências, facilitando a replicação e a execução em diferentes máquinas. O robô Bill é um robô móvel equipado com um Lidar, e os pacotes fornecidos permitem a sua visualização, simulação no Gazebo, localização, mapeamento e navegação.

## Estrutura do Repositório

O repositório está organizado da seguinte forma:

```
.
├── dependencies
│   ├── conda_environment.yml
│   └── requirements_apt.txt
├── Dockerfile
├── build_image.sh
├── container.sh
├── install_dependencies.sh
└── nav_ws
    ├── src
    │   ├── bill
    │   ├── bill_description
    │   ├── bill_gazebo
    │   ├── bill_navigation
    │   ├── rf2o_laser_odometry
    │   └── tasks
    ├── ...
```

### Pacotes ROS 2

O workspace do ROS 2 (`nav_ws/src`) contém os seguintes pacotes:

  * **bill**: Pacote principal que pode ser usado para agregar dependências dos outros pacotes do projeto.
  * **bill\_description**: Contém a descrição URDF do robô Bill, incluindo seus links, juntas e meshes visuais.
  * **bill\_gazebo**: Contém os modelos e mundos do Gazebo para a simulação do robô.
  * **bill\_navigation**: Fornece as configurações e os arquivos de inicialização para a navegação, localização (AMCL) e mapeamento (SLAM Toolbox).
  * **rf2o\_laser\_odometry**: Um pacote para estimar a odometria 2D com base em varreduras de laser planas.
  * **tasks**: Contém scripts para tarefas de alto nível, como navegação para um ponto e desvio de obstáculos.

## Pré-requisitos

  * Docker
  * Um sistema operacional baseado em Linux

## Instalação

1.  **Clone o repositório:**

    ```bash
    git clone https://github.com/vfranceline/bill_docker.git
    cd bill_docker
    ```

2.  **Construa a imagem Docker:**
    O script `build_image.sh` automatiza a construção da imagem Docker.

    ```bash
    ./build_image.sh
    ```

## Como Usar

### Iniciando o Contêiner

O script `container.sh` é usado para iniciar e gerenciar o contêiner Docker. Ele irá criar um novo contêiner se nenhum existir, iniciar um contêiner parado ou conectar-se a um contêiner em execução.

```bash
./container.sh
```

Dentro do contêiner, o workspace `nav_ws` estará montado.

### Instale as dependências do sistema dentro do docker
    O script `install_dependencies.sh` pode ser usado para instalar as dependências de apt listadas em `dependencies/requirements_apt.txt`.

    ```bash
    cd nav_ws/src
    ./install_dependencies.sh
    ```

### Compilando o Workspace

Dentro do contêiner, compile o workspace do ROS 2:

```bash
cd /nav_ws
colcon build
```

### Executando as Simulações e Ferramentas

Após compilar o workspace, certifique-se de fazer o source do setup:

```bash
source /nav_ws/install/setup.bash
```

Aqui estão alguns dos principais comandos de inicialização:

  * **Visualizar o Robô no RViz:**
    Este comando inicia o RViz e exibe o modelo URDF do robô Bill.

    ```bash
    ros2 launch bill_description check_urdf.launch.py
    ```

  * **Iniciar a Simulação no Gazebo:**
    Este comando inicia o Gazebo com um mundo e spawna o robô Bill.

    ```bash
    ros2 launch bill_navigation spawn_robot.launch.py
    ```

  * **Mapeamento com SLAM Toolbox:**
    Para iniciar o processo de mapeamento, use o seguinte comando:

    ```bash
    ros2 launch bill_navigation mapping.launch.py
    ```

    Você pode então usar o teleop para mover o robô e construir o mapa.

  * **Localização com AMCL:**
    Para localizar o robô em um mapa pré-existente, use:

    ```bash
    ros2 launch bill_navigation localization.launch.py
    ```

  * **Navegação:**
    Para iniciar a navegação autônoma com o Nav2, use:

    ```bash
    ros2 launch bill_navigation navigation.launch.py
    ```

    Isso permitirá que você envie metas de pose para o robô.

  * **Executar Tarefas:**
    O pacote `tasks` contém scripts que podem ser executados para realizar ações específicas:

      * **Enviar uma meta de pose:**
        ```bash
        ros2 run tasks goal_pose
        ```
      * **Inspeção com desvio de obstáculos:**
        ```bash
        ros2 run tasks inspecao
        ```

## Configuração

Os arquivos de configuração para os pacotes de navegação podem ser encontrados em `nav_ws/src/bill_navigation/config`:

  * `amcl_localization.yaml`: Parâmetros para o nó de localização AMCL.
  * `ekf.yaml`: Configuração para o Extended Kalman Filter usado na localização do robô.
  * `navigation.yaml`: Parâmetros para o stack de navegação Nav2.
  * `slam_toolbox_mapping.yaml`: Parâmetros para o SLAM Toolbox.
