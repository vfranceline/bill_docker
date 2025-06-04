# Utiliza a imagem oficial do ROS 2 Jazzy com suporte completo ao desktop e Gazebo
FROM osrf/ros:jazzy-desktop-full

# Define variáveis de ambiente
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Atualiza o sistema e instala dependências adicionais
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    git \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Configura o locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Adiciona o ROS 2 ao ambiente
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Cria um diretório de trabalho
WORKDIR /root/ros2_ws

# Comando padrão ao iniciar o contêiner
CMD ["/bin/bash"]

