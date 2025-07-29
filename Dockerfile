# Usa a imagem oficial do ROS 2 Jazzy com suporte ao desktop e Gazebo
FROM osrf/ros:jazzy-desktop-full

# Usa o shell bash
SHELL ["/bin/bash", "-c"]

# Variáveis de ambiente
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Lista de pacotes APT
ENV APT_PACKAGES="\
    build-essential \
    cmake \
    usbutils \
    libopencv-dev \
    libboost-all-dev \
    libopenblas-dev \
    liblapack-dev \
    libx11-dev \
    libgtk-3-dev \
    libstdc++6 \
    portaudio19-dev \
    mpg123 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libusb-1.0-0-dev \
    libgstreamer-plugins-base1.0-dev \
    ffmpeg \
    libx264-dev \
    libsndfile1-dev \
    python3-pip \
    alsa-utils \
    pulseaudio \
    python3-gi \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    git \
    wget \
"

# Instala os pacotes
RUN apt-get update && \
    apt-get install -y --no-install-recommends ${APT_PACKAGES} && \
    rm -rf /var/lib/apt/lists/*

# Configura o locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Adiciona o ROS 2 ao ambiente bash padrão
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Cria e define o diretório de trabalho
WORKDIR /bill_nav

# Copia os arquivos do projeto para dentro da imagem
COPY ./bill_nav /bill_nav

# Adiciona o usuário root ao grupo dialout (permite acesso a /dev/tty*)
RUN usermod -a -G dialout root
