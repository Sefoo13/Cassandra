FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    wget curl git python3 \
    pulseaudio-utils \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN wget https://github.com/rhasspy/piper/releases/latest/download/piper_linux_aarch64.tar.gz \
    && tar -xzf piper_linux_aarch64.tar.gz \
    && rm piper_linux_aarch64.tar.gz

WORKDIR /app/piper

COPY piper_server.py /app/piper_server.py

EXPOSE 8000

ENTRYPOINT ["python3", "/app/piper_server.py"]
