# RUNBOOK: teste peer-to-peer MQTT no hardware real

Barco: Raspberry Pi 4 (imagem Emlid p/ Navio2) + ZED + Bullet M2.
Estação: t620-headnode + Rocket M2 (na NIC USB `enp0s26u1u6u3`).
Tudo desta pasta (`mqtt/`); nada fora dela precisa mudar.

## 0. Estado conhecido (2026-08-19)

- LAN do cluster t620 = `10.10.10.0/24` via `bond0` (eno1+eno2). NÃO usar.
- A Rocket fica na NIC USB `enp0s26u1u6u3` do t620 (link físico presente).
- 2026-08-19: Rocket **sem responder a nada** (45 s de captura promíscua em
  branco, sem ARP em 192.168.1.20, discovery UDP 10001 mudo) → quase certo
  injetor PoE desligado. Ligar antes de qualquer passo abaixo.
- O t620 já roda um mosquitto do swarm (`digitaltwin_mqtt`, porta 1883).
  Por isso a estação NÃO sobe broker: subscriber e probe são clientes do
  broker do Pi.

## 1. Plano de endereçamento

| Quem | IP | Onde |
|---|---|---|
| Rocket M2 (mgmt) | 192.168.1.20 | airOS |
| Bullet M2 (mgmt) | 192.168.1.21 | airOS |
| t620 (estação) | 192.168.1.10 | `enp0s26u1u6u3` |
| Raspberry Pi (barco) | 192.168.1.11 | eth0 |

t620, persistente:

    sudo nmcli con add type ethernet ifname enp0s26u1u6u3 con-name rocket \
        ip4 192.168.1.10/24

Pi (imagem Emlid usa dhcpcd): em `/etc/dhcpcd.conf`:

    interface eth0
    static ip_address=192.168.1.11/24

## 2. Antenas (airOS, http://192.168.1.20 e .21, padrão ubnt/ubnt)

| Campo | Rocket (estação) | Bullet (barco) |
|---|---|---|
| Wireless Mode | Access Point | Station |
| WDS (transparent bridge) | ativado | ativado |
| SSID | `jundia-link` | `jundia-link` |
| Network Mode | Bridge | Bridge |
| Canal / largura | fixo (ex. ch 6, 20 MHz) | herda do AP |
| IP mgmt | 192.168.1.20 | 192.168.1.21 |

- Bancada com antenas a metros: reduzir Output Power ao mínimo (saturação
  piora RTT e mascara o resultado).
- Senha perdida: reset 10 s → ubnt/ubnt @ 192.168.1.20.
- Validar: página Main do airOS com Signal −50 a −65 dBm; depois
  `ping 192.168.1.11` a partir do t620. Só então MQTT.

## 3. Instalação

### Pi (Emlid/Buster)

    sudo apt update && sudo apt install -y mosquitto git python3-opencv python3-pip
    pip3 install paho-mqtt
    git clone -b mqtt-p2p-test https://github.com/lsa-pucrs/fleet-dt.git ~/fleet-dt
    sudo systemctl disable --now mosquitto   # o serviço default sobe sem o nosso conf

Se o pip da imagem instalar paho 1.x (Python 3.7), os scripts precisam de
adaptação pequena (`CallbackAPIVersion` é 2.x) — reportar antes de hackear.

### Estação (t620)

    pip3 install --user paho-mqtt
    git clone -b mqtt-p2p-test https://github.com/lsa-pucrs/fleet-dt.git ~/fleet-dt

Sem broker novo (ver §0).

## 4. Rodar

### Pi

    mosquitto -c ~/fleet-dt/mqtt/pi/mosquitto.conf &
    python3 ~/fleet-dt/mqtt/pi/publisher.py --broker localhost --boat-id b1

ZED (segunda sessão) — modos UVC reais da ZED (olhos lado a lado);
confirmar com `v4l2-ctl --list-formats-ext -d /dev/video0`:

    python3 ~/fleet-dt/mqtt/pi/zed_publisher.py --broker localhost --boat-id b1 \
        --width 2560 --height 720 --fps 15 --quality 70
    # começar leve: --width 1344 --height 376

Se o encode JPEG + ArduPilot da Emlid saturarem CPU (`top`), derrubar
`--fps`/`--quality` — o custo aparece como kbps menor no subscriber.

### t620

    cd ~/fleet-dt && mkdir -p results/mqtt-p2p/$(date +%F)
    D=results/mqtt-p2p/$(date +%F)

    # baseline sem video
    python3 mqtt/jmcs/probe.py --broker 192.168.1.11 --boat-id b1 \
        --count 200 --csv $D/rtt-semvideo.csv
    python3 mqtt/jmcs/subscriber.py --broker 192.168.1.11 --boat-id b1 \
        --duration-s 120 --csv $D/load-semvideo.csv

    # liga zed_publisher no Pi, repete
    python3 mqtt/jmcs/probe.py --broker 192.168.1.11 --boat-id b1 \
        --count 200 --csv $D/rtt-comvideo.csv
    python3 mqtt/jmcs/subscriber.py --broker 192.168.1.11 --boat-id b1 \
        --duration-s 120 --csv $D/load-comvideo.csv

A diferença rtt-semvideo vs rtt-comvideo = custo do vídeo-pelo-broker nos
pacotes pequenos. O subscriber responde "quais pacotes, quantos kbps".

## 5. Salvar (nesta branch)

    results/mqtt-p2p/<AAAA-MM-DD>/
    ├── notes.md            # OBRIGATÓRIO: distância, potência TX, canal,
    │                       # signal dBm, o que mais rodava nos hosts
    ├── load-semvideo.csv
    ├── load-comvideo.csv
    ├── rtt-semvideo.csv
    └── rtt-comvideo.csv

    git add results && git commit -m "mqtt-p2p: <condição medida>" && git push

Sem `notes.md` dois dias de medida não se comparam.

## 6. Problemas conhecidos

| Sintoma | Causa provável |
|---|---|
| ping passa, MQTT não conecta | broker do Pi não subiu com o conf da pasta, ou porta 1883 filtrada |
| nada chega no subscriber | rádios em Router/NAT em vez de Bridge |
| RTT pior na bancada que em campo | potência TX alta a curta distância (saturação) |
| frames zed não chegam | `max_packet_size` do broker (conf desta pasta já traz 1 MB) |
| RTT com cauda longa sob vídeo | fila do broker; anotar e reportar — é o dado do experimento |
