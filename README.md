# 🤖 Mini AGV - Autonomous Guided Vehicle

Robô seguidor de linha com controle PID, detecção de obstáculos e monitoramento em tempo real via dashboard web.

## 🎯 Sobre o Projeto

Este projeto implementa um AGV (Automated Guided Vehicle) utilizando o microcontrolador **RP2040**, capaz de seguir linhas de forma autônoma com controle PID ajustável, detectar obstáculos e transmitir telemetria em tempo real para um dashboard web.

### Características Principais

- **Seguidor de linha** com 8 sensores infravermelhos multiplexados
- **Controle PID** ajustável em tempo real via Bluetooth
- **Detecção de obstáculos** com sensor ultrassônico HC-SR04
- **Dashboard web** para visualização de telemetria e controle remoto
- **Comunicação Bluetooth** para comandos e telemetria
- **Sistema de calibração** automático dos sensores

---

## ✨ Funcionalidades

- ✅ Seguimento de linha preto/branco com controle PID
- ✅ Detecção e parada automática diante de obstáculos (< 120mm)
- ✅ Telemetria em tempo real (200ms): PWM dos motores, erro de linha, distância
- ✅ Dashboard web com gráficos ao vivo (Chart.js)
- ✅ Comandos via Bluetooth: START, STOP, CALIBRATE, PID
- ✅ Calibração automática dos sensores de linha
- ✅ Ajuste de constantes PID em tempo real
- ✅ Controle de velocidade base dos motores

---

## 🔧 Hardware

### Lista de Materiais

#### Barra de Sensor

| Componente | Quantidade | Descrição |
|------------|------------|-----------|
| **Sensor de linha IR** | 8 | QRE1113 |
| **Resistor 220 Ohms** | 8 | 220 Ohms |
| **Resistor 22k Ohms** | 8 | 22k Ohms|
| **CD4051** | 1 | Multiplexador analógico 8:1 |

#### Placa Principal

| Componente | Quantidade | Descrição |
|------------|------------|-----------|
| **Raspberry Pi Pico** | 1 | Microcontrolador principal (RP2040) |
| **HC-SR04** | 1 | Sensor ultrassônico para detecção de obstáculos |
| **L298N** | 1 | Drivers de motor DC (ou 1 módulo duplo) |
| **Motor DC** | 2 | Motores DC 3-6V |
| **HC-05/06** | 1 | Módulo Bluetooth (UART) |
| **LM7805** | 1 | Regulador 5V |
| **LM1117** | 1 | Regulador 3v3|
| **Capacitor 10uF** | 4 | Capacitor Eletrolitico 10uF |
| **Resistor 10k Ohms**| 4 | Resistor 10k Ohms |
| **2N7000** | 2 | MOSFET 2N7000 |
| **Diodo** | 1 | Diodo 1N4007 |

#### Chassi
| Componente | Quantidade | Descrição |
|------------|------------|-----------|
| **Placa de MDF** | 1 | Placa de MDF de 3mm para corte do chassi |
| **Rodas** | 2 | Rodas de 65mm |
| **PLA** | 1 | Carretel de filamento PLA para impressão de peças |
| **Elasticos** | 10 | Elasticos de dinheiro para serem colocados nas rodas |
| **Parafusos e Porcas** | 6 | Parafusos e Porcas M3 |
| **Feltro** | 1 | Feltro para terceiro ponto de apoio |
### Pinagem

| Periférico | Pino(s) Pico W | Descrição |
|------------|----------------|-----------|
| **Bluetooth (UART1)** | GPIO 8 (TX), GPIO 9 (RX) | Comunicação serial 9600 baud |
| **Ultrassônico** | GPIO 2 (TRIG), GPIO 3 (ECHO) | Medição de distância |
| **Sensores de Linha** | GPIO 18, 19, 20 (Mux S0-S2), GPIO 26 (ADC0) | 8 sensores via multiplexador |
| **Motor Esquerdo** | GPIO 11 (ENA), GPIO 14 (IN1A), GPIO 15 (IN2A) | Controle PWM + direção |
| **Motor Direito** | GPIO 10 (ENB), GPIO 12 (IN1B), GPIO 13 (IN2B) | Controle PWM + direção |

---

## 💻 Software

### Tecnologias Utilizadas

#### Firmware (C++)
- **Pico SDK 2.2.0** - Framework oficial da Raspberry Pi
- **C++17** - Linguagem principal
- **CMake** - Sistema de build
- **bt_kmn** - Biblioteca customizada para Bluetooth com sistema de comandos

#### Dashboard (Web)
- **Python 3.x** - Servidor WebSocket/HTTP
- **asyncio + websockets** - Comunicação assíncrona
- **pyserial** - Interface com porta serial Bluetooth
- **HTML5 + JavaScript** - Frontend do dashboard
- **Chart.js** - Visualização de telemetria em tempo real

### Arquitetura

```
Raspberry Pi Pico (Firmware C++)
         ↓ Bluetooth UART (9600 baud)
    Módulo HC-05/06
         ↓ USB Serial (COM Port)
 Computador (Python Server)
         ↓ WebSocket (8765)
   Dashboard Web (HTML/JS)
```

---

## 🔨 Montagem

### Esquemas de Ligação

#### Esquema Principal - Motores, Bluetooth e Ultrassônico

<p align="center">
  <img src="imagens/esquema_principal.png" alt="Esquema Principal" />
</p>

#### Esquema dos Sensores de Linha (Multiplexador)

<p align="center">
  <img src="imagens/esquema_sensores.png" alt="Esquema Sensores" />
</p>

### Foto da Montagem Final

<p align="center">
  <img src="imagens/montagem_final.JPG" alt="Montagem Final" />
</p>

<p align="center"><em>Robô montado com todos os componentes integrados</em></p>
---

## 🚀 Instalação e Configuração

### Pré-requisitos

- **Raspberry Pi Pico SDK** (2.2.0 ou superior)
- **CMake** (3.13+)
- **Ninja** (build system)
- **Python 3.8+** com pip
- **Git** (para clonar submódulos)

### Passo 1: Clonar o Repositório

```bash
git clone https://github.com/ptBauke1/mini_agv.git
cd mini_agv
git submodule update --init --recursive
```

### Passo 2: Compilar o Firmware

```bash
# Criar diretório de build
mkdir build
cd build

# Configurar CMake
cmake ..

# Compilar
ninja
```

O arquivo `projeto_agvs.uf2` será gerado em `build/`.

### Passo 3: Instalar Dependências Python (Dashboard)

```bash
cd dashboard
python -m venv .venv
source .venv/bin/activate  # Linux/Mac
# ou
.venv\Scripts\activate     # Windows

pip install asyncio websockets pyserial
```

### Passo 4: Flashear o Pico W

1. Conecte o Pico W ao PC segurando o botão **BOOTSEL**
2. Copie `build/projeto_agvs.uf2` para o drive USB que apareceu
3. O Pico reiniciará automaticamente com o novo firmware

---

## 🎮 Uso

### Iniciar o Sistema

1. **Ligar o robô** (conectar bateria)
2. **Conectar o módulo Bluetooth** ao computador
3. **Iniciar o servidor Python**:
   ```bash
   cd dashboard
   python bt_websocket_server.py
   ```
4. **Abrir o dashboard** no navegador:
   ```
   http://localhost:8080/dashboard.html
   ```

### Calibração dos Sensores

1. No dashboard, clique em **"Connect WS"** para conectar ao servidor
2. Selecione a porta COM do Bluetooth e clique em **"Connect COM"**
3. Posicione o robô sobre a linha (metade dos sensores no preto, metade no branco)
4. Clique em **"🎯 Calibrate Sensors"**
5. **Movimente o robô lentamente** sobre a linha por ~10 segundos
6. Aguarde a mensagem "Calibration COMPLETE"

### Operação Normal

1. Posicione o robô sobre a linha
2. Clique em **"▶️ Start Robot"**
3. Ajuste constantes PID se necessário (painel "⚙️ PID Control")
4. Monitore telemetria em tempo real nos gráficos

---

## 📊 Dashboard Web

### Recursos do Dashboard

- **Status de Conexão**: WebSocket e COM Port separados
- **Controles do Robô**: START, STOP, CALIBRATE
- **Painel PID**: Ajuste de Kp, Ki, Kd em tempo real
- **Indicador de Obstáculos**: Visual + alerta sonoro (animado)
- **Métricas Instantâneas**: PWM motores, erro de linha, distância
- **Gráficos em Tempo Real**: 
  - PWM dos motores (esquerdo/direito)
  - Erro do seguidor de linha
  - Distância do sensor ultrassônico
- **Log de Dados**: Histórico de comandos e eventos

### Captura de Tela

<p align="center">
  <img src="imagens/dashboard.png" alt="Dashboard Web" />
</p>

<p align="center"><em>Interface do dashboard com gráficos em tempo real e controles do robô</em></p>

---

## 📡 Comandos Bluetooth

O robô aceita os seguintes comandos via Bluetooth (9600 baud, `\n` terminador):

### Comandos Básicos

| Comando | Formato | Descrição | Resposta |
|---------|---------|-----------|----------|
| **START** | `START` | Inicia o robô | `Robot STARTED` |
| **STOP** | `STOP` | Para o robô e motores | `Robot STOPPED` |
| **CALIBRATE** | `CALIBRATE` | Calibra sensores de linha | `Calibration COMPLETE` |

### Ajuste de PID

| Comando | Formato | Exemplo | Descrição |
|---------|---------|---------|-----------|
| **PID** | `PID <kp> <ki> <kd>` | `PID 0.3 0.0 0.02` | Define constantes PID |
| **PID** | `PID` (sem parâmetros) | `PID` | Consulta valores atuais |

**Resposta**: `PID updated: Kp=0.300 Ki=0.000 Kd=0.020`

### Formato de Telemetria

A cada 200ms, o robô envia automaticamente:
```
L:<left_pwm>,R:<right_pwm>,E:<error>,D:<distance>
```
**Exemplo**: `L:850,R:750,E:-1.25,D:350`

Veja [COMMANDS.md](COMMANDS.md) para documentação completa.

---

## 📁 Estrutura do Projeto

```
projeto_agvs/
├── CMakeLists.txt              # Configuração CMake principal
├── projeto_agvs.cpp            # Código principal (main loop, PID)
├── pico_sdk_import.cmake       # Importação do Pico SDK
├── COMMANDS.md                 # Documentação detalhada de comandos
├── README.md                   # Este arquivo
│
├── include/                    # Headers personalizados
│   ├── sensors.hpp             # Sensores de linha + ultrassônico
│   ├── motors.hpp              # Controle de motores L298N
│   ├── gyro.hpp                # Giroscópio MPU6050
│   └── agv_commands.h          # Sistema de comandos
│
├── src/                        # Implementações
│   ├── sensors.cpp
│   ├── motors.cpp
│   ├── gyro.cpp
│   └── agv_commands.cpp        # Comandos Bluetooth (START, STOP, PID, etc.)
│
├── bluetooth_sdk/              # Biblioteca bt_kmn (submódulo)
│   ├── include/bt_kmn/
│   │   ├── bluetooth.h
│   │   └── commands.h
│   └── src/
│       ├── bluetooth.c
│       └── commands.c
│
├── dashboard/                  # Interface web
│   ├── bt_websocket_server.py # Servidor Python (WebSocket + Serial)
│   └── dashboard.html          # Frontend (Chart.js + controles)
│
└── build/                      # Arquivos de compilação (gerado)
    └── projeto_agvs.uf2        # Firmware final
```

---

## 🔬 Algoritmo de Controle

### Controle PID

O robô utiliza um controlador PID para seguir a linha:

```
erro = posição_calculada - centro_linha
integral += erro
derivada = erro - erro_anterior
correção = Kp × erro + Ki × integral + Kd × derivada

pwm_esquerdo = base_speed + correção
pwm_direito = base_speed - correção
```

**Valores padrão**:
- Kp = 1.75 (proporcional)
- Ki = 0.0 (integral - desabilitado para evitar overshooting)
- Kd = 520.0 (derivativo)
- Base Speed = 800 (PWM de 0-1023)

### Detecção de Obstáculos

- Leitura do ultrassônico a cada 10ms
- Limiar de parada: **120mm**
- Timeout de leitura: 50ms
- Ação: Para motores automaticamente (`robot_enabled = false`)

---

## 🐛 Troubleshooting

### Problema: Sensores de linha não calibram corretamente
**Solução**: 
- Verificar se os 8 sensores estão conectados ao multiplexador
- Aumentar contraste da linha (preto mais escuro, branco mais claro)
- Ajustar altura dos sensores (3-5mm da superfície)

### Problema: Ultrassônico retorna 0 ou 65535
**Solução**:
- Verificar pino ECHO (deve estar em nível lógico compatível - usar divisor de tensão se necessário)
- Aumentar timeout em `sensors.cpp` (padrão 50ms)
- Verificar alimentação do HC-SR04 (5V estável)

### Problema: Dashboard não conecta
**Solução**:
- Verificar firewall (liberar porta 8765 WebSocket + 8080 HTTP)
- Conferir porta COM do Bluetooth (ajustar em `bt_websocket_server.py` ou no dashboard)
- Testar comunicação serial: `python -m serial.tools.miniterm COM3 9600`

### Problema: Motores não respondem
**Solução**:
- Verificar alimentação dos drivers L298N (bateria carregada)
- Conferir jumpers de enable nos L298N
- Testar PWM com osciloscópio/LED (pinos 10 e 11 do Pico)

---

## 📚 Referências

- [Raspberry Pi Pico SDK Documentation](https://www.raspberrypi.com/documentation/pico-sdk/)
- [Chart.js Documentation](https://www.chartjs.org/docs/)

---

**Última atualização**: Novembro 2025
