# Comandos Bluetooth do AGV

Este documento descreve todos os comandos disponíveis para controle do AGV via Bluetooth.

## Sistema de Comandos

Os comandos seguem o formato da biblioteca `bt_kmn`:
- Comandos simples: `COMANDO\n`
- Comandos com parâmetros: `COMANDO=parametros\n`

## Lista de Comandos

### 1. START
**Uso**: `START`  
**Descrição**: Inicia o robô e habilita o seguidor de linha  
**Exemplo**:
```
START
```
**Resposta**: `Robot STARTED`

---

### 2. STOP
**Uso**: `STOP`  
**Descrição**: Para o robô imediatamente e desabilita todos os motores  
**Exemplo**:
```
STOP
```
**Resposta**: `Robot STOPPED`

---

### 3. STATUS
**Uso**: `STATUS`  
**Descrição**: Retorna o estado atual do robô (rodando ou parado)  
**Exemplo**:
```
STATUS
```
**Resposta**: `Robot is RUNNING` ou `Robot is STOPPED`

---

### 4. SPEED
**Uso**: `SPEED=<valor>` ou `SPEED` (consulta)  
**Descrição**: Define ou consulta a velocidade base dos motores  
**Parâmetros**:
- `valor`: Velocidade entre 0 e 1000 (PWM)

**Exemplos**:
```
SPEED=500        # Define velocidade para 500
SPEED            # Consulta velocidade atual
```
**Resposta**: `Base speed set to 500` ou `Current speed: 500`

---

### 5. PID
**Uso**: `PID=<kp>,<ki>,<kd>` ou `PID` (consulta)  
**Descrição**: Define ou consulta as constantes do controlador PID  
**Parâmetros**:
- `kp`: Constante proporcional
- `ki`: Constante integral
- `kd`: Constante derivativa

**Exemplos**:
```
PID=1.5,0.0,0.5  # Define Kp=1.5, Ki=0.0, Kd=0.5
PID              # Consulta valores atuais
```
**Resposta**: `PID set to Kp=1.500 Ki=0.000 Kd=0.500`

---

### 6. HELP
**Uso**: `HELP`  
**Descrição**: Lista todos os comandos disponíveis  
**Exemplo**:
```
HELP
```
**Resposta**: Lista formatada de comandos

---

## Telemetria Automática

O AGV envia automaticamente dados de telemetria a cada 200ms:

**Formato**: `L:<pwm_esq>,R:<pwm_dir>,E:<erro>,D:<distancia>\n`

**Exemplo**:
```
L:150,R:180,E:1.23,D:450
```

Onde:
- `L`: PWM do motor esquerdo (0-255)
- `R`: PWM do motor direito (0-255)
- `E`: Erro do seguidor de linha (float)
- `D`: Distância do obstáculo em mm

---

## Usando via Dashboard Web

O dashboard web envia comandos automaticamente quando você clica nos botões:
- **▶️ Start Robot** → envia `START`
- **⏹️ Stop Robot** → envia `STOP`

---

## Usando via Terminal Serial

Você pode enviar comandos manualmente usando qualquer terminal serial (PuTTY, Arduino IDE, etc.):

1. Conecte à porta COM do Bluetooth (9600 baud)
2. Digite o comando seguido de Enter
3. Observe a resposta

**Exemplo de sessão**:
```
> HELP
=== AGV Commands ===
START        - Start robot
STOP         - Stop robot
...

> START
Robot STARTED

> SPEED=600
Base speed set to 600

> STATUS
Robot is RUNNING

> STOP
Robot STOPPED
```

---

## Adicionando Novos Comandos

Para adicionar um novo comando, edite `src/agv_commands.cpp`:

```cpp
// Command: MEU_COMANDO
// Usage: MEU_COMANDO=<param>
// Description: Descrição do comando
BT_COMMAND_DEFINE(MEU_COMANDO) {
    // params contém os parâmetros após o '='
    // Implemente a lógica aqui
    
    Bluetooth_SendMessage("Resposta do comando\n");
}
```

O sistema de comandos usa macros da biblioteca `bt_kmn` para registrar automaticamente os comandos disponíveis.

---

## Notas Técnicas

1. **Terminação**: Todos os comandos devem terminar com `\n`
2. **Case Sensitive**: Os comandos são case-sensitive (use maiúsculas)
3. **Buffer**: Comandos limitados a 128 caracteres
4. **Delimitador**: Parâmetros são separados por `=`
5. **Múltiplos Parâmetros**: Use vírgulas: `CMD=param1,param2,param3`
