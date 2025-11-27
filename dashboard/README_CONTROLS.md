# Sistema de Controle Remoto do AGV

## Funcionalidades Adicionadas

### 1. **Botões de Controle**
- **▶️ Start Robot**: Inicia o movimento do AGV (habilita motores)
- **⏹️ Stop Robot**: Para o AGV imediatamente (desabilita motores)

### 2. **Indicador de Obstáculos**
- **✅ No Obstacle**: Exibido quando não há obstáculos detectados
- **⚠️ Obstacle at Xmm**: Pisca em vermelho quando um obstáculo é detectado a menos de 120mm
- Animação de pulso para alertar visualmente

## Arquitetura

### Fluxo de Comandos
```
Dashboard → WebSocket → Python Server → Serial/Bluetooth → Microcontrolador
```

### Fluxo de Detecção de Obstáculos
```
Sensor Ultrassônico → Microcontrolador → Bluetooth → Python → WebSocket → Dashboard
```

## Protocolo de Comunicação

### Comandos (Dashboard → MCU)
- `CMD:START\n` - Inicia o robô
- `CMD:STOP\n` - Para o robô

### Telemetria (MCU → Dashboard)
- `L:<pwm>,R:<pwm>,E:<erro>,D:<distancia>\n` - Dados em tempo real

## Comportamento do Sistema

### Estado Inicial
- Robô desabilitado (motores parados)
- Aguardando comando START

### Após START
- Motores habilitados
- Seguidor de linha ativo
- Detecção de obstáculos ativa
- Se obstáculo < 120mm → motores param automaticamente

### Após STOP
- Motores desabilitados imediatamente
- Telemetria continua sendo enviada
- Sensores continuam lendo (sem atuação)

## Segurança

1. **Parada de Emergência**: Botão STOP funciona a qualquer momento
2. **Detecção Automática**: Obstáculos param o robô mesmo sem comando
3. **Estado Persistente**: Robô sempre inicia desabilitado
4. **Feedback Visual**: Indicador de obstáculo em tempo real

## Uso

1. Conecte o dashboard ao WebSocket
2. Aguarde telemetria (confirma conexão)
3. Clique **START** quando pronto
4. Monitore obstáculos no indicador
5. Use **STOP** para parada de emergência

## Detalhes Técnicos

- **Threshold de Obstáculo**: 120mm
- **Taxa de Atualização**: 200ms (5Hz)
- **Latência de Comando**: <50ms
- **Animação de Alerta**: 1s pulse cycle
