# Plataforma Didática Embarcada para Reprodução de Arquivos COMTRADE
e Emulação de Relés de Proteção em Sistemas Elétricos de Potência

Projeto de plataforma embarcada para geração e recepção de sinais COMTRADE em uma arquitetura com duas ESP32:

- a ESP32 geradora reproduz os arquivos `.cfg` e `.bdat/.dat`;
- a ESP32 receptora faz aquisição, processamento RMS/Fourier e execução das proteções.

O repositório também guarda versões antigas e experimentos anteriores em `old_versions/` e subpastas com nomes históricos. A implementação atual está concentrada principalmente em:

- `generator/python/embedded_generator/`
- `receiver/python/embedded_receiver/`

## Visão Geral

O fluxo principal é este:

1. O MATLAB/Simulink gera a oscilografia em formato COMTRADE.
2. `run_embedded_generator.py` envia os arquivos para a ESP32 geradora e produz um `config.json` do ensaio.
3. A ESP32 geradora armazena os dados e reproduz os canais de tensão e corrente.
4. `run_embedded_receiver.py` configura a ESP32 receptora com base no `config.json`.
5. A ESP32 receptora executa as funções de proteção e expõe o resultado pela serial e/ou pelos GPIOs, dependendo da versão do firmware.

## Estrutura Principal

- `generator/python/embedded_generator/embedded_generator.ino`
  - firmware da ESP32 geradora;
- `generator/python/embedded_generator/run_embedded_generator.py`
  - script de upload e terminal para a geradora;
- `generator/python/embedded_generator/config.json`
  - exemplo de configuração gerada para um ensaio;
- `receiver/python/embedded_receiver/embedded_receiver.ino`
  - firmware da ESP32 receptora;
- `receiver/python/embedded_receiver/run_embedded_receiver.py`
  - script de configuração e execução da receptora;
- `receiver/python/embedded_receiver/configure_embedded_receiver.py`
  - utilitário para aplicar configurações no firmware via edição do sketch;
- `receiver/python/embedded_receiver/README.md`
  - documentação detalhada da versão do receptor embarcado.

## Requisitos

- Python 3.10+;
- `pyserial`;
- ambiente Arduino IDE ou PlatformIO com suporte a ESP32;
- uma ESP32 para a geradora e outra para a receptora, se você for usar o fluxo completo.

Instalação mínima do pacote Python:

```bash
pip install pyserial
```

## Fluxo Recomendado

### 1. Preparar o COMTRADE

Gere os arquivos `.cfg` e `.bdat/.dat` do ensaio no seu fluxo de simulação.

### 2. Enviar o sinal para a ESP32 geradora

Exemplo:

```bash
python3 generator/python/embedded_generator/run_embedded_generator.py \
  --cfg generator/python/embedded_generator/samples/samples_/Matlab_comtrade/01_CC_Trifasico/25%/export.cfg \
  --bdat generator/python/embedded_generator/samples/samples_/Matlab_comtrade/01_CC_Trifasico/25%/export.bdat \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --receiver-config config.json
```

Comandos interativos da geradora:

- `t`: senoide de teste;
- `s`: pré-falta contínua;
- `f`: COMTRADE completo a partir da pré-falta;
- `p`: COMTRADE completo;
- `q`: retorno ao idle;
- `x`: encerra o terminal.

### 3. Configurar e iniciar a ESP32 receptora

Exemplo:

```bash
python3 receiver/python/embedded_receiver/run_embedded_receiver.py \
  --port /dev/ttyUSB1 \
  --config config.json \
  --normalize-to-comtrade \
  --over-current 50 10 0.05 \
  --distance 52.12496 80 120 0.05 \
  --distance-line-angle 86.636 \
  --directional-67 forward \
  --directional-67-power-min 10000 \
  --under-voltage 50 10 0.20 \
  --over-voltage 50 10 0.20 \
  --protection-events
```

## Observações Importantes

- Use sempre o `config.json` produzido para o mesmo ensaio COMTRADE.
- A ordem dos parâmetros de proteção importa. Por exemplo, em `--over-current 50 10 0.05`, o primeiro valor é o estágio instantâneo e o segundo é o temporizado.
- O baudrate padrão dos scripts é `921600`.
- A pasta `old_versions/` contém histórico de experimentos e não deve ser tratada como a versão principal do projeto.

## Documentação Relacionada

- `generator/python/embedded_generator/play.txt`
- `generator/python/embedded_generator/TCC.txt`
- `receiver/python/embedded_receiver/README.md`

## Licença

Não há arquivo de licença explícito neste repositório. Se necessário, adicione uma antes de distribuir o projeto.
