# Receptor embarcado ESP32

Primeira versão da recepção full-embedded baseada nos arquivos ideais:

- `receiver/python/v_1/v_1.ino`: referência de aquisição ADC robusta.
- `receiver/python/v_1/read_from_generator_rms_robust.py`: referência de processamento, Fourier e proteções.

## Fluxo

```text
ADC GPIO35 (tensão) / GPIO34 (corrente)
  -> auto-offset e escala
  -> zero-cross por tensão
  -> fasores RMS da fundamental por Fourier
  -> proteções 50/51, 21 MHO, 67, 27/59
  -> saídas digitais para relés
```

Nesta versão não há escrita nem exportação de oscilografia pela ESP32. O resultado da proteção de distância é enviado diretamente para GPIOs, pensados para um módulo de relé de 4 canais.

## Saídas de relé

Mapeamento padrão em `embedded_receiver.ino`:

- GPIO14 / IN1: bloqueio pela supervisão 67.
- GPIO27 / IN2: habilitação/permissivo da supervisão 67.
- GPIO26 / IN3: trip instantâneo agregado (21/Z1, 27, 50 ou 59).
- GPIO25 / IN4: trip temporizado agregado (21/Z2, 27, 51 ou 59).

Na ESP32 DevKit da foto, esses pinos ficam no lado direito da placa como `D14`, `D27`, `D26` e `D25`. Ligue cada entrada do módulo de relé ao GPIO correspondente acima, independentemente da ordem física no conector.

Ligação sugerida do módulo de relé:

- IN1 -> D14/GPIO14.
- IN2 -> D27/GPIO27.
- IN3 -> D26/GPIO26.
- IN4 -> D25/GPIO25.
- GND do módulo -> GND da ESP32.
- VCC do módulo -> alimentação exigida pelo módulo de relé.

As saídas estão configuradas como ativas em nível alto (`RELAY_ACTIVE_LOW = false`). Se o módulo utilizado possuir entradas ativas em nível baixo, altere essa constante e verifique o estado seguro durante a inicialização.

As saídas GPIO26 e GPIO25 dependem das funções habilitadas no ensaio. Elas são
acionadas somente depois do trip; pickup e temporização em andamento são
informados pela serial. A origem específica do trip também permanece disponível
nos eventos seriais.

## Comandos seriais

- `start`: retoma aquisição e proteção.
- `stop`: pausa aquisição e proteção.
- `status`: mostra estado atual.
- `resettrip`: limpa trips retidos e desaciona as saídas digitais.
- `testrelays`: pulsa as quatro saídas, uma por vez, para validar ligação GPIO/modulo.

## Rodar como o receptor Python antigo

Depois de compilar/enviar o firmware `embedded_receiver.ino` para a ESP32, rode a recepção com um único comando:

```bash
python3 run_embedded_receiver.py \
  --port /dev/ttyUSB1 \
  --config /home/paulo/Arduino/dev/1_phase/generator/python/embedded_generator/config.json \
  --normalize-to-comtrade \
  --over-current 50 10 0.05 \
  --under-voltage 50 10 0.20 \
  --over-voltage 50 10 0.20 \
  --distance 52.12496 80 120 0.05 \
  --distance-line-angle 86.636 \
  --directional-67 reverse \
  --protection-events
```

Os argumentos das proteções de magnitude seguem a ordem estágio instantâneo,
estágio temporizado e atraso. Assim, `--over-current 50 10 0.05` configura a
função 50 em 1,50 vezes a corrente nominal e a função 51 em 1,10 vezes a corrente
nominal por 50 ms. Em `--under-voltage 50 10 0.20`, a função 27 atua
instantaneamente em 0,50 vezes a tensão nominal e temporiza em 0,90 vezes a
nominal por 200 ms. Em `--over-voltage 50 10 0.20`, os níveis correspondentes
são 1,50 e 1,10 vezes a tensão nominal.

O script configura a ESP32 por serial, inicia a leitura/proteção e fica em execução. Ao pressionar `Ctrl+C`, ele envia `stop`. Os trips ficam retidos nas saídas até `resettrip` ou reinício da ESP32.

Com `--protection-events`, a ESP32 imprime eventos no terminal no estilo do receptor Python antigo, sem escrever oscilografia:

- `[D21 BLOCKED BY 67]`
- `[D21Z1 TRIP]`
- `[D21Z2 PICKUP]`
- `[D21Z2 TIMING]`
- `[D21Z2 RESET]`
- `[D21Z2 TRIP]`
- `[BREAKER TRIP]`

## Pontos críticos

- Use em cada ensaio o `config.json` produzido para o mesmo COMTRADE; `run_embedded_receiver.py` transmite as escalas e referências ao firmware em tempo de execução.
- Não há oscilografia nesta versão; a validação passa a ser feita pelas saídas digitais de trip.
- Use `testrelays` antes do ensaio para confirmar se IN1..IN4 estão acionando fisicamente.
- Deixe as funções de proteção desligadas até configurar os parâmetros do ensaio no topo do firmware.
