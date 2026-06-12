# ESP32 Receiver Embedded v2

Primeira versão da recepção full-embedded baseada nos arquivos ideais:

- `receiver/python/v_1/v_1.ino`: referência de aquisição ADC robusta.
- `receiver/python/v_1/read_from_generator_rms_robust.py`: referência de processamento, Fourier e proteções.

## Fluxo

```text
ADC GPIO34/GPIO35
  -> auto-offset e escala
  -> zero-cross por tensão
  -> fasores RMS da fundamental por Fourier
  -> proteções 50/51, 21 MHO, 67, 27/59
  -> saídas digitais para relés
```

Nesta versão não há escrita nem exportação de oscilografia pela ESP32. O resultado da proteção de distância é enviado diretamente para GPIOs, pensados para um módulo de relé de 4 canais.

## Saídas de relé

Mapeamento padrão em `v_2.ino`:

- GPIO25 / IN1: bloqueio 21/67.
- GPIO26 / IN2: habilitação/permissivo 21/67.
- GPIO27 / IN3: trip zona 1.
- GPIO14 / IN4: trip zona 2.

Na ESP32 DevKit da foto, esses pinos ficam no lado direito da placa como `D14`, `D27`, `D26` e `D25`. Ligue cada entrada do módulo de relé ao GPIO correspondente acima, independentemente da ordem física no conector.

Ligação sugerida do módulo de relé:

- IN1 -> D25/GPIO25.
- IN2 -> D26/GPIO26.
- IN3 -> D27/GPIO27.
- IN4 -> D14/GPIO14.
- GND do módulo -> GND da ESP32.
- VCC do módulo -> alimentação exigida pelo módulo de relé.

Os relés estão configurados como ativo em nível baixo (`RELAY_ACTIVE_LOW = true`), que é comum em módulos de 4 canais. Se o seu módulo acionar em nível alto, altere essa constante para `false`.

Quando houver trip em zona 1 ou zona 2, a saída de habilitação também fica acionada.

## Comandos seriais

- `start`: retoma aquisição e proteção.
- `stop`: pausa aquisição e proteção.
- `status`: mostra estado atual.
- `resettrip`: limpa trips retidos e desaciona as saídas digitais.
- `testrelays`: pulsa as quatro saídas, uma por vez, para validar ligação GPIO/modulo.

## Rodar como o receptor Python antigo

Depois de compilar/enviar o firmware `v_2.ino` para a ESP32, rode a recepção com um único comando:

```bash
python3 run_embedded_receiver.py \
  --port /dev/ttyUSB1 \
  --config /home/paulo/Arduino/dev/1_phase/generator/pyhton/v_6/receiver_config.json \
  --normalize-to-comtrade \
  --distance 52.12496 80 120 0.05 \
  --distance-line-angle 86.636 \
  --directional-67 reverse 86.636 90 \
  --protection-events
```

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

- Atualize as constantes de escala no topo de `v_2.ino` sempre que o `receiver_config.json` do gerador mudar.
- Não há oscilografia nesta versão; a validação passa a ser feita pelas saídas digitais de trip.
- Use `testrelays` antes do ensaio para confirmar se IN1..IN4 estão acionando fisicamente.
- Deixe as funções de proteção desligadas até configurar os parâmetros do ensaio no topo do firmware.
