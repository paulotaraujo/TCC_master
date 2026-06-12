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
  -> saída de relé
```

O CSV não é escrito no laço crítico. A ESP32 mantém um log compacto em RAM e só exporta CSV quando recebe `dump` pela serial.

## Comandos seriais

- `start`: retoma aquisição e proteção.
- `stop`: pausa aquisição e proteção.
- `status`: mostra estado atual.
- `dump`: exporta o log RAM como CSV essencial.
- `clear`: limpa o log RAM.
- `stream on`: envia uma linha CSV por ciclo fechado.
- `stream off`: desativa streaming.
- `resettrip`: limpa trips retidos e relé.

## Exportar CSV

```bash
python3 dump_embedded_log.py --port /dev/ttyUSB1 --out embedded_capture.csv --stop-first
python3 read_embedded_capture_csv.py --csv embedded_capture.csv --plot --plot-mode fourier_scaled
```

## Pontos críticos

- Atualize as constantes de escala no topo de `v_2.ino` sempre que o `receiver_config.json` do gerador mudar.
- O log em RAM é limitado e circular (`512` linhas nesta versão). Para ensaios longos, a próxima etapa recomendada é SD com arquivo binário compacto.
- Não habilite escrita CSV em tempo real dentro do laço de proteção.
- Deixe as funções de proteção desligadas até configurar os parâmetros do ensaio no topo do firmware.
