%% plot_subtensao_csv.m
% Compara o dominio FDT/Fourier do Simulink (.mat) com a ESP32 (.csv)
% e plota os trips da funcao 27 instantanea e temporizada.

clear; clc; close all;

%% ===================== CONFIGURACAO =====================
nivel_subtensao = '50';
arquivo_mat = sprintf('faseA_fourier_subtensao_50.mat', nivel_subtensao);
arquivo_csv = sprintf('trimmed_subtensao_50_embedded.csv', nivel_subtensao);

Tfinal = 2.0;
Nref = 2001;
t_ref = linspace(0, Tfinal, Nref).';
metodo_interp = 'pchip';

% CSV no dominio FDT/Fourier.
col_csv_tempo = 'host_t_s';
col_csv_v_fdt = 'v1_mag';

% Canal da fase A no MAT.
canal_mat_v = 1;

% Use true se o bloco Fourier do Simulink exportar pico em vez de RMS.
converter_mat_pico_para_rms = true;

% Suavizacao da curva CSV para reduzir zig-zag visual no FDT/Fourier.
% Janela em amostras do CSV; use 1 para desativar.
suavizar_csv = true;
janela_media_csv = 9;

% Alinha a curva FDT/Fourier da ESP32 ao evento de queda da referencia MAT.
alinhar_csv_por_amplitude_temporal = true;

salvar_figura = true;
arquivo_png = sprintf('sobreposicao_fdt_subtensao_%s_trips_27.png', nivel_subtensao);
arquivo_pdf = sprintf('sobreposicao_fdt_subtensao_%s_trips_27.pdf', nivel_subtensao);

%% ===================== LEITURA DOS ARQUIVOS =====================
if ~isfile(arquivo_mat)
    error('Arquivo MAT nao encontrado: %s', arquivo_mat);
end
if ~isfile(arquivo_csv)
    error('Arquivo CSV nao encontrado: %s', arquivo_csv);
end

S = load(arquivo_mat);
Tcsv = readtable(arquivo_csv, 'VariableNamingRule', 'preserve');

[t_mat_v, v_mat, nome_mat_v] = extrair_sinal_mat(S, 'tensao', canal_mat_v, Tfinal);

checar_coluna(Tcsv, col_csv_v_fdt, arquivo_csv);

t_csv = obter_tempo_csv(Tcsv, col_csv_tempo);
v_csv = double(Tcsv.(col_csv_v_fdt));

if suavizar_csv
    v_csv = media_movel(v_csv, janela_media_csv);
end

if converter_mat_pico_para_rms
    v_mat = v_mat ./ sqrt(2);
end

t_mat_v = normalizar_tempo(t_mat_v, Tfinal);
t_csv = normalizar_tempo(t_csv, Tfinal);

%% ===================== INTERPOLACAO PARA EIXO COMUM =====================
v_mat_i = interp1(t_mat_v(:), v_mat(:), t_ref, metodo_interp, 'extrap');

trip27_inst = obter_trip(Tcsv, 'uv_instant_trip');
trip27_temp = obter_trip(Tcsv, 'uv_timed_trip');
trip27_agregado = obter_trip(Tcsv, 'uv_trip');
if ~any(trip27_inst) && ~any(trip27_temp) && any(trip27_agregado)
    trip27_temp = trip27_agregado;
end

pickup27_inst = obter_constante_csv(Tcsv, 'uv_instant_pickup');
pickup27_temp = obter_constante_csv(Tcsv, 'uv_timed_pickup');
if ~isfinite(pickup27_inst)
    pickup27_inst = obter_constante_csv(Tcsv, 'uv_pickup');
end
if ~isfinite(pickup27_temp)
    pickup27_temp = obter_constante_csv(Tcsv, 'uv_pickup');
end

delta_csv = 0;
if alinhar_csv_por_amplitude_temporal
    t_degrau_mat = estimar_tempo_degrau_descida(t_mat_v, v_mat);
    t_degrau_fdt_csv = estimar_tempo_degrau_descida(t_csv, v_csv);
    if isfinite(t_degrau_mat) && isfinite(t_degrau_fdt_csv)
        delta_csv = t_degrau_mat - t_degrau_fdt_csv;
    else
        warning('Nao foi possivel estimar deslocamento temporal MAT/CSV; usando delta_csv = 0.');
    end
end

t_csv_corrigido = t_csv + delta_csv;
v_csv_i = interp1(t_csv_corrigido(:), v_csv(:), t_ref, metodo_interp, 'extrap');

%% ===================== PLOTS =====================
fig = figure('Color', 'w', 'Name', sprintf('Subtensao %s%% - tensao FDT e trips 27', nivel_subtensao), ...
    'Units', 'centimeters', 'Position', [2, 2, 19, 14]);
tl = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_v = nexttile(tl, 1, [3, 1]);
h_v_mat = plot(ax_v, t_ref, v_mat_i, '--', 'LineWidth', 1.15, ...
    'Color', aplicar_transparencia([0.000, 0.000, 1.000], 0.40), ...
    'DisplayName', 'Tensão de referência (Simulink)'); hold(ax_v, 'on');
h_v_csv = plot(ax_v, t_ref, v_csv_i, 'r-.', 'LineWidth', 1.15, ...
    'DisplayName', 'Tensão processada (ESP32 receptora)');
leg_v = [h_v_mat, h_v_csv];
if isfinite(pickup27_inst)
    cor_p27_inst = aplicar_transparencia([0.929, 0.494, 0.133], 0.20);
    h_p27_inst = plot(ax_v, [0, Tfinal], [pickup27_inst, pickup27_inst], '-', ...
        'Color', cor_p27_inst, 'LineWidth', 1.05, ...
        'DisplayName', 'Tensão de pickup instantânea');
    leg_v(end+1) = h_p27_inst; %#ok<SAGROW>
end
if isfinite(pickup27_temp)
    cor_p27_temp = aplicar_transparencia([0.494, 0.184, 0.556], 0.20);
    h_p27_temp = plot(ax_v, [0, Tfinal], [pickup27_temp, pickup27_temp], '-', ...
        'Color', cor_p27_temp, 'LineWidth', 1.25, ...
        'DisplayName', 'Tensão de pickup temporizada');
    leg_v(end+1) = h_p27_temp; %#ok<SAGROW>
end
grid(ax_v, 'on');
xlim(ax_v, [0, Tfinal]);
ylim(ax_v, limite_y_amplitude_zero([v_mat_i; v_csv_i; pickup27_inst; pickup27_temp]));
ylabel(ax_v, 'Tensão normalizada (Vrms)');
title(ax_v, 'Tensão no domínio FDT');
legend(ax_v, leg_v, 'Location', 'best', 'FontSize', 8);

ax_trip = nexttile(tl, 4);
plotar_trips_27(ax_trip, t_csv_corrigido, trip27_inst, trip27_temp, Tfinal);
xlabel(ax_trip, 'Tempo (s)');
xlim(ax_trip, [0, Tfinal]);

linkaxes([ax_v, ax_trip], 'x');

fprintf('\nArquivos usados:\n');
fprintf('  MAT: %s\n', arquivo_mat);
fprintf('  CSV: %s\n', arquivo_csv);
fprintf('\nVariáveis MAT usadas:\n');
fprintf('  Tensão  : %s, canal %d\n', nome_mat_v, canal_mat_v);
if converter_mat_pico_para_rms
    fprintf('  Conversão MAT: pico -> RMS aplicada com fator 1/sqrt(2).\n');
end
if suavizar_csv
    fprintf('  Suavização CSV: média móvel com janela de %d amostras.\n', janela_media_csv);
end
if alinhar_csv_por_amplitude_temporal
    fprintf('  Degrau MAT: %.6f s | degrau FDT CSV: %.6f s.\n', ...
        t_degrau_mat, t_degrau_fdt_csv);
    fprintf('  Deslocamento aplicado no FDT CSV: %.6f s (%.3f ms).\n', ...
        delta_csv, 1e3 * delta_csv);
    fprintf('  Regra aplicada: t_csv_corrigido = t_csv + deslocamento.\n');
end

if salvar_figura
    exportgraphics(fig, arquivo_png, 'Resolution', 300);
    exportgraphics(fig, arquivo_pdf, 'ContentType', 'vector');
    fprintf('\nFigura salva em: %s\n', fullfile(pwd, arquivo_png));
    fprintf('Figura PDF salva em: %s\n', fullfile(pwd, arquivo_pdf));
end

%% ===================== METRICAS E RELATORIO =====================
met_v = metricas_erro_percentual(v_csv_i, v_mat_i);
t_trip_27_inst = primeiro_instante_trip(t_csv_corrigido, trip27_inst);
t_trip_27_temp = primeiro_instante_trip(t_csv_corrigido, trip27_temp);
tensao_nominal = obter_constante_csv(Tcsv, 'v_nominal_rms');
tensao_falta_permanente = estimar_tensao_falta_permanente(t_ref, v_csv_i, v_mat_i);
t_pickup_27_inst = primeiro_cruzamento_pickup(t_ref, v_csv_i, pickup27_inst);
t_pickup_27_temp = primeiro_cruzamento_pickup(t_ref, v_csv_i, pickup27_temp);
atraso_trip_27_inst = t_trip_27_inst - t_pickup_27_inst;
atraso_trip_27_temp = t_trip_27_temp - t_pickup_27_temp;

fprintf('\n============================================================\n');
fprintf('Erro percentual da tensão FDT entre Simulink e ESP32 receptora\n');
fprintf('Referência: Simulink | Sinal comparado: ESP32 receptora\n');
fprintf('============================================================\n');
fprintf('Tensão FDT:\n');
fprintf('  MAE                   = %.6g V\n',  met_v.MAE);
fprintf('  RMSE                  = %.6g V\n',  met_v.RMSE);
fprintf('  MaxAbs                = %.6g V\n',  met_v.MaxAbs);
fprintf('  Erro relativo ao pico = %.6f %%\n', met_v.ErroPico_pct);
fprintf('  NRMSE                 = %.6f %%\n', met_v.NRMSE_pct);
fprintf('  Tensão nominal        = %.6g V\n', tensao_nominal);
fprintf('  Tensão permanente na falta = %.6g V\n', tensao_falta_permanente);
fprintf('  Pickup 27 instantâneo em t = %.6f s\n', t_pickup_27_inst);
fprintf('  Trip 27 instantâneo em t   = %.6f s\n', t_trip_27_inst);
fprintf('  Pickup 27 temporizado em t = %.6f s\n', t_pickup_27_temp);
fprintf('  Trip 27 temporizado em t   = %.6f s\n', t_trip_27_temp);

TabelaErro = table( ...
    {'Tensão FDT'}, ...
    met_v.MAE, ...
    met_v.RMSE, ...
    met_v.MaxAbs, ...
    met_v.ErroPico_pct, ...
    met_v.NRMSE_pct, ...
    tensao_nominal, ...
    tensao_falta_permanente, ...
    t_pickup_27_inst, ...
    t_trip_27_inst, ...
    atraso_trip_27_inst, ...
    t_pickup_27_temp, ...
    t_trip_27_temp, ...
    atraso_trip_27_temp, ...
    'VariableNames', {'Grandeza','MAE','RMSE','MaxAbs','Erro_relativo_pico_percent','NRMSE_percent','Tensao_nominal_V','Tensao_falta_permanente_V','Pickup_27_instantaneo_s','Trip_27_instantaneo_s','Atraso_trip_27_instantaneo_s','Pickup_27_temporizado_s','Trip_27_temporizado_s','Atraso_trip_27_temporizado_s'} );

arquivo_metricas = sprintf('metricas_erro_tensao_subtensao_%s.csv', nivel_subtensao);
writetable(TabelaErro, arquivo_metricas);
fprintf('Tabela de métricas salva em: %s\n', fullfile(pwd, arquivo_metricas));

arquivo_relatorio = sprintf('relatorio_tensao_subtensao_%s.txt', nivel_subtensao);
fid = fopen(arquivo_relatorio, 'w');
if fid < 0
    warning('Nao foi possivel criar o relatorio TXT: %s', arquivo_relatorio);
else
    fprintf(fid, 'Relatório de comparação - Subtensão %s%%\n', nivel_subtensao);
    fprintf(fid, 'Gerado em: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

    fprintf(fid, 'Arquivos usados:\n');
    fprintf(fid, '  MAT: %s\n', arquivo_mat);
    fprintf(fid, '  CSV: %s\n', arquivo_csv);
    fprintf(fid, '  Figura: %s\n', arquivo_png);
    fprintf(fid, '  Figura PDF: %s\n', arquivo_pdf);
    fprintf(fid, '  Tabela de métricas: %s\n\n', arquivo_metricas);

    fprintf(fid, 'Parâmetros gerais:\n');
    fprintf(fid, '  Tfinal = %.9g s\n', Tfinal);
    fprintf(fid, '  Nref = %d amostras\n', Nref);
    fprintf(fid, '  Método de interpolação = %s\n', metodo_interp);
    fprintf(fid, '  Coluna de tempo CSV = %s\n', col_csv_tempo);
    fprintf(fid, '  Coluna de tensão FDT CSV = %s\n', col_csv_v_fdt);
    fprintf(fid, '  Converter MAT pico -> RMS = %d\n', converter_mat_pico_para_rms);
    fprintf(fid, '  Suavizar CSV = %d\n', suavizar_csv);
    fprintf(fid, '  Janela de média móvel CSV = %d amostras\n', janela_media_csv);
    fprintf(fid, '  Alinhar CSV por amplitude temporal = %d\n', alinhar_csv_por_amplitude_temporal);
    fprintf(fid, '  Referência do alinhamento = degrau de queda da tensão FDT no MAT\n\n');

    fprintf(fid, 'Variáveis MAT usadas:\n');
    fprintf(fid, '  Tensão: %s, canal %d\n\n', nome_mat_v, canal_mat_v);

    fprintf(fid, 'Alinhamento temporal:\n');
    if alinhar_csv_por_amplitude_temporal
        fprintf(fid, '  Degrau MAT = %.9f s\n', t_degrau_mat);
        fprintf(fid, '  Degrau FDT CSV = %.9f s\n', t_degrau_fdt_csv);
        fprintf(fid, '  Deslocamento aplicado no FDT CSV = %.9f s = %.3f ms\n', ...
            delta_csv, 1e3 * delta_csv);
        fprintf(fid, '  Regra aplicada: t_csv_corrigido = t_csv + deslocamento\n\n');
    else
        fprintf(fid, '  Alinhamento desativado. Deslocamento aplicado = 0 s\n\n');
    end

    fprintf(fid, 'Proteção 27:\n');
    fprintf(fid, '  Tensão nominal = %.9g V\n', tensao_nominal);
    fprintf(fid, '  Tensão em regime permanente da falta = %.9g V\n', tensao_falta_permanente);
    fprintf(fid, '  Pickup 27 instantâneo = %.9g V\n', pickup27_inst);
    fprintf(fid, '  Pickup 27 temporizado = %.9g V\n', pickup27_temp);
    fprintf(fid, '  Primeiro pickup 27 instantâneo = %.9f s\n', t_pickup_27_inst);
    fprintf(fid, '  Primeiro trip 27 instantâneo = %.9f s\n', t_trip_27_inst);
    fprintf(fid, '  Atraso trip 27 instantâneo após pickup = %.9f s\n', atraso_trip_27_inst);
    fprintf(fid, '  Primeiro pickup 27 temporizado = %.9f s\n', t_pickup_27_temp);
    fprintf(fid, '  Primeiro trip 27 temporizado = %.9f s\n', t_trip_27_temp);
    fprintf(fid, '  Atraso trip 27 temporizado após pickup = %.9f s\n\n', atraso_trip_27_temp);

    fprintf(fid, 'Resumo da comparação da tensão FDT usando Simulink como referência:\n');
    escrever_resumo_erro(fid, 'Tensão FDT CSV x MAT', v_csv_i, v_mat_i);

    fprintf(fid, '\nErro percentual da tensão FDT entre Simulink e ESP32 receptora:\n');
    fprintf(fid, '  MAE                   = %.6g V\n',  met_v.MAE);
    fprintf(fid, '  RMSE                  = %.6g V\n',  met_v.RMSE);
    fprintf(fid, '  MaxAbs                = %.6g V\n',  met_v.MaxAbs);
    fprintf(fid, '  Erro relativo ao pico = %.6f %%\n', met_v.ErroPico_pct);
    fprintf(fid, '  NRMSE                 = %.6f %%\n', met_v.NRMSE_pct);

    fclose(fid);
    fprintf('Relatório TXT salvo em: %s\n', fullfile(pwd, arquivo_relatorio));
end

%% ===================== FUNCOES LOCAIS =====================
function checar_coluna(T, col, arquivo)
    if ~ismember(col, T.Properties.VariableNames)
        error('A coluna "%s" nao existe em %s. Colunas disponiveis:\n%s', ...
            col, arquivo, strjoin(T.Properties.VariableNames, ', '));
    end
end

function t = obter_tempo_csv(T, col_preferida)
    nomes = T.Properties.VariableNames;
    if ismember(col_preferida, nomes)
        t = double(T.(col_preferida));
    elseif ismember('dev_t_s', nomes)
        t = double(T.dev_t_s);
    elseif ismember('dev_t_us', nomes)
        t = double(T.dev_t_us) / 1e6;
    else
        t = (0:height(T)-1).';
    end
    t = t(:);
end

function trip = obter_trip(T, col)
    if ismember(col, T.Properties.VariableNames)
        trip = double(T.(col));
    else
        trip = zeros(height(T), 1);
    end
    trip = trip(:) > 0.5;
end

function v_temporal = obter_tensao_temporal_csv(T)
    if ismember('v_inst', T.Properties.VariableNames)
        v_temporal = double(T.v_inst);
        if ismember('norm_gain_v', T.Properties.VariableNames)
            v_temporal = v_temporal .* double(T.norm_gain_v);
        end
    elseif ismember('v_rms', T.Properties.VariableNames)
        v_temporal = double(T.v_rms);
    elseif ismember('v1_mag', T.Properties.VariableNames)
        v_temporal = double(T.v1_mag);
    else
        error('Nao encontrei v_inst, v_rms nem v1_mag para estimar o deslocamento temporal.');
    end
    v_temporal = v_temporal(:);
end

function valor = obter_constante_csv(T, col)
    valor = NaN;
    if ~ismember(col, T.Properties.VariableNames)
        return;
    end
    x = double(T.(col));
    x = x(isfinite(x) & x > 0);
    if isempty(x)
        return;
    end
    valor = median(x, 'omitnan');
end

function y_rms = rms_movel(y, janela)
    y = double(y(:));
    janela = max(1, round(janela));
    if mod(janela, 2) == 0
        janela = janela + 1;
    end
    y_rms = sqrt(media_movel(y.^2, janela));
end

function t_degrau = estimar_maior_degrau_subida(t, y)
    t = double(t(:));
    y = double(y(:));
    mask = isfinite(t) & isfinite(y);
    t = t(mask);
    y = y(mask);

    t_degrau = NaN;
    if numel(t) < 20
        return;
    end

    y_s = media_movel(y, max(7, round(0.01 * numel(y))));
    dy = diff(y_s);
    t_mid = 0.5 * (t(1:end-1) + t(2:end));

    % Ignora energizacao/inicializacao e bordas da janela.
    busca = t_mid >= 0.12 & t_mid <= (t(end) - 0.05);
    if nnz(busca) < 5
        busca = true(size(dy));
    end

    dy_busca = dy;
    dy_busca(~busca) = -Inf;
    [maior_subida, idx] = max(dy_busca);
    if ~isfinite(maior_subida) || maior_subida <= 0
        return;
    end

    y_antes = y_s(idx);
    y_depois = y_s(idx + 1);
    limiar = 0.5 * (y_antes + y_depois);
    if y_depois == y_antes
        t_degrau = t_mid(idx);
    else
        t_degrau = t(idx) + (limiar - y_antes) * (t(idx + 1) - t(idx)) / (y_depois - y_antes);
    end
end

function t_degrau = estimar_tempo_degrau_subida(t, y)
    t = double(t(:));
    y = double(y(:));
    mask = isfinite(t) & isfinite(y);
    t = t(mask);
    y = y(mask);

    t_degrau = NaN;
    if numel(t) < 20
        return;
    end

    y = media_movel(y, max(5, round(0.01 * numel(y))));

    janela_pre = t >= 0.10 & t <= 0.40;
    janela_falta = t >= 0.60 & t <= 0.95;
    if nnz(janela_pre) < 5 || nnz(janela_falta) < 5
        janela_pre = t <= prctile(t, 30);
        janela_falta = t >= prctile(t, 40) & t <= prctile(t, 70);
    end

    nivel_pre = median(y(janela_pre), 'omitnan');
    nivel_falta = median(y(janela_falta), 'omitnan');
    if ~isfinite(nivel_pre) || ~isfinite(nivel_falta) || nivel_falta <= nivel_pre
        return;
    end

    limiar = 0.5 * (nivel_pre + nivel_falta);
    busca = t >= 0.20 & t <= 0.80;
    idx_busca = find(busca);
    if isempty(idx_busca)
        return;
    end

    idx_rel = find(y(idx_busca) >= limiar, 1, 'first');
    if isempty(idx_rel)
        return;
    end

    idx = idx_busca(idx_rel);
    if idx <= 1
        t_degrau = t(idx);
        return;
    end

    t1 = t(idx - 1);
    t2 = t(idx);
    y1 = y(idx - 1);
    y2 = y(idx);
    if y2 == y1
        t_degrau = t2;
    else
        t_degrau = t1 + (limiar - y1) * (t2 - t1) / (y2 - y1);
    end
end

function t = normalizar_tempo(t, Tfinal)
    t = double(t(:));
    if isempty(t)
        error('Eixo de tempo vazio.');
    end
    t = t - t(1);
    if numel(t) <= 1
        t = 0;
        return;
    end
    dur = t(end) - t(1);
    if dur > 0
        t = t * (Tfinal / dur);
    else
        t = linspace(0, Tfinal, numel(t)).';
    end
end

function [t, y, nome_usado] = extrair_sinal_mat(S, tipo, canal, Tfinal)
    nomes = fieldnames(S);
    nomes = nomes(~startsWith(nomes, '__'));

    if strcmpi(tipo, 'tensao')
        prioridade = {'Vin', 'Vin_saida', 'VinFasor', 'Vout', 'Va', 'Vabc', 'tensao', 'voltage'};
        palavras_boas = {'vin', 'vout', 'tens', 'volt', 'va', 'vabc'};
        palavras_ruins = {'iin', 'iout', 'corr', 'current', 'ia', 'iabc'};
    else
        prioridade = {'Iin', 'Iin_saida', 'IinFasor', 'Iout', 'Ia', 'Iabc', 'corrente', 'current'};
        palavras_boas = {'iin', 'iout', 'corr', 'current', 'ia', 'iabc'};
        palavras_ruins = {'vin', 'vout', 'tens', 'volt', 'va', 'vabc'};
    end

    candidatos = [intersect(prioridade, nomes, 'stable'); setdiff(nomes, prioridade, 'stable')];
    melhor_score = -Inf;
    melhor = struct('t', [], 'D', [], 'nome', '');

    for k = 1:numel(candidatos)
        nome = candidatos{k};
        [ok, t0, D0] = tentar_extrair_matriz(S.(nome), Tfinal);
        if ~ok || isempty(D0) || size(D0, 1) < 2
            continue;
        end

        nlow = lower(nome);
        score = 0;
        if any(strcmp(nome, prioridade)), score = score + 100; end
        for p = 1:numel(palavras_boas)
            if contains(nlow, palavras_boas{p}), score = score + 10; end
        end
        for p = 1:numel(palavras_ruins)
            if contains(nlow, palavras_ruins{p}), score = score - 50; end
        end

        if score > melhor_score
            melhor_score = score;
            melhor.t = t0;
            melhor.D = D0;
            melhor.nome = nome;
        end
    end

    if isempty(melhor.D)
        disp('Variáveis encontradas no MAT:');
        disp(nomes);
        error('Nao consegui encontrar variavel de %s no MAT.', tipo);
    end
    if canal > size(melhor.D, 2)
        error('Variavel "%s" possui %d canal(is), mas foi pedido canal %d.', ...
            melhor.nome, size(melhor.D, 2), canal);
    end

    t = melhor.t(:);
    y = double(melhor.D(:, canal));
    nome_usado = melhor.nome;
end

function [ok, t, D] = tentar_extrair_matriz(v, Tfinal)
    ok = false; t = []; D = [];

    if isa(v, 'timeseries')
        t = v.Time(:);
        D = organizar_matriz(squeeze(v.Data));
        ok = true;
        return;
    end

    if isstruct(v)
        if all(isfield(v, {'Time', 'Data'}))
            t = v.Time(:);
            D = organizar_matriz(squeeze(v.Data));
            ok = true;
            return;
        end
        if isfield(v, 'time') && isfield(v, 'signals')
            t = v.time(:);
            D = organizar_matriz(v.signals.values);
            ok = true;
            return;
        end
        if isfield(v, 'Values') && isa(v.Values, 'timeseries')
            t = v.Values.Time(:);
            D = organizar_matriz(squeeze(v.Values.Data));
            ok = true;
            return;
        end
    end

    if isnumeric(v) && numel(v) > 1
        v = squeeze(v);
        if ismatrix(v)
            if size(v, 2) >= 2 && is_monotonic_time(v(:, 1))
                t = v(:, 1);
                D = v(:, 2:end);
            else
                D = organizar_matriz(v);
                t = linspace(0, Tfinal, size(D, 1)).';
            end
            ok = true;
        end
    end
end

function tf = is_monotonic_time(x)
    x = x(:);
    tf = numel(x) > 3 && all(diff(x) > 0) && x(1) >= 0;
end

function D = organizar_matriz(D)
    D = squeeze(D);
    if isvector(D)
        D = D(:);
        return;
    end
    if size(D, 1) < size(D, 2)
        D = D.';
    end
end

function plotar_trips_27(ax, t, trip27_inst, trip27_temp, Tfinal)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');

    trilhas = {
        '27 (temporizado)',    trip27_temp, 0.0, [0.494, 0.184, 0.556];
        '27 (instantâneo)',    trip27_inst, 2.0, [0.929, 0.494, 0.133];
    };

    altura = 1.0;
    h_leg = gobjects(1, size(trilhas, 1));
    for k = 1:size(trilhas, 1)
        nome = trilhas{k, 1};
        valores = double(trilhas{k, 2});
        y0 = trilhas{k, 3};
        cor = trilhas{k, 4};
        ativo = y0 + altura * valores;

        plot(ax, [0, Tfinal], [y0, y0], ':', ...
            'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.7, ...
            'HandleVisibility', 'off');
        plot(ax, [0, Tfinal], [y0 + altura, y0 + altura], ':', ...
            'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.7, ...
            'HandleVisibility', 'off');
        preencher_trip_acionado(ax, t, valores, y0, altura, cor);
        h_leg(k) = stairs(ax, t, ativo, 'Color', cor, 'LineWidth', 1.5, 'DisplayName', nome);
        text(ax, Tfinal + 0.015 * Tfinal, y0, '0', ...
            'Color', [0.35, 0.35, 0.35], 'VerticalAlignment', 'middle');
        text(ax, Tfinal + 0.015 * Tfinal, y0 + altura, '1', ...
            'Color', [0.35, 0.35, 0.35], 'VerticalAlignment', 'middle');
    end

    ylim(ax, [-0.35, 3.35]);
    yticks(ax, [0.5, 2.5]);
    yticklabels(ax, {'27', '27'});
    ylabel(ax, 'Trip');
    title(ax, 'Status de trip da função 27');
    legend(ax, h_leg([2, 1]), {'27 (instantâneo)', '27 (temporizado)'}, ...
        'Location', 'northeast', 'FontSize', 8);
    hold(ax, 'off');
end

function preencher_trip_acionado(ax, t, valores, y0, altura, cor)
    valores = double(valores(:) > 0.5);
    t = double(t(:));
    if isempty(t) || numel(t) ~= numel(valores)
        return;
    end

    bordas = diff([0; valores; 0]);
    inicios = find(bordas == 1);
    fins = find(bordas == -1) - 1;

    for idx = 1:numel(inicios)
        t_ini = t(inicios(idx));
        t_fim = t(fins(idx));
        if t_fim <= t_ini
            continue;
        end
        patch(ax, ...
            [t_ini, t_fim, t_fim, t_ini], ...
            [y0, y0, y0 + altura, y0 + altura], ...
            cor, ...
            'FaceAlpha', 0.12, ...
            'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
    end
end

function y_suave = media_movel(y, janela)
    y = double(y(:));
    janela = max(1, round(janela));
    if janela <= 1 || numel(y) < 3
        y_suave = y;
        return;
    end
    if mod(janela, 2) == 0
        janela = janela + 1;
    end

    if exist('movmean', 'file') == 2 || exist('movmean', 'builtin') == 5
        y_suave = movmean(y, janela, 'Endpoints', 'shrink');
        return;
    end

    meio = floor(janela / 2);
    y_suave = zeros(size(y));
    for k = 1:numel(y)
        ini = max(1, k - meio);
        fim = min(numel(y), k + meio);
        y_suave(k) = mean(y(ini:fim), 'omitnan');
    end
end

function cor_out = aplicar_transparencia(cor, transparencia)
    transparencia = min(max(transparencia, 0), 1);
    fundo = [1, 1, 1];
    cor_out = (1 - transparencia) .* cor + transparencia .* fundo;
end

function lim = limite_y_amplitude_zero(valores)
    valores = double(valores(:));
    valores = valores(isfinite(valores) & valores >= 0);
    if isempty(valores)
        lim = [0, 1];
        return;
    end

    y_max = max(valores, [], 'omitnan');
    if ~isfinite(y_max) || y_max <= 0
        y_max = 1;
    end
    lim = [0, 1.08 * y_max];
end

function t_trip = primeiro_instante_trip(t, trip)
    t = double(t(:));
    trip = double(trip(:)) > 0.5;
    idx = find(trip & isfinite(t), 1, 'first');
    if isempty(idx)
        t_trip = NaN;
    else
        t_trip = t(idx);
    end
end

function t_pickup = primeiro_cruzamento_pickup(t, grandeza, pickup)
    t = double(t(:));
    grandeza = double(grandeza(:));
    t_pickup = NaN;
    if ~isfinite(pickup)
        return;
    end
    mask = isfinite(t) & isfinite(grandeza);
    t = t(mask);
    grandeza = grandeza(mask);
    if numel(t) < 2
        return;
    end

    idx = find(grandeza <= pickup, 1, 'first');
    if isempty(idx)
        return;
    end
    if idx <= 1
        t_pickup = t(idx);
        return;
    end

    t1 = t(idx - 1);
    t2 = t(idx);
    y1 = grandeza(idx - 1);
    y2 = grandeza(idx);
    if y2 == y1
        t_pickup = t2;
    else
        t_pickup = t1 + (pickup - y1) * (t2 - t1) / (y2 - y1);
    end
end

function tensao = estimar_tensao_falta_permanente(t, v_csv, v_ref)
    t = double(t(:));
    v_csv = double(v_csv(:));
    v_ref = double(v_ref(:));
    tensao = NaN;

    mask = isfinite(t) & isfinite(v_csv) & isfinite(v_ref);
    if nnz(mask) < 20
        return;
    end

    t_valid = t(mask);
    v_csv_valid = v_csv(mask);
    v_ref_valid = v_ref(mask);

    t_descida = estimar_tempo_degrau_descida(t_valid, v_ref_valid);
    t_retorno = estimar_tempo_retorno_subida(t_valid, v_ref_valid);
    if ~isfinite(t_descida)
        return;
    end
    if ~isfinite(t_retorno) || t_retorno <= t_descida
        t_retorno = t_valid(end);
    end

    margem = max(0.03, 0.10 * (t_retorno - t_descida));
    ini = t_descida + margem;
    fim = t_retorno - margem;
    if fim <= ini
        ini = t_descida;
        fim = t_retorno;
    end

    janela = t_valid >= ini & t_valid <= fim;
    if nnz(janela) < 5
        return;
    end
    tensao = median(v_csv_valid(janela), 'omitnan');
end

function t_degrau = estimar_tempo_degrau_descida(t, y)
    t = double(t(:));
    y = double(y(:));
    mask = isfinite(t) & isfinite(y);
    t = t(mask);
    y = y(mask);

    t_degrau = NaN;
    if numel(t) < 20
        return;
    end

    y = media_movel(y, max(5, round(0.01 * numel(y))));

    janela_pre = t >= 0.10 & t <= 0.40;
    janela_falta = t >= 0.60 & t <= 0.95;
    if nnz(janela_pre) < 5 || nnz(janela_falta) < 5
        janela_pre = t <= prctile(t, 30);
        janela_falta = t >= prctile(t, 40) & t <= prctile(t, 70);
    end

    nivel_pre = median(y(janela_pre), 'omitnan');
    nivel_falta = median(y(janela_falta), 'omitnan');
    if ~isfinite(nivel_pre) || ~isfinite(nivel_falta) || nivel_falta >= nivel_pre
        return;
    end

    limiar = 0.5 * (nivel_pre + nivel_falta);
    busca = t >= 0.20 & t <= 0.80;
    idx_busca = find(busca);
    if isempty(idx_busca)
        return;
    end

    idx_rel = find(y(idx_busca) <= limiar, 1, 'first');
    if isempty(idx_rel)
        return;
    end

    idx = idx_busca(idx_rel);
    if idx <= 1
        t_degrau = t(idx);
        return;
    end

    t1 = t(idx - 1);
    t2 = t(idx);
    y1 = y(idx - 1);
    y2 = y(idx);
    if y2 == y1
        t_degrau = t2;
    else
        t_degrau = t1 + (limiar - y1) * (t2 - t1) / (y2 - y1);
    end
end

function t_degrau = estimar_tempo_retorno_subida(t, y)
    t = double(t(:));
    y = double(y(:));
    mask = isfinite(t) & isfinite(y);
    t = t(mask);
    y = y(mask);

    t_degrau = NaN;
    if numel(t) < 20
        return;
    end

    y = media_movel(y, max(5, round(0.01 * numel(y))));

    janela_pre = t >= 0.10 & t <= 0.40;
    janela_falta = t >= 0.60 & t <= 0.95;
    if nnz(janela_pre) < 5 || nnz(janela_falta) < 5
        janela_pre = t <= prctile(t, 30);
        janela_falta = t >= prctile(t, 40) & t <= prctile(t, 70);
    end

    nivel_pre = median(y(janela_pre), 'omitnan');
    nivel_falta = median(y(janela_falta), 'omitnan');
    if ~isfinite(nivel_pre) || ~isfinite(nivel_falta) || nivel_falta >= nivel_pre
        return;
    end

    limiar = 0.5 * (nivel_pre + nivel_falta);
    busca = t >= 0.75 & t <= 1.30;
    idx_busca = find(busca);
    if isempty(idx_busca)
        return;
    end

    idx_rel = find(y(idx_busca) >= limiar, 1, 'first');
    if isempty(idx_rel)
        return;
    end

    idx = idx_busca(idx_rel);
    if idx <= 1
        t_degrau = t(idx);
        return;
    end

    t1 = t(idx - 1);
    t2 = t(idx);
    y1 = y(idx - 1);
    y2 = y(idx);
    if y2 == y1
        t_degrau = t2;
    else
        t_degrau = t1 + (limiar - y1) * (t2 - t1) / (y2 - y1);
    end
end

function escrever_resumo_erro(fid, nome, y, yref)
    y = y(:);
    yref = yref(:);
    valido = isfinite(y) & isfinite(yref);
    y = y(valido);
    yref = yref(valido);
    erro = y - yref;
    rmse = sqrt(mean(erro.^2, 'omitnan'));
    mae = mean(abs(erro), 'omitnan');
    emax = max(abs(erro), [], 'omitnan');
    pico_ref = max(abs(yref), [], 'omitnan');
    if pico_ref > 0
        erro_pico_pct = 100 * rmse / pico_ref;
    else
        erro_pico_pct = NaN;
    end
    fprintf(fid, '  %-24s | RMSE = %.6g | MAE = %.6g | Max = %.6g | Erro pico = %.6f %%\n', ...
        nome, rmse, mae, emax, erro_pico_pct);
end

function met = metricas_erro_percentual(y, yref)
    y = y(:);
    yref = yref(:);
    valido = isfinite(y) & isfinite(yref);
    y = y(valido);
    yref = yref(valido);
    erro = y - yref;
    met.MAE = mean(abs(erro), 'omitnan');
    met.RMSE = sqrt(mean(erro.^2, 'omitnan'));
    met.MaxAbs = max(abs(erro), [], 'omitnan');
    pico_ref = max(abs(yref), [], 'omitnan');
    faixa_ref = max(yref, [], 'omitnan') - min(yref, [], 'omitnan');
    if pico_ref > 0
        met.ErroPico_pct = 100 * met.RMSE / pico_ref;
    else
        met.ErroPico_pct = NaN;
    end
    if faixa_ref > 0
        met.NRMSE_pct = 100 * met.RMSE / faixa_ref;
    else
        met.NRMSE_pct = NaN;
    end
end
