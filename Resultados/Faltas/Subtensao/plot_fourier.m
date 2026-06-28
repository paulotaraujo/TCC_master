%% plot_comparacao_fdt_fourier_pico_para_rms.m
% Compara os valores de Fourier/FDT de tensao e corrente obtidos em:
%   1) MAT: trifasico_fourier_10.mat
%   2) CSV: trimmed_trifasico_oscilografia_10.csv
%
% O CSV usa, por padrao, as colunas:
%   v1_mag  -> modulo RMS da fundamental de tensao calculada na ESP32
%   i1_mag  -> modulo RMS da fundamental de corrente calculada na ESP32
%
% O MAT e lido automaticamente. O script procura variaveis relacionadas a
% tensao/corrente e Fourier/FDT. Caso a selecao automatica nao seja a correta,
% configure manualmente as variaveis na secao CONFIGURACAO.
%
% Saidas:
%   figuras_fourier_fdt/comparacao_fdt_tensao.png/.pdf
%   figuras_fourier_fdt/comparacao_fdt_corrente.png/.pdf
%   figuras_fourier_fdt/metricas_erro_fdt_simulink_esp32.csv
%   figuras_fourier_fdt/relatorio_saida_fdt.txt

clear; clc; close all;

%% ===================== CONFIGURACAO =====================
arquivo_mat = ['faseA_fourier_subtensao_15.mat'];
arquivo_csv = 'trimmed_sobretensao_15_embedded.csv';

Tfinal = 2.0;
Nesperado = 2001;
t_ref = linspace(0,Tfinal,Nesperado).';

% Colunas de tempo e Fourier no CSV.
col_csv_tempo = 'host_t_s';
col_csv_v_fdt = 'v1_mag';
col_csv_i_fdt = 'i1_mag';

% Para reproduzir o comportamento do read_adc_capture_csv.py nos modos
% fourier_scaled/rms_scaled, normalmente se usam apenas atualizacoes validas.
usar_apenas_cycle_update = true;
filtrar_fourier_valid = true;
filtrar_norm_ready = true;

% Se quiser forcar nomes especificos do MAT, preencha abaixo.
% Exemplo:
%   var_mat_v = 'V1_mag_saida';
%   var_mat_i = 'I1_mag_saida';
% Caso deixe vazio, o script tenta detectar automaticamente.
var_mat_v = '';
var_mat_i = '';

% Canal dentro da variavel MAT selecionada.
canal_mat_v = 1;
canal_mat_i = 1;

% Alinhamento automatico do CSV em relacao ao MAT.
% Para valores Fourier/FDT, geralmente o atraso e pequeno. Deixe true se as
% curvas de magnitude estiverem deslocadas no tempo.
alinhar_csv = true;
janela_alinhamento = [0 2.0];
limite_deslocamento = 2e-3;
metodo_interp = 'pchip';

salvar_figuras = true;
pasta_saida = 'figuras_fourier_fdt';

% Rotulos dos eixos.
ylabel_v = 'FDT de tensão escalonada (V RMS)';
ylabel_i = 'FDT de corrente escalonada(A RMS)';

% O bloco Fourier/FDT do Simulink pode exportar magnitude de pico.
% A ESP32/receptor normalmente registra a magnitude RMS da fundamental.
% Se as curvas do MAT estiverem cerca de sqrt(2) maiores que o CSV,
% mantenha esta opcao como true para converter pico -> RMS.
corrigir_mat_pico_para_rms = true;

%% ===================== PREPARACAO =====================
if salvar_figuras && ~exist(pasta_saida,'dir')
    mkdir(pasta_saida);
end

%% ===================== LEITURA DO CSV =====================
Tcsv = readtable(arquivo_csv, 'VariableNamingRule','preserve');

if ismember(col_csv_tempo, Tcsv.Properties.VariableNames)
    t_csv = Tcsv.(col_csv_tempo);
elseif ismember('dev_t_s', Tcsv.Properties.VariableNames)
    t_csv = Tcsv.dev_t_s;
else
    t_csv = (0:height(Tcsv)-1).' / 1000;
end

checar_coluna(Tcsv, col_csv_v_fdt, arquivo_csv);
checar_coluna(Tcsv, col_csv_i_fdt, arquivo_csv);

v_csv = Tcsv.(col_csv_v_fdt);
i_csv = Tcsv.(col_csv_i_fdt);

% Filtros equivalentes ao modo fourier_scaled do read_adc_capture_csv.py.
mask_csv = true(height(Tcsv),1);
if usar_apenas_cycle_update && ismember('cycle_update', Tcsv.Properties.VariableNames)
    mask_csv = mask_csv & (Tcsv.cycle_update == 1);
end
if filtrar_fourier_valid && ismember('fourier_valid', Tcsv.Properties.VariableNames)
    mask_csv = mask_csv & (Tcsv.fourier_valid == 1);
end
if filtrar_norm_ready && ismember('norm_ready', Tcsv.Properties.VariableNames)
    mask_csv = mask_csv & (Tcsv.norm_ready == 1);
end

t_csv_f = t_csv(mask_csv);
v_csv_f = v_csv(mask_csv);
i_csv_f = i_csv(mask_csv);

fprintf('CSV: coluna de tensao Fourier/FDT usada: %s\n', col_csv_v_fdt);
fprintf('CSV: coluna de corrente Fourier/FDT usada: %s\n', col_csv_i_fdt);
fprintf('CSV: amostras totais = %d | amostras usadas apos filtros = %d\n', height(Tcsv), numel(t_csv_f));

%% ===================== LEITURA DO MAT =====================
S = load(arquivo_mat);

if ~isempty(var_mat_v)
    [t_mat_v, v_mat, nome_mat_v] = extrair_variavel_mat_por_nome(S, var_mat_v, canal_mat_v, Tfinal);
else
    [t_mat_v, v_mat, nome_mat_v] = extrair_sinal_mat_fdt_por_referencia(S, 'tensao', canal_mat_v, Tfinal, v_csv_f);
end

if ~isempty(var_mat_i)
    [t_mat_i, i_mat, nome_mat_i] = extrair_variavel_mat_por_nome(S, var_mat_i, canal_mat_i, Tfinal);
else
    [t_mat_i, i_mat, nome_mat_i] = extrair_sinal_mat_fdt_por_referencia(S, 'corrente', canal_mat_i, Tfinal, i_csv_f);
end

%% ===================== CONVERSAO PICO -> RMS DO MAT =====================
% Se o Fourier/FDT do Simulink saiu como magnitude de pico, converte para RMS.
% Isso corrige a discrepancia classica de fator sqrt(2):
%   Vpico/sqrt(2) = Vrms
%   Ipico/sqrt(2) = Irms
if corrigir_mat_pico_para_rms
    v_mat = v_mat ./ sqrt(2);
    i_mat = i_mat ./ sqrt(2);
    fprintf('MAT: aplicado fator 1/sqrt(2) em tensao FDT e corrente FDT para converter pico -> RMS.\n');
else
    fprintf('MAT: fator pico -> RMS nao aplicado.\n');
end

% Se o MAT tiver exatamente 2001 pontos, assume eixo 0..2 s.
if numel(t_mat_v) == Nesperado, t_mat_v = t_ref; end
if numel(t_mat_i) == Nesperado, t_mat_i = t_ref; end

%% ===================== EIXO COMUM E ALINHAMENTO =====================
% Para Fourier/FDT, o CSV normalmente possui menos pontos quando se usa
% cycle_update. Por isso, o eixo comum e definido pela referencia MAT.
t_plot = t_ref;
v_mat_i = interp1(t_mat_v(:), v_mat(:), t_plot, metodo_interp, 'extrap');
i_mat_i = interp1(t_mat_i(:), i_mat(:), t_plot, metodo_interp, 'extrap');

% Alinha o CSV pela curva de tensao e aplica o mesmo deslocamento na corrente.
delta_csv = 0;
if alinhar_csv && numel(t_csv_f) >= 5
    delta_csv = estimar_deslocamento_fracionario(t_csv_f(:), v_csv_f(:), t_plot, v_mat_i(:), ...
        janela_alinhamento, limite_deslocamento, metodo_interp);
end

t_csv_alinhado = t_csv_f(:) + delta_csv;
v_csv_i = interp1(t_csv_alinhado, v_csv_f(:), t_plot, metodo_interp, 'extrap');
i_csv_i = interp1(t_csv_alinhado, i_csv_f(:), t_plot, metodo_interp, 'extrap');

%% ===================== PLOTS =====================
plotar_fdt(t_plot, v_mat_i, v_csv_i, ...
    'Comparação dos valores FDT da tensão', ylabel_v, ...
    'FDT de tensão do Simulink', ...
    'FDT de tensão da ESP32 receptora', ...
    pasta_saida, 'comparacao_fdt_tensao', salvar_figuras);

plotar_fdt(t_plot, i_mat_i, i_csv_i, ...
    'Comparação dos valores FDT da corrente', ylabel_i, ...
    'FDT de corrente do Simulink', ...
    'FDT de corrente da ESP32 receptora', ...
    pasta_saida, 'comparacao_fdt_corrente', salvar_figuras);

%% ===================== METRICAS DE ERRO =====================
met_v = metricas_erro_percentual(v_csv_i, v_mat_i);
met_i = metricas_erro_percentual(i_csv_i, i_mat_i);

fprintf('\nAlinhamento automatico do CSV Fourier/FDT:\n');
fprintf('  Deslocamento aplicado ao tempo do CSV: %.9f s = %.3f ms\n', delta_csv, delta_csv*1e3);
fprintf('  Regra usada: t_csv_alinhado = t_csv + deslocamento\n');

fprintf('\nVariaveis MAT usadas:\n');
fprintf('  Tensao FDT  : %s, canal %d\n', nome_mat_v, canal_mat_v);
fprintf('  Corrente FDT: %s, canal %d\n', nome_mat_i, canal_mat_i);

fprintf('\nResumo da comparacao FDT usando o Simulink como referencia:\n');
resumo_erro('FDT Tensao CSV x MAT', v_csv_i, v_mat_i);
resumo_erro('FDT Corrente CSV x MAT', i_csv_i, i_mat_i);

fprintf('\n============================================================\n');
fprintf('Erro percentual FDT entre Simulink e ESP32 receptora\n');
fprintf('Referencia: Simulink | Sinal comparado: ESP32 receptora\n');
fprintf('============================================================\n');
fprintf('Tensao FDT:\n');
fprintf('  MAE                   = %.6g V\n',  met_v.MAE);
fprintf('  RMSE                  = %.6g V\n',  met_v.RMSE);
fprintf('  Erro relativo ao pico = %.6f %%\n', met_v.ErroPico_pct);
fprintf('  NRMSE                 = %.6f %%\n', met_v.NRMSE_pct);

fprintf('\nCorrente FDT:\n');
fprintf('  MAE                   = %.6g A\n',  met_i.MAE);
fprintf('  RMSE                  = %.6g A\n',  met_i.RMSE);
fprintf('  Erro relativo ao pico = %.6f %%\n', met_i.ErroPico_pct);
fprintf('  NRMSE                 = %.6f %%\n', met_i.NRMSE_pct);

TabelaErro = table( ...
    {'Tensao FDT'; 'Corrente FDT'}, ...
    [met_v.MAE; met_i.MAE], ...
    [met_v.RMSE; met_i.RMSE], ...
    [met_v.ErroPico_pct; met_i.ErroPico_pct], ...
    [met_v.NRMSE_pct; met_i.NRMSE_pct], ...
    'VariableNames', {'Grandeza','MAE','RMSE','Erro_relativo_pico_percent','NRMSE_percent'} );

if salvar_figuras
    arquivo_tabela = fullfile(pasta_saida, 'metricas_erro_fdt_simulink_esp32.csv');
else
    arquivo_tabela = 'metricas_erro_fdt_simulink_esp32.csv';
end
writetable(TabelaErro, arquivo_tabela);
fprintf('\nTabela de erro salva em: %s\n', arquivo_tabela);

%% ===================== RELATORIO TXT =====================
if salvar_figuras
    arquivo_relatorio = fullfile(pasta_saida, 'relatorio_saida_fdt.txt');
else
    arquivo_relatorio = 'relatorio_saida_fdt.txt';
end

fid = fopen(arquivo_relatorio, 'w');
if fid < 0
    warning('Nao foi possivel criar o relatorio TXT: %s', arquivo_relatorio);
else
    fprintf(fid, 'CSV: coluna de tensao Fourier/FDT usada: %s\n', col_csv_v_fdt);
    fprintf(fid, 'CSV: coluna de corrente Fourier/FDT usada: %s\n', col_csv_i_fdt);
    fprintf(fid, 'CSV: amostras totais = %d | amostras usadas apos filtros = %d\n', height(Tcsv), numel(t_csv_f));
    if corrigir_mat_pico_para_rms
        fprintf(fid, 'MAT: aplicado fator 1/sqrt(2) em tensao FDT e corrente FDT para converter pico -> RMS.\n');
    else
        fprintf(fid, 'MAT: fator pico -> RMS nao aplicado.\n');
    end

    fprintf(fid, '\nAlinhamento automatico do CSV Fourier/FDT:\n');
    fprintf(fid, '  Deslocamento aplicado ao tempo do CSV: %.9f s = %.3f ms\n', delta_csv, delta_csv*1e3);
    fprintf(fid, '  Regra usada: t_csv_alinhado = t_csv + deslocamento\n');

    fprintf(fid, '\nVariaveis MAT usadas:\n');
    fprintf(fid, '  Tensao FDT  : %s, canal %d\n', nome_mat_v, canal_mat_v);
    fprintf(fid, '  Corrente FDT: %s, canal %d\n', nome_mat_i, canal_mat_i);

    fprintf(fid, '\nResumo da comparacao FDT usando o Simulink como referencia:\n');
    escrever_resumo_erro(fid, 'FDT Tensao CSV x MAT', v_csv_i, v_mat_i);
    escrever_resumo_erro(fid, 'FDT Corrente CSV x MAT', i_csv_i, i_mat_i);

    fprintf(fid, '\n============================================================\n');
    fprintf(fid, 'Erro percentual FDT entre Simulink e ESP32 receptora\n');
    fprintf(fid, 'Referencia: Simulink | Sinal comparado: ESP32 receptora\n');
    fprintf(fid, '============================================================\n');
    fprintf(fid, 'Tensao FDT:\n');
    fprintf(fid, '  MAE                   = %.6g V\n',  met_v.MAE);
    fprintf(fid, '  RMSE                  = %.6g V\n',  met_v.RMSE);
    fprintf(fid, '  Erro relativo ao pico = %.6f %%\n', met_v.ErroPico_pct);
    fprintf(fid, '  NRMSE                 = %.6f %%\n', met_v.NRMSE_pct);

    fprintf(fid, '\nCorrente FDT:\n');
    fprintf(fid, '  MAE                   = %.6g A\n',  met_i.MAE);
    fprintf(fid, '  RMSE                  = %.6g A\n',  met_i.RMSE);
    fprintf(fid, '  Erro relativo ao pico = %.6f %%\n', met_i.ErroPico_pct);
    fprintf(fid, '  NRMSE                 = %.6f %%\n', met_i.NRMSE_pct);

    fprintf(fid, '\nTabela de erro salva em: %s\n', arquivo_tabela);
    fclose(fid);
    fprintf('Relatorio TXT salvo em: %s\n', arquivo_relatorio);
end

%% ===================== FUNCOES LOCAIS =====================
function checar_coluna(T, col, arquivo)
    if ~ismember(col, T.Properties.VariableNames)
        error('A coluna "%s" nao existe em %s. Colunas disponiveis:\n%s', ...
            col, arquivo, strjoin(T.Properties.VariableNames, ', '));
    end
end

function [t, y, nome_usado] = extrair_variavel_mat_por_nome(S, nome, canal, Tfinal)
    if ~isfield(S, nome)
        error('A variavel "%s" nao existe no arquivo MAT.', nome);
    end
    [ok, t, D] = tentar_extrair_matriz(S.(nome), Tfinal);
    if ~ok
        error('Nao consegui extrair dados numericos da variavel MAT "%s".', nome);
    end
    if canal > size(D,2)
        error('A variavel MAT "%s" possui apenas %d canal(is). Canal solicitado: %d.', nome, size(D,2), canal);
    end
    y = D(:,canal);
    nome_usado = nome;
end

function [t, y, nome_usado] = extrair_sinal_mat_fdt_por_referencia(S, tipo, canal, Tfinal, y_ref_csv)
    % Seleciona a variavel MAT usando nome + ordem de grandeza do CSV.
    % Isso evita comparar tensao FDT com um canal de corrente/angulo/tempo.
    nomes = fieldnames(S);
    nomes = nomes(~startsWith(nomes,'__'));

    if strcmpi(tipo,'tensao')
        prioridade = {'V1_mag_saida','V1_saida','Vfdt_saida','Vfdt','V_fourier','V1_mag','v1_mag','Vmag','v_mag','Fourier_V','FDT_V','V_FDT','tensao_fdt','tensao_fourier','V_fundamental','V1'};
        palavras_boas = {'v1','vfdt','v_fdt','fourier_v','fdt_v','tens','volt','vmag','v_mag','fund_v'};
        palavras_ruins = {'i1','ifdt','i_fdt','corr','current','imag','i_mag','angle','ang','fase','phase','tempo','time'};
    else
        prioridade = {'I1_mag_saida','I1_saida','Ifdt_saida','Ifdt','I_fourier','I1_mag','i1_mag','Imag','i_mag','Fourier_I','FDT_I','I_FDT','corrente_fdt','corrente_fourier','I_fundamental','I1'};
        palavras_boas = {'i1','ifdt','i_fdt','fourier_i','fdt_i','corr','current','imag','i_mag','fund_i'};
        palavras_ruins = {'v1','vfdt','v_fdt','tens','volt','vmag','v_mag','angle','ang','fase','phase','tempo','time'};
    end

    amp_ref = robust_amp(y_ref_csv);
    candidatos = [intersect(prioridade, nomes, 'stable'); setdiff(nomes, prioridade, 'stable')];

    melhorScore = -Inf;
    melhor = struct('t',[],'D',[],'nome','', 'amp', NaN, 'scoreNome', NaN, 'scoreAmp', NaN);

    fprintf('\nSelecao automatica MAT para %s FDT:\n', tipo);
    fprintf('  Amplitude robusta de referencia CSV: %.6g\n', amp_ref);

    for k = 1:numel(candidatos)
        nome = candidatos{k};
        v = S.(nome);
        [ok, t0, D0] = tentar_extrair_matriz(v, Tfinal);
        if ~ok || isempty(D0) || size(D0,1) < 5 || canal > size(D0,2)
            continue;
        end

        y0 = D0(:,canal);
        amp = robust_amp(y0);
        if ~isfinite(amp) || amp <= 0
            continue;
        end

        nlow = lower(nome);
        scoreNome = 0;
        if any(strcmp(nome, prioridade)), scoreNome = scoreNome + 100; end
        for p = 1:numel(palavras_boas)
            if contains(nlow, palavras_boas{p}), scoreNome = scoreNome + 12; end
        end
        for p = 1:numel(palavras_ruins)
            if contains(nlow, palavras_ruins{p}), scoreNome = scoreNome - 80; end
        end
        if contains(nlow,'fourier') || contains(nlow,'fdt') || contains(nlow,'mag') || contains(nlow,'rms')
            scoreNome = scoreNome + 25;
        end
        if contains(nlow,'inst') || contains(nlow,'adc') || contains(nlow,'raw')
            scoreNome = scoreNome - 25;
        end

        % Compara a ordem de grandeza com a curva CSV correspondente.
        scoreAmp = 0;
        if isfinite(amp_ref) && amp_ref > 0
            ratio = amp / amp_ref;
            scoreAmp = -45*abs(log10(max(ratio, eps)));

            % Penalizacoes fortes quando a ordem de grandeza nao bate.
            if ratio < 0.05 || ratio > 20
                scoreAmp = scoreAmp - 150;
            end
        end

        score = scoreNome + scoreAmp;
        fprintf('  candidato: %-30s | amp = %.6g | score = %.2f\n', nome, amp, score);

        if score > melhorScore
            melhorScore = score;
            melhor.t = t0;
            melhor.D = D0;
            melhor.nome = nome;
            melhor.amp = amp;
            melhor.scoreNome = scoreNome;
            melhor.scoreAmp = scoreAmp;
        end
    end

    if isempty(melhor.D)
        disp('Variaveis encontradas no MAT:');
        disp(nomes);
        error('Nao consegui encontrar uma variavel FDT/Fourier de %s no arquivo MAT.', tipo);
    end

    t = melhor.t(:);
    y = melhor.D(:,canal);
    nome_usado = melhor.nome;

    fprintf('  ==> selecionado: %s | amp MAT = %.6g | amp CSV = %.6g\n', nome_usado, melhor.amp, amp_ref);

    % Aviso se ainda assim houver grande discrepancia de escala.
    if isfinite(amp_ref) && amp_ref > 0
        ratio = melhor.amp / amp_ref;
        if ratio < 0.1 || ratio > 10
            warning(['A variavel MAT selecionada para %s tem amplitude muito diferente do CSV. ', ...
                     'Confira var_mat_v/var_mat_i manualmente com: whos(''-file'',''trifasico_fourier_10.mat'')'], tipo);
        end
    end
end

function a = robust_amp(x)
    x = x(:);
    x = x(isfinite(x));
    if isempty(x)
        a = NaN;
        return;
    end
    % Usa mediana do modulo para sinais FDT/RMS e evita outliers.
    a = median(abs(x), 'omitnan');
    if a == 0
        a = prctile(abs(x), 95);
    end
end

function [t, y, nome_usado] = extrair_sinal_mat_fdt(S, tipo, canal, Tfinal)
    nomes = fieldnames(S);
    nomes = nomes(~startsWith(nomes,'__'));

    if strcmpi(tipo,'tensao')
        prioridade = {'V1_mag_saida','V1_saida','Vfdt_saida','Vfdt','V_fourier','V1_mag','v1_mag','Vmag','v_mag','Fourier_V','FDT_V','V_FDT','tensao_fdt','tensao_fourier'};
        palavras_boas = {'v1','vfdt','v_fdt','fourier_v','fdt_v','tens','volt','vmag','v_mag'};
        palavras_ruins = {'i1','ifdt','i_fdt','corr','current','imag','i_mag'};
    else
        prioridade = {'I1_mag_saida','I1_saida','Ifdt_saida','Ifdt','I_fourier','I1_mag','i1_mag','Imag','i_mag','Fourier_I','FDT_I','I_FDT','corrente_fdt','corrente_fourier'};
        palavras_boas = {'i1','ifdt','i_fdt','fourier_i','fdt_i','corr','current','imag','i_mag'};
        palavras_ruins = {'v1','vfdt','v_fdt','tens','volt','vmag','v_mag'};
    end

    candidatos = [intersect(prioridade, nomes, 'stable'); setdiff(nomes, prioridade, 'stable')];

    melhorScore = -Inf;
    melhor = struct('t',[],'D',[],'nome','');

    for k = 1:numel(candidatos)
        nome = candidatos{k};
        v = S.(nome);
        [ok, t0, D0] = tentar_extrair_matriz(v, Tfinal);
        if ~ok || isempty(D0) || size(D0,1) < 5
            continue;
        end

        nlow = lower(nome);
        score = 0;
        if any(strcmp(nome, prioridade)), score = score + 100; end
        for p = 1:numel(palavras_boas)
            if contains(nlow, palavras_boas{p}), score = score + 10; end
        end
        for p = 1:numel(palavras_ruins)
            if contains(nlow, palavras_ruins{p}), score = score - 60; end
        end
        if contains(nlow,'fourier') || contains(nlow,'fdt') || contains(nlow,'mag')
            score = score + 20;
        end

        % Evita escolher sinais instantaneos se existirem candidatos FDT.
        if contains(nlow,'inst') || contains(nlow,'adc')
            score = score - 20;
        end

        if score > melhorScore
            melhorScore = score;
            melhor.t = t0;
            melhor.D = D0;
            melhor.nome = nome;
        end
    end

    if isempty(melhor.D)
        disp('Variaveis encontradas no MAT:');
        disp(nomes);
        error('Nao consegui encontrar uma variavel FDT/Fourier de %s no arquivo MAT.', tipo);
    end

    if canal > size(melhor.D,2)
        error('A variavel MAT "%s" foi selecionada para %s, mas possui apenas %d canal(is). Canal solicitado: %d.', ...
            melhor.nome, tipo, size(melhor.D,2), canal);
    end

    t = melhor.t(:);
    y = melhor.D(:,canal);
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
        if all(isfield(v, {'Time','Data'}))
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

    if isnumeric(v) && numel(v) > 5
        v = squeeze(v);
        if ismatrix(v)
            if size(v,2) >= 2 && is_monotonic_time(v(:,1))
                t = v(:,1);
                D = v(:,2:end);
            else
                D = organizar_matriz(v);
                t = linspace(0,Tfinal,size(D,1)).';
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
    if size(D,1) < size(D,2)
        D = D.';
    end
end

function plotar_fdt(t, y_mat, y_csv, titulo, ylabel_txt, leg_mat, leg_csv, pasta, nome, salvar)
    fig = figure('Color','w', 'Units','centimeters', 'Position',[2 2 18 10]);
    hold on;

    h_mat = plot(t, y_mat, 'b-', 'LineWidth', 0.85);
    h_csv = plot(t, y_csv, 'r--', 'LineWidth', 0.95);

    grid on;
    box on;
    xlabel('Tempo (s)', 'FontName','Times New Roman', 'FontSize',12);
    ylabel(ylabel_txt, 'FontName','Times New Roman', 'FontSize',12, 'Interpreter','none');
    title(titulo, 'FontName','Times New Roman', 'FontSize',13, 'FontWeight','bold', 'Interpreter','none');
    legend([h_mat h_csv], {leg_mat, leg_csv}, ...
        'Location','best', 'FontName','Times New Roman', 'FontSize',11, ...
        'Interpreter','none', 'Box','on');
    xlim([t(1) t(end)]);
    set(gca, 'FontName','Times New Roman', 'FontSize',11, 'LineWidth',1);

    if salvar
        exportgraphics(fig, fullfile(pasta, [nome '.png']), 'Resolution',600);
        exportgraphics(fig, fullfile(pasta, [nome '.pdf']), 'ContentType','vector');
    end
end

function delta_otimo = estimar_deslocamento_fracionario(t_csv, y_csv, t_ref, y_ref, janela, limite, metodo)
    t_csv = t_csv(:); y_csv = y_csv(:);
    t_ref = t_ref(:); y_ref = y_ref(:);

    idx = t_ref >= janela(1) & t_ref <= janela(2) & isfinite(y_ref);
    tr = t_ref(idx);
    yr = y_ref(idx);
    yr0 = yr - mean(yr, 'omitnan');

    function e = obj(delta)
        yc = interp1(t_csv + delta, y_csv, tr, metodo, NaN);
        valido = isfinite(yc) & isfinite(yr0);
        if nnz(valido) < 5
            e = Inf;
            return;
        end
        yc0 = yc(valido) - mean(yc(valido), 'omitnan');
        yrv = yr0(valido);
        g = (yc0(:)'*yrv(:)) / max(yc0(:)'*yc0(:), eps);
        erro = g*yc0(:) - yrv(:);
        e = sqrt(mean(erro.^2, 'omitnan'));
    end

    delta_otimo = fminbnd(@obj, -limite, limite);
end

function resumo_erro(nome, y, yref)
    erro = y(:) - yref(:);
    rmse = sqrt(mean(erro.^2, 'omitnan'));
    mae  = mean(abs(erro), 'omitnan');
    emax = max(abs(erro), [], 'omitnan');
    pico_ref = max(abs(yref(:)), [], 'omitnan');
    if pico_ref > 0
        erro_pico_pct = 100*rmse/pico_ref;
    else
        erro_pico_pct = NaN;
    end
    fprintf('  %-24s | RMSE = %.6g | MAE = %.6g | Max = %.6g | Erro pico = %.6f %%\n', ...
        nome, rmse, mae, emax, erro_pico_pct);
end

function escrever_resumo_erro(fid, nome, y, yref)
    erro = y(:) - yref(:);
    rmse = sqrt(mean(erro.^2, 'omitnan'));
    mae  = mean(abs(erro), 'omitnan');
    emax = max(abs(erro), [], 'omitnan');
    pico_ref = max(abs(yref(:)), [], 'omitnan');
    if pico_ref > 0
        erro_pico_pct = 100*rmse/pico_ref;
    else
        erro_pico_pct = NaN;
    end
    fprintf(fid, '  %-24s | RMSE = %.6g | MAE = %.6g | Max = %.6g | Erro pico = %.6f %%\n', ...
        nome, rmse, mae, emax, erro_pico_pct);
end

function met = metricas_erro_percentual(y, yref)
    y = y(:); yref = yref(:);
    valido = isfinite(y) & isfinite(yref);
    y = y(valido); yref = yref(valido);
    erro = y - yref;
    met.MAE  = mean(abs(erro), 'omitnan');
    met.RMSE = sqrt(mean(erro.^2, 'omitnan'));
    met.MaxAbs = max(abs(erro), [], 'omitnan');
    pico_ref = max(abs(yref), [], 'omitnan');
    faixa_ref = max(yref, [], 'omitnan') - min(yref, [], 'omitnan');
    if pico_ref > 0
        met.ErroPico_pct = 100*met.RMSE/pico_ref;
    else
        met.ErroPico_pct = NaN;
    end
    if faixa_ref > 0
        met.NRMSE_pct = 100*met.RMSE/faixa_ref;
    else
        met.NRMSE_pct = NaN;
    end
end