%% gerador_amostras.m
clc;

%% ==========================
% Selecione as fases
%
% [1]       -> A
% [2]       -> B
% [3]       -> C
% [1 2]     -> A B
% [1 3]     -> A C
% [2 3]     -> B C
% [1 2 3]   -> A B C
%% ==========================

fases = [1];

%% Verifica se Vin e Iin existem

if ~exist('Vin','var')
    error('A variável Vin não existe no Workspace. Rode a simulação primeiro.');
end

if ~exist('Iin','var')
    error('A variável Iin não existe no Workspace. Rode a simulação primeiro.');
end

%% Verifica quantos canais existem

nFases_V = size(Vin.Data,2);
nFases_I = size(Iin.Data,2);

fprintf('Vin possui %d canal(is).\n', nFases_V);
fprintf('Iin possui %d canal(is).\n', nFases_I);

%% Verifica seleção

if any(~ismember(fases,[1 2 3]))
    error('As fases devem ser 1, 2 e/ou 3.');
end

if any(fases > nFases_V)
    error('Vin possui apenas %d canal(is). Você pediu as fases %s.', ...
        nFases_V, mat2str(fases));
end

if any(fases > nFases_I)
    error('Iin possui apenas %d canal(is). Você pediu as fases %s.', ...
        nFases_I, mat2str(fases));
end

%% Cria cópias para não modificar Vin e Iin originais no Workspace

Vin_saida = Vin;
Iin_saida = Iin;

Vin_saida.Data = Vin.Data(:,fases);
Iin_saida.Data = Iin.Data(:,fases);

fases_salvas = fases;

%% Pasta de saída

pasta_saida = fullfile(getenv('HOME'), ...
    'Arduino','dev','1_phase','Resultados');

if ~exist(pasta_saida,'dir')
    mkdir(pasta_saida);
end

%% Arquivo

arquivo_saida = fullfile(pasta_saida,'faseA_oscilografia_25_inverso.mat');

%% Salva

save(arquivo_saida,'Vin_saida','Iin_saida','fases_salvas');

%% Informações

fprintf('\n=====================================\n');
fprintf('Arquivo salvo com sucesso!\n');
fprintf('Fases salvas: %s\n', mat2str(fases_salvas));
fprintf('Arquivo: %s\n', arquivo_saida);
fprintf('=====================================\n');