%% gerador_amostras_fourier.m
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

%% Verifica se as variáveis existem no Workspace

if ~exist('VinFasor','var')
    error('A variável VinFasor não existe no Workspace. Rode a simulação primeiro.');
end

if ~exist('IinFasor','var')
    error('A variável IinFasor não existe no Workspace. Rode a simulação primeiro.');
end

%% Verifica seleção das fases

if any(~ismember(fases,[1 2 3]))
    error('As fases devem ser 1, 2 e/ou 3.');
end

%% Verifica quantidade de canais disponíveis

nFases_V = size(VinFasor.Data,2);
nFases_I = size(IinFasor.Data,2);

fprintf('VinFasor possui %d canal(is).\n', nFases_V);
fprintf('IinFasor possui %d canal(is).\n', nFases_I);

if any(fases > nFases_V)
    error('VinFasor possui apenas %d canal(is). Você pediu as fases %s.', ...
        nFases_V, mat2str(fases));
end

if any(fases > nFases_I)
    error('IinFasor possui apenas %d canal(is). Você pediu as fases %s.', ...
        nFases_I, mat2str(fases));
end

%% Cria variáveis de saída sem alterar as originais

Vin = VinFasor;
Iin = IinFasor;

Vin.Data = VinFasor.Data(:,fases);
Iin.Data = IinFasor.Data(:,fases);

fases_salvas = fases;

%% Pasta de saída

pasta_saida = fullfile(getenv('HOME'), ...
    'Arduino','dev','1_phase','Resultados');

if ~exist(pasta_saida,'dir')
    mkdir(pasta_saida);
end

%% Arquivo de saída

arquivo_saida = fullfile(pasta_saida,'faseA_fourier_sobretensao_15.mat');

%% Salva

save(arquivo_saida,'Vin','Iin','fases_salvas');

%% Informações

fprintf('\n=====================================\n');
fprintf('Arquivo Fourier salvo com sucesso!\n');
fprintf('Fases salvas: %s\n', mat2str(fases_salvas));
fprintf('Arquivo: %s\n', arquivo_saida);
fprintf('=====================================\n');