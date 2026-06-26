%% plotar_amostras.m
clc;
clear;
close all;

%% Arquivo .mat

arquivo = fullfile(getenv('HOME'), ...
    'Arduino','dev','1_phase','Resultados', ...
    ['faseA_fourier_sobretensao_15.mat']);

load(arquivo);

%% Compatibilidade com os nomes das variáveis

if exist('Vin_saida','var')
    Vin = Vin_saida;
end

if exist('Iin_saida','var')
    Iin = Iin_saida;
end

%% Verificação

if ~exist('Vin','var')
    error('A variável Vin não foi encontrada.');
end

if ~exist('Iin','var')
    error('A variável Iin não foi encontrada.');
end

%% Fases

if exist('fases_salvas','var')
    fases = fases_salvas;
else
    fases = 1:size(Vin.Data,2);
end

nomes = {'A','B','C'};

%% Dados

t = Vin.Time;
V = Vin.Data;
I = Iin.Data;

%% -------------------------
% Gráfico de tensão
%% -------------------------

figure('Name','Tensão','Color','w');
hold on;

for k = 1:size(V,2)
    plot(t,V(:,k),'LineWidth',1.5);
end

grid on;
box on;

xlabel('Tempo (s)');
ylabel('Tensão (V)');
title('Oscilografia das Tensões');

legend(nomes(fases),'Location','best');

set(gca,'FontSize',12);

%% -------------------------
% Gráfico de corrente
%% -------------------------

figure('Name','Corrente','Color','w');
hold on;

for k = 1:size(I,2)
    plot(t,I(:,k),'LineWidth',1.5);
end

grid on;
box on;

xlabel('Tempo (s)');
ylabel('Corrente (A)');
title('Oscilografia das Correntes');

legend(nomes(fases),'Location','best');

set(gca,'FontSize',12);

disp('Gráficos exibidos com sucesso.');