%% Exportar Vin e Iin do Simulink para COMTRADE
% Vin.Data = [Va Vb Vc]
% Iin.Data = [Ia Ib Ic]

clc;
close all;

nome_base = 'curto_simulink';

%% Leitura dos sinais

t = Vin.Time;
N = length(t);

Va_data = Vin.Data(:,1);
Vb_data = Vin.Data(:,2);
Vc_data = Vin.Data(:,3);

Ia_data = Iin.Data(:,1);
Ib_data = Iin.Data(:,2);
Ic_data = Iin.Data(:,3);

%% Garante vetores coluna

t = t(:);

Va_data = Va_data(:);
Vb_data = Vb_data(:);
Vc_data = Vc_data(:);

Ia_data = Ia_data(:);
Ib_data = Ib_data(:);
Ic_data = Ic_data(:);

%% Frequência de amostragem

fs = 1/mean(diff(t));

disp(['Frequência de amostragem: ', num2str(fs), ' Hz']);
disp(['Número de amostras: ', num2str(N)]);

%% Pasta para salvar figuras

pasta_figuras = 'figuras_tcc';

if ~exist(pasta_figuras,'dir')
    mkdir(pasta_figuras);
end

%% Configuração das figuras

largura_cm = 28;
altura_cm  = 14;

fonte = 14;
espessura = 1.2;

%% Gráfico das tensões trifásicas

fig1 = figure('Color','w');

set(fig1, 'Units', 'centimeters');
set(fig1, 'Position', [2 2 largura_cm altura_cm]);

plot(t, Va_data, 'LineWidth', espessura); hold on;
plot(t, Vb_data, 'LineWidth', espessura);
plot(t, Vc_data, 'LineWidth', espessura);

grid on;
box on;

xlabel('Tempo (s)', 'FontSize', fonte);
ylabel('Tensão (V)', 'FontSize', fonte);

title('Amostras trifásicas de tensão geradas no Simulink', ...
    'FontSize', fonte + 2);

legend('V_a', 'V_b', 'V_c', 'Location', 'northeast');

set(gca, 'FontSize', fonte);
set(gca, 'LooseInset', max(get(gca,'TightInset'), 0.05));

set(fig1, 'PaperUnits', 'centimeters');
set(fig1, 'PaperSize', [largura_cm altura_cm]);
set(fig1, 'PaperPosition', [0 0 largura_cm altura_cm]);
set(fig1, 'PaperPositionMode', 'manual');

print(fig1, fullfile(pasta_figuras,'tensoes_trifasicas.pdf'), ...
    '-dpdf', '-painters');

print(fig1, fullfile(pasta_figuras,'tensoes_trifasicas.png'), ...
    '-dpng', '-r300');

%% Gráfico das correntes trifásicas

fig2 = figure('Color','w');

set(fig2, 'Units', 'centimeters');
set(fig2, 'Position', [2 2 largura_cm altura_cm]);

plot(t, Ia_data, 'LineWidth', espessura); hold on;
plot(t, Ib_data, 'LineWidth', espessura);
plot(t, Ic_data, 'LineWidth', espessura);

grid on;
box on;

xlabel('Tempo (s)', 'FontSize', fonte);
ylabel('Corrente (A)', 'FontSize', fonte);

title('Amostras trifásicas de corrente geradas no Simulink', ...
    'FontSize', fonte + 2);

legend('I_a', 'I_b', 'I_c', 'Location', 'northeast');

set(gca, 'FontSize', fonte);
set(gca, 'LooseInset', max(get(gca,'TightInset'), 0.05));

set(fig2, 'PaperUnits', 'centimeters');
set(fig2, 'PaperSize', [largura_cm altura_cm]);
set(fig2, 'PaperPosition', [0 0 largura_cm altura_cm]);
set(fig2, 'PaperPositionMode', 'manual');

print(fig2, fullfile(pasta_figuras,'correntes_trifasicas.pdf'), ...
    '-dpdf', '-painters');

print(fig2, fullfile(pasta_figuras,'correntes_trifasicas.png'), ...
    '-dpng', '-r300');

disp('Figuras exportadas com sucesso.');
disp(pasta_figuras);

%% Arquivo .CFG

fid = fopen([nome_base '.cfg'], 'w');

fprintf(fid, 'Simulink,Paulo,1999\n');
fprintf(fid, '6,6A,0D\n');

fprintf(fid, '1,Va,A,BarraEntrada,V,1,0,0,%f,%f,1,1,P\n', min(Va_data), max(Va_data));
fprintf(fid, '2,Vb,B,BarraEntrada,V,1,0,0,%f,%f,1,1,P\n', min(Vb_data), max(Vb_data));
fprintf(fid, '3,Vc,C,BarraEntrada,V,1,0,0,%f,%f,1,1,P\n', min(Vc_data), max(Vc_data));

fprintf(fid, '4,Ia,A,LinhaEntrada,A,1,0,0,%f,%f,1,1,P\n', min(Ia_data), max(Ia_data));
fprintf(fid, '5,Ib,B,LinhaEntrada,A,1,0,0,%f,%f,1,1,P\n', min(Ib_data), max(Ib_data));
fprintf(fid, '6,Ic,C,LinhaEntrada,A,1,0,0,%f,%f,1,1,P\n', min(Ic_data), max(Ic_data));

fprintf(fid, '60\n');
fprintf(fid, '1\n');

fprintf(fid, '%.0f,%d\n', fs, N);

fprintf(fid, '01/01/2026,00:00:00.000000\n');
fprintf(fid, '01/01/2026,00:00:00.000000\n');

fprintf(fid, 'ASCII\n');
fprintf(fid, '1\n');

fclose(fid);

%% Arquivo .DAT

fid = fopen([nome_base '.dat'], 'w');

for k = 1:N

    tempo_us = round(t(k)*1e6);

    fprintf(fid, '%d,%d,%f,%f,%f,%f,%f,%f\n', ...
        k, tempo_us, ...
        Va_data(k), Vb_data(k), Vc_data(k), ...
        Ia_data(k), Ib_data(k), Ic_data(k));

end

fclose(fid);

disp('Arquivos COMTRADE gerados com sucesso:');
disp([nome_base '.cfg']);
disp([nome_base '.dat']);
