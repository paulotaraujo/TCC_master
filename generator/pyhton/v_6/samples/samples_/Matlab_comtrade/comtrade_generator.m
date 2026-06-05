%% Exportar Vin e Iin do Simulink para COMTRADE
% Canais esperados no Workspace:
% Vin e Iin como timeseries
% Vin.Data = [Va Vb Vc]
% Iin.Data = [Ia Ib Ic]

clc;

nome_base = 'curto_simulink';

%% Captura do tempo
t = Vin.Time;              % tempo em segundos
N = length(t);

%% Captura dos sinais trifásicos
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
fs = 1 / mean(diff(t));

disp(['Frequência de amostragem: ', num2str(fs), ' Hz']);
disp(['Número de amostras: ', num2str(N)]);

%% Arquivo .CFG
fid = fopen([nome_base '.cfg'], 'w');

fprintf(fid, 'Simulink,Paulo,1999\n');
fprintf(fid, '6,6A,0D\n');

% Canal, nome, fase, circuito, unidade, a, b, skew, min, max, primary, secondary, PS
fprintf(fid, '1,Va,A,BarraEntrada,V,1,0,0,%f,%f,1,1,P\n', min(Va_data), max(Va_data));
fprintf(fid, '2,Vb,B,BarraEntrada,V,1,0,0,%f,%f,1,1,P\n', min(Vb_data), max(Vb_data));
fprintf(fid, '3,Vc,C,BarraEntrada,V,1,0,0,%f,%f,1,1,P\n', min(Vc_data), max(Vc_data));

fprintf(fid, '4,Ia,A,LinhaEntrada,A,1,0,0,%f,%f,1,1,P\n', min(Ia_data), max(Ia_data));
fprintf(fid, '5,Ib,B,LinhaEntrada,A,1,0,0,%f,%f,1,1,P\n', min(Ib_data), max(Ib_data));
fprintf(fid, '6,Ic,C,LinhaEntrada,A,1,0,0,%f,%f,1,1,P\n', min(Ic_data), max(Ic_data));

fprintf(fid, '60\n');       % frequência nominal do sistema
fprintf(fid, '1\n');        % número de taxas de amostragem

fprintf(fid, '%.0f,%d\n', fs, N);

fprintf(fid, '01/01/2026,00:00:00.000000\n');
fprintf(fid, '01/01/2026,00:00:00.000000\n');

fprintf(fid, 'ASCII\n');
fprintf(fid, '1\n');

fclose(fid);

%% Arquivo .DAT
fid = fopen([nome_base '.dat'], 'w');

for k = 1:N
    tempo_us = t(k) * 1e6;   % COMTRADE usa tempo em microssegundos

    fprintf(fid, '%d,%d,%f,%f,%f,%f,%f,%f\n', ...
        k, round(tempo_us), ...
        Va_data(k), Vb_data(k), Vc_data(k), ...
        Ia_data(k), Ib_data(k), Ic_data(k));
end

fclose(fid);

disp('Arquivos COMTRADE gerados com sucesso:');
disp([nome_base '.cfg']);
disp([nome_base '.dat']);