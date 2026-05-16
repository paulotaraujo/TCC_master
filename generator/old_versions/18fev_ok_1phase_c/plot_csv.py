import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

# ============================================
# Configurações do Plot
# ============================================
# Arquivo de entrada
CSV_FILE = "output.csv"  # Ajuste o caminho se necessário

# Configurações dos gráficos
PLOT_CONFIG = {
    'figsize': (14, 10),
    'dpi': 100,
    'font_size': 10,
    'title_size': 12,
    'colors': {
        'dac_v': '#1f77b4',  # azul
        'dac_i': '#ff7f0e',  # laranja
        'adc_v': '#2ca02c',  # verde
        'adc_i': '#d62728',  # vermelho
        'clip': '#9467bd',   # roxo
        'transition': '#000000'  # preto para linha de transição
    }
}

# ============================================
# Funções de Plot
# ============================================
def load_csv_data(filename):
    """Carrega dados do CSV"""
    if not os.path.exists(filename):
        print(f"❌ Arquivo não encontrado: {filename}")
        return None
    
    try:
        # Tentar diferentes codificações
        encodings = ['utf-8', 'latin1', 'cp1252']
        df = None
        
        for encoding in encodings:
            try:
                df = pd.read_csv(filename, encoding=encoding)
                print(f"✅ Arquivo carregado com encoding: {encoding}")
                break
            except UnicodeDecodeError:
                continue
        
        if df is None:
            print("❌ Não foi possível ler o arquivo com as codificações testadas")
            return None
        
        print(f"\n📊 Estatísticas do arquivo:")
        print(f"   - Total de amostras: {len(df)}")
        print(f"   - Colunas: {list(df.columns)}")
        print(f"   - Tempo total: {df['t_us'].iloc[-1]/1e6:.3f} segundos")
        print(f"   - Taxa média: {len(df)/(df['t_us'].iloc[-1]/1e6):.1f} amostras/segundo")
        
        # Converter tempo para segundos para facilitar
        df['t_s'] = df['t_us'] / 1_000_000
        
        return df
        
    except Exception as e:
        print(f"❌ Erro ao carregar CSV: {e}")
        return None

def find_transition_point(df):
    """Encontra o ponto de transição pré-falta -> falta"""
    if 'mode' in df.columns:
        # Procurar a linha de transição
        transition_idx = df[df['mode'] == 'TRANSITION'].index
        if len(transition_idx) > 0:
            return transition_idx[0]
    
    # Se não encontrar, tentar pelo clip
    if 'clipV' in df.columns and 'clipI' in df.columns:
        clip_start = df[(df['clipV'] == 1) | (df['clipI'] == 1)].index
        if len(clip_start) > 0:
            return clip_start[0]
    
    return None

def plot_time_domain(df, save_plots=True):
    """Plot no domínio do tempo"""
    fig, axes = plt.subplots(3, 1, figsize=PLOT_CONFIG['figsize'], 
                             sharex=True, dpi=PLOT_CONFIG['dpi'])
    
    # Encontrar ponto de transição
    transition_idx = find_transition_point(df)
    transition_time = df['t_s'].iloc[transition_idx] if transition_idx else None
    
    # 1. Tensão: DAC vs ADC
    ax = axes[0]
    ax.plot(df['t_s'], df['dacV_code']/4095*3.3, 
            label='DAC Tensão (V)', color=PLOT_CONFIG['colors']['dac_v'], 
            linewidth=0.8, alpha=0.7)
    if 'adcV_V' in df.columns:
        ax.plot(df['t_s'], df['adcV_V'], 
                label='ADC Tensão (V)', color=PLOT_CONFIG['colors']['adc_v'], 
                linewidth=0.8, alpha=0.7, linestyle='--')
    ax.set_ylabel('Tensão (V)', fontsize=PLOT_CONFIG['font_size'])
    ax.set_title('Canal de Tensão - DAC vs ADC', fontsize=PLOT_CONFIG['title_size'])
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=PLOT_CONFIG['font_size'])
    
    # Linha de transição
    if transition_time:
        ax.axvline(x=transition_time, color=PLOT_CONFIG['colors']['transition'], 
                   linestyle='--', linewidth=1, label='Início da Falta')
    
    # 2. Corrente: DAC vs ADC
    ax = axes[1]
    ax.plot(df['t_s'], df['dacI_code']/4095*3.3, 
            label='DAC Corrente (V)', color=PLOT_CONFIG['colors']['dac_i'], 
            linewidth=0.8, alpha=0.7)
    if 'adcI_V' in df.columns:
        ax.plot(df['t_s'], df['adcI_V'], 
                label='ADC Corrente (V)', color=PLOT_CONFIG['colors']['adc_i'], 
                linewidth=0.8, alpha=0.7, linestyle='--')
    ax.set_ylabel('Tensão (V)', fontsize=PLOT_CONFIG['font_size'])
    ax.set_title('Canal de Corrente - DAC vs ADC', fontsize=PLOT_CONFIG['title_size'])
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=PLOT_CONFIG['font_size'])
    
    if transition_time:
        ax.axvline(x=transition_time, color=PLOT_CONFIG['colors']['transition'], 
                   linestyle='--', linewidth=1)
    
    # 3. Sinais de Clip e Modo
    ax = axes[2]
    if 'clipV' in df.columns:
        ax.plot(df['t_s'], df['clipV'], label='Clip Tensão', 
                color=PLOT_CONFIG['colors']['clip'], linewidth=1)
    if 'clipI' in df.columns:
        ax.plot(df['t_s'], df['clipI'], label='Clip Corrente', 
                color=PLOT_CONFIG['colors']['clip'], linewidth=1, alpha=0.5)
    if 'mode' in df.columns:
        # Converter modo para valores numéricos para plot
        mode_map = {'PRE': 0.5, 'TRANSITION': 1.0, 'FAULT': 1.5}
        mode_numeric = df['mode'].map(mode_map).fillna(0)
        ax.plot(df['t_s'], mode_numeric, label='Modo', 
                color='gray', linewidth=2, alpha=0.5)
    
    ax.set_xlabel('Tempo (s)', fontsize=PLOT_CONFIG['font_size'])
    ax.set_ylabel('Estado', fontsize=PLOT_CONFIG['font_size'])
    ax.set_title('Sinais de Controle', fontsize=PLOT_CONFIG['title_size'])
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=PLOT_CONFIG['font_size'])
    ax.set_ylim(-0.1, 2.0)
    
    if transition_time:
        ax.axvline(x=transition_time, color=PLOT_CONFIG['colors']['transition'], 
                   linestyle='--', linewidth=1)
    
    plt.tight_layout()
    
    if save_plots:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"plot_time_domain_{timestamp}.png"
        plt.savefig(filename, dpi=PLOT_CONFIG['dpi'], bbox_inches='tight')
        print(f"✅ Gráfico salvo: {filename}")
    
    plt.show()
    return fig

def plot_frequency_domain(df, save_plots=True):
    """Análise no domínio da frequência (FFT)"""
    # Separar pré-falta e falta
    transition_idx = find_transition_point(df)
    
    if transition_idx is None:
        print("⚠️ Não foi possível identificar transição para FFT")
        return
    
    pre_fault = df.iloc[:transition_idx]
    fault = df.iloc[transition_idx+1:] if transition_idx+1 < len(df) else None
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), dpi=PLOT_CONFIG['dpi'])
    
    # Função para calcular FFT
    def plot_fft(data, signal_name, ax, color, title_prefix):
        if len(data) < 10:
            ax.text(0.5, 0.5, 'Dados insuficientes', ha='center', va='center')
            return
        
        # Calcular FFT
        fs = 1 / (data['dt_us'].mean() / 1e6)  # Frequência de amostragem
        n = len(data)
        
        if signal_name == 'tensao':
            signal = data['dacV_code'].values
        else:
            signal = data['dacI_code'].values
        
        # FFT
        fft_vals = np.fft.fft(signal)
        fft_freq = np.fft.fftfreq(n, d=1/fs)
        
        # Pegar apenas frequências positivas
        pos_mask = fft_freq >= 0
        fft_freq = fft_freq[pos_mask]
        fft_vals = np.abs(fft_vals[pos_mask])
        
        # Normalizar
        fft_vals = fft_vals / n
        
        # Plotar (até 500 Hz para ver harmônicas)
        max_freq = min(500, fs/2)
        freq_mask = fft_freq <= max_freq
        
        ax.semilogy(fft_freq[freq_mask], fft_vals[freq_mask], 
                    color=color, linewidth=0.8)
        ax.set_xlabel('Frequência (Hz)', fontsize=PLOT_CONFIG['font_size'])
        ax.set_ylabel('Magnitude', fontsize=PLOT_CONFIG['font_size'])
        ax.set_title(f'{title_prefix} - {signal_name.capitalize()}', 
                    fontsize=PLOT_CONFIG['title_size'])
        ax.grid(True, alpha=0.3, which='both')
        ax.set_xlim([0, max_freq])
    
    # Plotar FFTs
    plot_fft(pre_fault, 'tensao', axes[0, 0], PLOT_CONFIG['colors']['dac_v'], 
             'Pré-falta')
    plot_fft(pre_fault, 'corrente', axes[0, 1], PLOT_CONFIG['colors']['dac_i'], 
             'Pré-falta')
    
    if fault is not None and len(fault) > 10:
        plot_fft(fault, 'tensao', axes[1, 0], PLOT_CONFIG['colors']['dac_v'], 
                 'Falta')
        plot_fft(fault, 'corrente', axes[1, 1], PLOT_CONFIG['colors']['dac_i'], 
                 'Falta')
    else:
        axes[1, 0].text(0.5, 0.5, 'Sem dados de falta', ha='center', va='center')
        axes[1, 1].text(0.5, 0.5, 'Sem dados de falta', ha='center', va='center')
    
    plt.tight_layout()
    
    if save_plots:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"plot_frequency_{timestamp}.png"
        plt.savefig(filename, dpi=PLOT_CONFIG['dpi'], bbox_inches='tight')
        print(f"✅ Gráfico salvo: {filename}")
    
    plt.show()
    return fig

def plot_phase_plane(df, save_plots=True):
    """Plano de fase (V x I)"""
    transition_idx = find_transition_point(df)
    
    if transition_idx is None:
        print("⚠️ Não foi possível identificar transição para plano de fase")
        return
    
    pre_fault = df.iloc[:transition_idx]
    fault = df.iloc[transition_idx+1:] if transition_idx+1 < len(df) else None
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=PLOT_CONFIG['dpi'])
    
    # Converter códigos DAC para tensão
    v_pre = pre_fault['dacV_code'] / 4095 * 3.3
    i_pre = pre_fault['dacI_code'] / 4095 * 3.3
    
    axes[0].scatter(v_pre, i_pre, c=pre_fault.index, cmap='viridis', 
                    s=1, alpha=0.5, label='Pré-falta')
    axes[0].set_xlabel('Tensão DAC (V)', fontsize=PLOT_CONFIG['font_size'])
    axes[0].set_ylabel('Corrente DAC (V)', fontsize=PLOT_CONFIG['font_size'])
    axes[0].set_title('Plano de Fase - Pré-falta', fontsize=PLOT_CONFIG['title_size'])
    axes[0].grid(True, alpha=0.3)
    axes[0].axis('equal')
    
    if fault is not None:
        v_fault = fault['dacV_code'] / 4095 * 3.3
        i_fault = fault['dacI_code'] / 4095 * 3.3
        
        scatter = axes[1].scatter(v_fault, i_fault, c=fault.index, 
                                   cmap='hot', s=1, alpha=0.7)
        axes[1].set_xlabel('Tensão DAC (V)', fontsize=PLOT_CONFIG['font_size'])
        axes[1].set_ylabel('Corrente DAC (V)', fontsize=PLOT_CONFIG['font_size'])
        axes[1].set_title('Plano de Fase - Falta', fontsize=PLOT_CONFIG['title_size'])
        axes[1].grid(True, alpha=0.3)
        axes[1].axis('equal')
        plt.colorbar(scatter, ax=axes[1], label='Amostra')
    
    plt.tight_layout()
    
    if save_plots:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"plot_phase_plane_{timestamp}.png"
        plt.savefig(filename, dpi=PLOT_CONFIG['dpi'], bbox_inches='tight')
        print(f"✅ Gráfico salvo: {filename}")
    
    plt.show()
    return fig

def plot_zoomed_regions(df, save_plots=True):
    """Zoom em regiões específicas"""
    transition_idx = find_transition_point(df)
    
    if transition_idx is None:
        return
    
    # Definir janelas de zoom
    pre_start = max(0, transition_idx - 200)
    pre_end = min(len(df), transition_idx + 200)
    zoom_df = df.iloc[pre_start:pre_end]
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), dpi=PLOT_CONFIG['dpi'])
    
    # Zoom na transição
    ax = axes[0]
    ax.plot(zoom_df['t_s'], zoom_df['dacV_code']/4095*3.3, 
            label='Tensão', color=PLOT_CONFIG['colors']['dac_v'], linewidth=1)
    ax.plot(zoom_df['t_s'], zoom_df['dacI_code']/4095*3.3, 
            label='Corrente', color=PLOT_CONFIG['colors']['dac_i'], linewidth=1)
    ax.axvline(x=df['t_s'].iloc[transition_idx], 
               color=PLOT_CONFIG['colors']['transition'], 
               linestyle='--', linewidth=2, label='Início da Falta')
    ax.set_xlabel('Tempo (s)', fontsize=PLOT_CONFIG['font_size'])
    ax.set_ylabel('Tensão (V)', fontsize=PLOT_CONFIG['font_size'])
    ax.set_title('Zoom na Transição Pré-falta → Falta', 
                 fontsize=PLOT_CONFIG['title_size'])
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    
    # Erro DAC vs ADC (se disponível)
    ax = axes[1]
    if 'adcV_V' in df.columns and 'adcI_V' in df.columns:
        v_error = zoom_df['dacV_code']/4095*3.3 - zoom_df['adcV_V']
        i_error = zoom_df['dacI_code']/4095*3.3 - zoom_df['adcI_V']
        
        ax.plot(zoom_df['t_s'], v_error, label='Erro Tensão', 
                color=PLOT_CONFIG['colors']['dac_v'], linewidth=1)
        ax.plot(zoom_df['t_s'], i_error, label='Erro Corrente', 
                color=PLOT_CONFIG['colors']['dac_i'], linewidth=1)
        ax.axvline(x=df['t_s'].iloc[transition_idx], 
                   color=PLOT_CONFIG['colors']['transition'], 
                   linestyle='--', linewidth=1)
        ax.set_xlabel('Tempo (s)', fontsize=PLOT_CONFIG['font_size'])
        ax.set_ylabel('Erro (V)', fontsize=PLOT_CONFIG['font_size'])
        ax.set_title('Erro DAC vs ADC na Transição', 
                     fontsize=PLOT_CONFIG['title_size'])
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    
    plt.tight_layout()
    
    if save_plots:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"plot_zoom_{timestamp}.png"
        plt.savefig(filename, dpi=PLOT_CONFIG['dpi'], bbox_inches='tight')
        print(f"✅ Gráfico salvo: {filename}")
    
    plt.show()
    return fig

def print_statistics(df):
    """Imprime estatísticas detalhadas"""
    print("\n" + "="*50)
    print("📈 ESTATÍSTICAS DETALHADAS")
    print("="*50)
    
    # Estatísticas básicas
    print(f"\n📊 Visão Geral:")
    print(f"   - Duração total: {df['t_s'].iloc[-1]:.3f} s")
    print(f"   - Total amostras: {len(df)}")
    print(f"   - Taxa média: {len(df)/df['t_s'].iloc[-1]:.1f} Hz")
    print(f"   - Taxa instantânea média: {1e6/df['dt_us'].mean():.1f} Hz")
    
    # Identificar modo
    if 'mode' in df.columns:
        pre_samples = len(df[df['mode'] == 'PRE'])
        fault_samples = len(df[df['mode'] == 'FAULT'])
        print(f"\n🎯 Modos de operação:")
        print(f"   - Pré-falta: {pre_samples} amostras ({pre_samples/len(df)*100:.1f}%)")
        print(f"   - Falta: {fault_samples} amostras ({fault_samples/len(df)*100:.1f}%)")
    
    # Estatísticas de clip
    if 'clipV' in df.columns:
        clip_v_samples = df['clipV'].sum()
        clip_i_samples = df['clipI'].sum()
        print(f"\n⚠️  Estatísticas de Clip:")
        print(f"   - Clip Tensão: {clip_v_samples} amostras ({clip_v_samples/len(df)*100:.2f}%)")
        print(f"   - Clip Corrente: {clip_i_samples} amostras ({clip_i_samples/len(df)*100:.2f}%)")
    
    # Estatísticas dos sinais
    print(f"\n📐 Amplitude dos Sinais DAC:")
    print(f"   - Tensão: min={df['dacV_code'].min()/4095*3.3:.3f}V, "
          f"max={df['dacV_code'].max()/4095*3.3:.3f}V, "
          f"média={df['dacV_code'].mean()/4095*3.3:.3f}V")
    print(f"   - Corrente: min={df['dacI_code'].min()/4095*3.3:.3f}V, "
          f"max={df['dacI_code'].max()/4095*3.3:.3f}V, "
          f"média={df['dacI_code'].mean()/4095*3.3:.3f}V")
    
    if 'adcV_V' in df.columns:
        v_error = df['dacV_code']/4095*3.3 - df['adcV_V']
        i_error = df['dacI_code']/4095*3.3 - df['adcI_V']
        print(f"\n🎯 Erro DAC vs ADC:")
        print(f"   - Tensão: RMS={np.sqrt(np.mean(v_error**2)):.4f}V, "
              f"max|erro|={np.max(np.abs(v_error)):.4f}V")
        print(f"   - Corrente: RMS={np.sqrt(np.mean(i_error**2)):.4f}V, "
              f"max|erro|={np.max(np.abs(i_error)):.4f}V")

# ============================================
# Main
# ============================================
def main():
    print("🎯 Plotter de Dados COMTRADE")
    print("="*50)
    
    # Carregar dados
    df = load_csv_data(CSV_FILE)
    if df is None:
        return
    
    # Imprimir estatísticas
    print_statistics(df)
    
    # Menu de plots
    while True:
        print("\n" + "="*50)
        print("📊 MENU DE PLOTS")
        print("="*50)
        print("1. Domínio do Tempo (completo)")
        print("2. Análise de Frequência (FFT)")
        print("3. Plano de Fase (V x I)")
        print("4. Zoom na Transição")
        print("5. Todos os plots")
        print("0. Sair")
        
        try:
            choice = input("\nEscolha uma opção: ").strip()
            
            if choice == '1':
                plot_time_domain(df)
            elif choice == '2':
                plot_frequency_domain(df)
            elif choice == '3':
                plot_phase_plane(df)
            elif choice == '4':
                plot_zoomed_regions(df)
            elif choice == '5':
                plot_time_domain(df)
                plot_frequency_domain(df)
                plot_phase_plane(df)
                plot_zoomed_regions(df)
            elif choice == '0':
                print("👋 Até mais!")
                break
            else:
                print("❌ Opção inválida!")
                
        except KeyboardInterrupt:
            print("\n\n👋 Interrompido pelo usuário")
            break
        except Exception as e:
            print(f"❌ Erro: {e}")

if __name__ == "__main__":
    main()