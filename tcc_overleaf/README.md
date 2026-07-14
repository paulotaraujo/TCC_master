# Projeto Overleaf do TCC

Este diretório contém um esqueleto em LaTeX para o TCC sobre a plataforma didática com ESP32, COMTRADE e relés de proteção.

## Como usar no Overleaf

1. Compacte a pasta `tcc_overleaf` em um arquivo `.zip`.
2. No Overleaf, clique em `New Project`.
3. Selecione `Upload Project`.
4. Envie o arquivo `.zip`.
5. Abra o arquivo `main.tex`.
6. No menu do Overleaf, use `pdfLaTeX` como compilador.

## O que editar primeiro

- Em `main.tex`, troque nome do autor, orientador, instituição, título e ano.
- Em `capitulos/01_introducao.tex`, revise introdução, justificativa e objetivos.
- Em `capitulos/03_metodologia.tex`, ajuste o fluxo real do projeto.
- Em `capitulos/04_implementacao.tex`, detalhe o protocolo serial, buffers, DAC e aquisição.
- Em `referencias.bib`, substitua ou complemente as referências conforme sua bibliografia.

## Figuras

Coloque imagens na pasta `figuras` e insira no texto usando:

```tex
\begin{figure}[H]
  \centering
  \includegraphics[width=0.85\textwidth]{figuras/nome_da_figura.png}
  \caption{Legenda da figura}
  \label{fig:nome-da-figura}
\end{figure}
```
