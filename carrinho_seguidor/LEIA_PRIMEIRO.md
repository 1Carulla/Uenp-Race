# V9.4 — faixa grossa, passos maiores, um único .ino

Base: V9.2 entregue anteriormente. Extraia a pasta `carrinho_seguidor` e abra `carrinho_seguidor.ino` no Arduino IDE. Este arquivo contém todo o código do projeto, inclusive HTML e JavaScript; não depende de headers locais, de arquivos JS soltos, do computador nem de cartão SD para servir o painel. Grave e recarregue carrinho.local ou o IP habitual. Confira **V9.4 · Prioridade para a faixa grossa**.

## Reconhecimento

Mantida a leitura próxima, entre 50% e 90% da imagem. Agora, na imagem de análise de 160 pixels de largura, o detector exige aproximadamente seis pixels de espessura perto do carrinho, reduzindo para quatro no trecho mais distante. Isso filtra riscos finos antes de escolher o trajeto.

Entre candidatos próximos, prioriza a faixa claramente mais larga; candidatos com menos de 65% da espessura da maior faixa são descartados. Depois de encontrar a faixa, continua exigindo proximidade entre pontos para não saltar para uma mancha distante. Duas faixas de espessuras semelhantes continuam sendo consideradas ambíguas quando representam rotas concorrentes.

Continuam necessários três pontos coerentes e três imagens para iniciar. Cruzamentos largos, ausência de linha próxima e contraste insuficiente seguem bloqueando movimento. A largura é aparente na imagem, não medida em centímetros: se a fita ficar muito distante ou estreita, poderá ser rejeitada. Sombras largas e riscos que se fundem à fita ainda podem causar ambiguidades; falta validar na pista.

## Movimento

| Ajuste | V9.2 | V9.4 |
|---|---:|---:|
| PWM do motor externo | 190/255 | 200/255 |
| Passo reto | 240 ms | 260 ms |
| Curva moderada | 190 ms | 220 ms |
| Curva forte/leitura parcial | 140 ms | 170 ms |
| Pausa mínima antes da nova imagem | 100 ms | 100 ms |

Aumento do PWM de aproximadamente 5,3%, para 78,4% da escala. Aplica-se também ao motor externo nas curvas. Em curva moderada, o interno continua recebendo uma fração do externo; em curva forte ele permanece parado para permitir a correção. PWM não mede potência elétrica, torque ou velocidade reais.

Para ajustar depois, altere no início do .ino:

```cpp
#define MOTOR_PWM 200
#define PASSO_RETA_MS 260
#define PASSO_CURVA_MS 220
#define PASSO_FECHADA_MS 170
#define PAUSA_OBSERVACAO_MS 100
```

Salve, grave novamente e recarregue o painel da ESP32. Tempos de movimento permitidos: 60–260 ms; pausa: 60–500 ms; PWM: 0–255. A reta já está no limite de passo desta versão. O painel incorporado recebe os valores dos mesmos defines, sem edição duplicada.

## Limpeza do código

Removidos o cálculo antigo de ângulo do servo, o controlador proporcional/derivativo que o acompanhava, a geração antiga de comandos de recuperação e seus estados sem uso. Esses métodos não participavam do modo de passos da V9.2. Buffers de análise de imagem agora são reutilizados.

Mantidos o bloqueio explícito de comandos de servo antigos, diagnóstico, proteção contra imagem atrasada, comunicação, cache, recuperação de câmera e servidores existentes. Esses trechos têm função atual. A contagem de linhas inclui todo o painel web e não mede o custo de processamento da ESP32; não foi feita compactação artificial para esconder linhas.

## Preservado

Wi-Fi e pinagem da V9.2, IO14/ENA esquerdo e IO15/ENB direito, servo desativado em IO13, câmera JPEG 320×240 em cinza e qualidade 18. As configurações de contraste e transporte são as da V9.2. Nenhuma nova função de servo foi adicionada.

Cada passo termina na própria ESP32. Comandos durante o movimento não prolongam nem enfileiram passos. O próximo passo exige uma imagem capturada após a pausa. Sem linha, fica parado procurando por até três segundos com imagens recentes; recupera com duas imagens válidas. Sem imagem/comunicação ou após três segundos, exige novo início.

Compile e faça a primeira tentativa com rodas suspensas. Compilação e testes de software aprovados; nenhum upload, movimento remoto ou teste físico realizado pelo agente nesta revisão.
