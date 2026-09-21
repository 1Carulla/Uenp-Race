# Validação V9.4

- Base conferida contra o ZIP entregue da V9.2. Firmware final em um .ino, sem includes de arquivos locais. HTML, JavaScript e configuração manual incorporados.
- Detecção: vinte cenas sintéticas com faixa grossa fora do centro e riscos finos perto do centro reconhecidas; dezesseis eram rejeitadas na V9.2. Testados riscos finos isolados verticais/diagonais, curvas para ambos os lados, perda da faixa, recuperação, duas faixas semelhantes, cruzamento largo, campo vazio/escuro e menos de três pontos.
- Reconstrução da câmera da captura anterior continua válida, com sete pontos. Essa imagem aproximada tem as marcações coloridas interpoladas e não equivale ao JPEG original nem ao teste ao vivo.
- Direção diferencial: 1.001 pares de desvios espelhados, PWM máximo 200, passos de 260/220/170 ms, troca de lados e ausência de linha. Núcleo C++ aceita 200, rejeita 201 nesta configuração e rejeita duração acima de 260 ms.
- Passo de 260 ms termina autonomamente, mensagens recebidas durante o movimento não prolongam o prazo nem trocam o alvo no meio do passo. Zero, botão parar, perda de Wi-Fi, falta de comando, imagem repetida/antiga e volta do contador testados.
- Painel real: três pontos/três imagens para início, comando zero durante ausência de linha, três segundos de procura parada, recuperação com duas imagens, imagem atrasada bloqueada e protocolo antigo impedido.
- Sessões WebSocket, reconexão sem retomada, transporte de câmera e configuração manual do .ino aprovados. Valores alterados apenas nos defines chegam ao HTML compilado e aos limites do firmware.
- Câmera, rede, diagnóstico e pinos comparados com V9.2. Remoções restritas aos cálculos antigos de servo/recuperação que já não eram chamados.
- Compilação/link para ESP32-CAM / Arduino ESP32 3.3.11. Etapas de geração binária afetadas pelo runtime local omitidas, como nas versões anteriores; entrega em fonte para o Arduino IDE. Sem upload ou validação física.
