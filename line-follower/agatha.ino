 // --- DEFINIÇÃO DOS PINOS ---
// Você pode alterar estes pinos conforme a sua montagem.

// Sensores Infravermelho (IR) - Seguidores de Linha
// Assumindo que o sensor retorna LOW (0) na linha preta e HIGH (1) na superfície branca.
const int SENSOR_ESQUERDA_PIN = A0; // Sensor IR da esquerda
const int SENSOR_DIREITA_PIN = A1;  // Sensor IR da direita

// Pinos para o Motor Esquerdo (Ponte H, ex: L298N)
const int MOTOR_ESQ_FRENTE_PIN = 4;
const int MOTOR_ESQ_TRAS_PIN = 5;

// Pinos para o Motor Direito (Ponte H, ex: L298N)
const int MOTOR_DIR_FRENTE_PIN = 6;
const int MOTOR_DIR_TRAS_PIN = 7;

// Pinos para o Sensor Ultrassônico (HC-SR04)
const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;

// Pino do LED de Debug
const int LED_DEBUG_PIN = 13; // LED embutido na maioria das placas Arduino

// --- VARIÁVEIS GLOBAIS PARA O LED DE DEBUG ---
unsigned long tempoAnteriorLed = 0;
const long intervaloLed = 250; // Intervalo para piscar o LED (em milissegundos)
int estadoLed = LOW;
int velocidade = 200;

// --- FUNÇÃO DE CONFIGURAÇÃO (SETUP) ---
// Roda uma única vez quando o Arduino é ligado ou resetado
void setup() {
  // Configuração dos pinos dos sensores de linha como entrada
  pinMode(SENSOR_ESQUERDA_PIN, INPUT);
  pinMode(SENSOR_DIREITA_PIN, INPUT);

  // Configuração dos pinos dos motores como saída
  pinMode(MOTOR_ESQ_FRENTE_PIN, OUTPUT);
  pinMode(MOTOR_ESQ_TRAS_PIN, OUTPUT);
  pinMode(MOTOR_DIR_FRENTE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_TRAS_PIN, OUTPUT);

  // Configuração dos pinos do sensor ultrassônico
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configuração do pino do LED de debug como saída
  pinMode(LED_DEBUG_PIN, OUTPUT);

  // Garante que os motores comecem parados
  motorParar();
Serial.begin(9600);
}

// --- FUNÇÃO PRINCIPAL (LOOP) ---
// Roda continuamente após o setup()
void loop() {
  // Atualiza o estado do LED de debug sem bloquear o código
  piscarLedDebug();

  // Mede a distância do obstáculo mais próximo
  int distancia = calcularDistancia();
  Serial.println("Distancia: " + distancia);
  // Lógica de decisão principal
  if (distancia > 0 && distancia <= 15) { // Aumentei a distância para 15cm para mais segurança
    motorParar();
  } else {
    // Se não há obstáculo, executa a lógica de seguir linha
    int leituraEsquerda = digitalRead(SENSOR_ESQUERDA_PIN);
    int leituraDireita = digitalRead(SENSOR_DIREITA_PIN);

    // Caso 1: Ambos os sensores fora da linha preta -> Seguir em frente
    if (leituraEsquerda == HIGH && leituraDireita == HIGH) {
      motorFrente();
      Serial.println("Esquerda HIGH Direita HIGH");
    }
    // Caso 2: Sensor da esquerda na linha preta -> Curva para a esquerda
    else if (leituraEsquerda == LOW && leituraDireita == HIGH) {
      motorCurvaEsq();
      Serial.println("Esquerda LOW Direita HIGH");
    }
    // Caso 3: Sensor da direita na linha preta -> Curva para a direita
    else if (leituraEsquerda == HIGH && leituraDireita == LOW) {
      motorCurvaDir();
      Serial.println("Esquerda HIGH Direita LOW");
    }
    // Caso 4: Ambos na linha preta (ex: cruzamento ou fim de linha) -> Parar
    else { // (leituraEsquerda == LOW && leituraDireita == LOW)
      motorParar();
      Serial.println("Esq LOW Dir LOW");
    }
  }

  delay(10); // Pequeno atraso para estabilidade
}

// --- FUNÇÕES DE CONTROLE DOS MOTORES ---

void motorFrente() {
  digitalWrite(MOTOR_ESQ_FRENTE_PIN, HIGH);
  digitalWrite(MOTOR_ESQ_TRAS_PIN, LOW);
  digitalWrite(MOTOR_DIR_FRENTE_PIN, HIGH);
  digitalWrite(MOTOR_DIR_TRAS_PIN, LOW);
}

void motorTras() {
  digitalWrite(MOTOR_ESQ_FRENTE_PIN, LOW);
  digitalWrite(MOTOR_ESQ_TRAS_PIN, HIGH);
  digitalWrite(MOTOR_DIR_FRENTE_PIN, LOW);
  digitalWrite(MOTOR_DIR_TRAS_PIN, HIGH);
}

void motorParar() {
  digitalWrite(MOTOR_ESQ_FRENTE_PIN, LOW);
  digitalWrite(MOTOR_ESQ_TRAS_PIN, LOW);
  digitalWrite(MOTOR_DIR_FRENTE_PIN, LOW);
  digitalWrite(MOTOR_DIR_TRAS_PIN, LOW);
}

void motorCurvaDir() {
  // Curva SUAVE para a direita:
  // Motor esquerdo (de fora) vai para frente
  digitalWrite(MOTOR_ESQ_FRENTE_PIN, HIGH);
  digitalWrite(MOTOR_ESQ_TRAS_PIN, LOW);

  // Motor direito (de dentro) PARA COMPLETAMENTE
  digitalWrite(MOTOR_DIR_FRENTE_PIN, LOW);
  digitalWrite(MOTOR_DIR_TRAS_PIN, LOW);
}

void motorCurvaEsq() {
  // Curva SUAVE para a esquerda:
  // Motor direito (de fora) vai para frente
  digitalWrite(MOTOR_DIR_FRENTE_PIN, HIGH);
  digitalWrite(MOTOR_DIR_TRAS_PIN, LOW);

  // Motor esquerdo (de dentro) PARA COMPLETAMENTE
  digitalWrite(MOTOR_ESQ_FRENTE_PIN, LOW);
  digitalWrite(MOTOR_ESQ_TRAS_PIN, LOW);
}
// --- FUNÇÕES DOS SENSORES E DEBUG ---

int calcularDistancia() {
  // Limpa o pino de trigger para garantir um pulso limpo
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // Envia o pulso ultrassônico de 10 microssegundos
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Lê o tempo que o pino de echo ficou em HIGH (em microssegundos)
  long duracao = pulseIn(ECHO_PIN, HIGH);

  // Calcula a distância em centímetros
  // Fórmula: distância = (duração * velocidade_do_som) / 2
  // Velocidade do som ≈ 0.034 cm/µs
  int distancia = duracao * 0.034 / 2;

  return distancia;
}

void piscarLedDebug() {
  unsigned long tempoAtual = millis();
  if (tempoAtual - tempoAnteriorLed >= intervaloLed) {
    tempoAnteriorLed = tempoAtual; // Salva o tempo atual

    // Inverte o estado do LED
    if (estadoLed == LOW) {
      estadoLed = HIGH;
    } else {
      estadoLed = LOW;
    }

    digitalWrite(LED_DEBUG_PIN, estadoLed);
  }
}
