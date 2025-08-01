#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h> // Inclui isso para o acesso a alguns tipos padronizados
// Ex: As vezes eu quero usar só 8 bits que eu poderia tipar como um char
// que é um tipo que tem exatamente 8 bits, mas quem fosse ler o código
// poderia pensar que a variável seria pra armazenar um caractere ao invés
// de um número então usando uint8_t ao inves de unsigned char permite eu
// ser mais claro na minha intenção de código
// 1 byte -> [0-255] -> 8 bits
// uint8_t number8     = testValue; // 255 -> sei que aqui vai ser um n
// unsigned char numberChar    = testValue; // 255 -> aqui eu represento um número ou um char?

// -- CONFIGURAÇÕES DO MICROCONTROLADOR --

#define F_CPU 20000000UL // INDICA A FREQUÊNCIA DO CLOCK


// --- DEFINIÇÃO DOS PINOS ---
// AQUI está o PINOUT para visualização dos pinos do atmega328p:
// https://www.bitfoic.com/upload/20231213/25844c0c0e8684550735d6f25012063d.jpg

// AQUI está o datasheet do atmega328p
// https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

// Sensores Infravermelho (IR) - Seguidores de Linha
// Assumindo que o sensor retorna LOW (0) na linha preta e HIGH (1) na superfície branca.
// A0 é o PC0, A1 é o PC1 de acordo com o datasheet
#define SENSOR_ESQUERDA_PIN PC0 // Sensor IR da esquerda
#define SENSOR_DIREITA_PIN PC1  // Sensor IR da direita

// Pinos para o Motor Esquerdo (Ponte H, ex: L298N)
// D4 é PD4, D5 é PD5
#define MOTOR_ESQ_FRENTE_PIN PD4
#define MOTOR_ESQ_TRAS_PIN PD5

// Pinos para o Motor Direito (Ponte H, ex: L298N)
// D6 is PD6, D7 is PD7
#define MOTOR_DIR_FRENTE_PIN PD6
#define MOTOR_DIR_TRAS_PIN PD7

// Pinos para o Sensor Ultrassônico (HC-SR04)
// D9 is PB1, D10 is PB2
#define TRIGGER_PIN PB1
#define ECHO_PIN PB2
#define ULTRASONIC_PORT PORTB // O registrador para ESCREVER HIGH/LOW nos pinos do Port B
#define ULTRASONIC_DDR  DDRB  // O registrador para definir a direção (ENTRADA/SAÍDA) dos pinos do Port B
#define ULTRASONIC_PIN  PINB  // O registrador para LER o estado dos pinos do Port B

// Pino do LED de Debug
// D13 is PB5
#define LED_DEBUG_PIN PB5 // LED embutido na maioria das placas Arduino

// --- VARIÁVEIS GLOBAIS PARA CRIAR A FUNÇÃO MILIS COM TIMER 0 ---
static volatile uint32_t timer_overflow_count = 0;
static volatile uint8_t timer_fractional_ms = 0;

// --- VARIÁVEIS GLOBAIS PARA O LED DE DEBUG ---
const long intervaloLed = 250; // Intervalo para piscar o LED (em milissegundos)
uint8_t estadoLed = 0; // 0 Desligado ( LOW ), 1 Ligado ( HIGH )
int velocidade = 200;

// PROTÓTIPOS DAS FUNÇÕES
// A gente precisa dizer quais são as funções que nosso programa pode usar caso
// a gente vá defini-las após a função main e referência-las dentro da função main
// (besteiras do c)
void loop();
void setup();
void motorFrente();
void motorTras();
void motorParar();
void motorCurvaDir();
void motorCurvaEsq();
void piscarLedDebug();

// Configura o contador TIMER 0 pro milis
void millisInit();
// Simula o millis do arduino usando o contador TIMER 0
unsigned long millis();

// Referência para a funções de funcionamento do ultrasônico
// Talvez vcs possam dizer que usaram esse código de um terceiro
// mas não do ecossistema do arduino
// https://github.com/hiibrarahmad/csr04-ultrasonic-sensor-with-atmega328p-2021-proteus-simulation/tree/main
void HCSR04Init(); // Configura os pinos
void signalPulse(); // Da start no processo de medição da distância
uint16_t GetPulseWidth(); // Mede a duração do pulso
uint16_t calcularDistancia(); // Essa eu mudei só a definição dela pra que tem na referencia -> na referência é chamada de get_ultrasonic_distance_cm

// --- FUNÇÃO DE CONFIGURAÇÃO (SETUP) ---
// Roda uma única vez quando o Arduino é ligado ou resetado
void setup() {
    // Aqui pra gente entender a gente tem que entender as operações bit a bit
    // AND, OR, XOR, NOT, etc e tal
    // Configuração dos pinos dos sensores de linha como entrada
    DDRC &= ~((1 << SENSOR_ESQUERDA_PIN) | (1 << SENSOR_DIREITA_PIN));
    PORTC |= ((1 << SENSOR_ESQUERDA_PIN) | (1 << SENSOR_DIREITA_PIN)); // Enable pull-ups, assuming not needed if sensor provides external pull-up/down

    // Configuração dos pinos dos motores como saída
    DDRD |= ((1 << MOTOR_ESQ_FRENTE_PIN) | (1 << MOTOR_ESQ_TRAS_PIN) |
             (1 << MOTOR_DIR_FRENTE_PIN) | (1 << MOTOR_DIR_TRAS_PIN)); // Set as output

    // Configuração dos pinos do sensor ultrassônico
    DDRB |= (1 << TRIGGER_PIN); // TRIGGER_PIN as output
    DDRB &= ~(1 << ECHO_PIN);   // ECHO_PIN as input

    // Configuração do pino do LED de debug como saída
    DDRB |= (1 << LED_DEBUG_PIN); // Set as output

    // Garante que os motores comecem parados
    motorParar();
    millisInit();
}

int main(void){
  setup();
  while(1){
    loop();
  }
  return 0;
}


// --- FUNÇÃO PRINCIPAL (LOOP) ---
// Roda continuamente após o setup()
void loop() {
    // Atualiza o estado do LED de debug sem bloquear o código
    piscarLedDebug();

    // Mede a distância do obstáculo mais próximo
    int distancia = calcularDistancia();

    // Aqui n funciona pq ngm implementou USART nesse código então comentei
    // Serial.println("Distancia: " + distancia);

    // Lógica de decisão principal
    if (distancia > 0 && distancia <= 15) { // Aumentei a distância para 15cm para mais segurança
        motorParar();
    } else {

        // Se não há obstáculo, executa a lógica de seguir linha
        // Use PINx register to read input pin states
        // Temos que usar esse registrador PINx para ler os estados dos inputs
        int leituraEsquerda = (PINC & (1 << SENSOR_ESQUERDA_PIN)) ? 1 : 0;
        int leituraDireita = (PINC & (1 << SENSOR_DIREITA_PIN)) ? 1 : 0;

        // 0 -> LINHA PRETA
        // 1 -> NA PARTE BRANCA (AQUI TALVEZ SEJA BOM ATIVAR O PULL_UP PRA SEMPRE MANTER O ESTADO COMO 1)
        // Algumas interferências podem gerar problemas e deixar o robo meio doido, vale a pena testar
        // -------------------------------------------------------------------------------------------------
        // | Sensor Esquerda | Sensor Direita | Estado do Robô (Lógica) | Ação Chamada                     |
        // -------------------------------------------------------------------------------------------------
        // | HIGH (Fora)     | HIGH (Fora)    | Ambos fora da linha     | Ir para frente                   |
        // | LOW (Na Linha)  | HIGH (Fora)    | Esquerda na linha       | Curvar para a esquerda           |
        // | HIGH (Fora)     | LOW (Na Linha) | Direita na linha        | Curvar para a direita            |
        // | LOW (Na Linha)  | LOW (Na Linha) | Ambos na linha (Fim?)   | Parar                            |
        // -------------------------------------------------------------------------------------------------
        // 1 - Os dois sensores não detectam a linha preta -> Vai retor
        if (leituraEsquerda == 1 && leituraDireita == 1) {
            motorFrente();
        }
        // 2 - Sensores esquerdo detectam a linha preta -> Vira a esquerda
        else if (leituraEsquerda == 0 && leituraDireita == 1) {
            motorCurvaEsq();
        }
        // 3 - Sensores direito detectam a linha preta -> Vira a direita
        else if (leituraEsquerda == 1 && leituraDireita == 0) {
            motorCurvaDir();
        }
        // Case 4: Both on black line (e.g., intersection or end of line) -> Stop
        // 4 - Sensores detectam a linha preta -> Parar
        else { // (leituraEsquerda == 0 && leituraDireita == 0)
            motorParar();
        }
    }

    _delay_ms(10); // Pequeno atraso para estabilidade
}

// --- FUNÇÕES DE CONTROLE DOS MOTORES ---
void motorFrente() {
    PORTD |= (1 << MOTOR_ESQ_FRENTE_PIN) | (1 << MOTOR_DIR_FRENTE_PIN);
    PORTD &= ~((1 << MOTOR_ESQ_TRAS_PIN) | (1 << MOTOR_DIR_TRAS_PIN));
}

void motorTras() {
    PORTD |= (1 << MOTOR_ESQ_TRAS_PIN) | (1 << MOTOR_DIR_TRAS_PIN);
    PORTD &= ~((1 << MOTOR_ESQ_FRENTE_PIN) | (1 << MOTOR_DIR_FRENTE_PIN));
}

void motorParar() {
    PORTD &= ~((1 << MOTOR_ESQ_FRENTE_PIN) | (1 << MOTOR_ESQ_TRAS_PIN) |
               (1 << MOTOR_DIR_FRENTE_PIN) | (1 << MOTOR_DIR_TRAS_PIN));
}

void motorCurvaDir() {
    // Motor esquerdo para frente, motor direito para trás (curva no próprio eixo)
    PORTD |= (1 << MOTOR_ESQ_FRENTE_PIN) | (1 << MOTOR_DIR_TRAS_PIN);
    PORTD &= ~((1 << MOTOR_ESQ_TRAS_PIN) | (1 << MOTOR_DIR_FRENTE_PIN));
}

void motorCurvaEsq() {
    // Motor direito para frente, motor esquerdo para trás (curva no próprio eixo)
    PORTD |= (1 << MOTOR_ESQ_TRAS_PIN) | (1 << MOTOR_DIR_FRENTE_PIN);
    PORTD &= ~((1 << MOTOR_ESQ_FRENTE_PIN) | (1 << MOTOR_DIR_TRAS_PIN));
}

// --- FUNÇÕES DOS SENSORES E DEBUG ---

void piscarLedDebug() {
    static unsigned long tempoAnteriorLed = 0;
    unsigned long tempoAtual = millis();
    if (tempoAtual - tempoAnteriorLed >= intervaloLed) {
        tempoAnteriorLed = tempoAtual; // Salva o tempo atual

        // Inverte o estado do LED
        if (estadoLed == 0) { // Desligado
            estadoLed = 1;    // Liga
            PORTB |= (1 << LED_DEBUG_PIN);
        } else { // Ligado
            estadoLed = 0;   // Desliga
            PORTB &= ~(1 << LED_DEBUG_PIN);
        }
    }
}



/*
 * Cálculo do millis
 * F_CPU = 16.000.000 Hz
 * Timer0 é de 8 bits, então ele faz o overflow em 256 contagens (0-255).
 * Queremos uma frequência tal que o overflow seja o mais próximo possível de 1ms.
 * Para isso podemos usar a frequencia do crystal junto de um prescaler pra fazer
 * esses calculos, temos algumas opções de prescaler (algo que é usado pra fazer o clock ser dividido por determinado valor pois ele é mto mto rápido):

 * Opções de prescaler: 1, 8, 64, 256, 1024
 *
 * Vamos a um exemplo
 * 1. Prescaler = 64:
 * Frequência do clock do [Timer = F_CPU / 64] ->  [16.000.000 / 64 = 250.000 Hz]
 * o que to chamando de tick -> o clock roda 250.000 vezes por segundo! o tick é
 * justamente o tempo que leva para 1 execução completa desse clock.
 * Tempo por "tick" = 1 / 250.000 = 0,000004 segundos = 4 microssegundos. Ou seja
 * com essa frequencia do crystal, o clock leva 4 microssegundos para rodar
 * Bom, então se o clock leva 4 microssegundos em um tick e eu preciso de 256
 * pro contador voltar ao zero (overflow) qual é o tempo para esse overflow
 * acontecer?
 * Tempo  overflow = 256 "ticks" * 4 ms/tick = 1024 microssegundos = 1,024 ms
 * Isso significa que a cada overflow, passou 1 ms, mas temos
 * um erro acumulado de 0,024 ms por overflow pq achamos 1024.
 * então precisamos corrigir esse erro acumulado.
 * A gente corrige isso acumulando em uma variavel essa parte fracional em
 * 24 cada vez que acontece um overflow, quando esse 24 chegar a 1000, a gente
 * pode incrementar 1 a mais no timer_overflow_count e zerar essa parte fracional
 */

// O que o TIMER0 Executa
ISR(TIMER0_OVF_vect) {
    timer_overflow_count++;
    timer_fractional_ms += 24;

    if (timer_fractional_ms >= 1000) {
        timer_overflow_count++;
        timer_fractional_ms -= 1000;
    }
}

// Setando os bits de acordo com o datasheet
void millisInit(void) {
    // Configurando o modo do preescaler como mostra no datasheet
    // modo normal
    TCCR0A = 0;
    TCCR0B = 0;

    // Configurando o valor do prescaler pra 64
    // CS02 = 0, CS01 = 1, CS00 = 1
    TCCR0B |= (1 << CS01);
    TCCR0B |= (1 << CS00);

    // Aparentemente tem que ativar o timer com esses bits ai
    TIMSK0 |= (1 << TOIE0);

    // Habilita as interrupções
    sei();
}

// Função usada pra dizer quanto tempo se passou
uint32_t millis(void) {
    uint32_t ms;
    cli();
    ms = timer_overflow_count;
    sei();

    return ms;
}

// FUNÇÕES DO ULTRASÔNICO === aqui eu copiei e colei, mudando os pinos. Essa parte veio todo de uma lib externa que não é do arduino
// botei la no começo o link dela.
void HCSR04Init() {
    // Configura o pino de trigger como saída, pois ele irá gerar a onda sonora ultrassônica.
    // DDR (Data Direction Register): 1 para saída, 0 para entrada.
    ULTRASONIC_DDR |= (1 << TRIGGER_PIN);
    // O pino ECHO deve ser configurado como entrada, caso não esteja.
    ULTRASONIC_DDR &= ~(1 << ECHO_PIN);
}

uint16_t GetPulseWidth() {
    // Esta função será usada para medir a duração do pulso. Quando o eco ultrassônico retorna após atingir um objeto,
    // o microcontrolador lerá o pulso usando o pino de eco do sensor ultrassônico conectado a ele.

    uint32_t i, result;

    // Seção - 1: As linhas de código a seguir, antes da Seção - 2, verificam se o ultrassônico está funcionando.
    // Ele verifica o pino de eco por um certo tempo. Se não houver sinal, significa que o sensor não está funcionando ou não está conectado corretamente.
    for(i = 0; i < 600000; i++) {
        // PIN (Port Input Register): Verifica o estado do pino.
        if(!(ULTRASONIC_PIN & (1 << ECHO_PIN)))
            continue;    // A linha ainda está baixa, então espere (pulso não iniciou)
        else
            break;       // Borda de subida detectada, então saia do loop.
    }

    if(i == 600000)
        return -1;    // Indica timeout (tempo esgotado esperando a borda de subida)

    // Borda de subida encontrada

    // Seção -2 : Esta seção prepara o Timer1 para contar o tempo do pulso.
    // Temporizadores em microcontroladores são usados para operações de temporização.
    // Configurar Timer1:
    TCCR1A = 0x00;           // Modo Normal (ou outro modo, dependendo da necessidade, mas para contagem simples, Normal é comum)
    TCCR1B = (1 << CS11);    // Configura o prescaler para 8.
                             // F_CPU = 16MHz, prescaler 8 -> Timer clock = 2MHz -> 1 tick = 0.5us.
                             // Isso define a resolução do temporizador.
    TCNT1 = 0x00;            // Inicia o contador do Timer1 do zero.

    // Seção -3 : Esta seção verifica se há algum objeto ou não.
    for(i = 0; i < 600000; i++) { // O valor 600000 é usado aleatoriamente para denotar um tempo pequeno, quase 40 milissegundos.
        if(ULTRASONIC_PIN & (1 << ECHO_PIN)) { // Verifica se o pino ECHO ainda está HIGH
            // Se o valor de TCNT1 ficar maior que 60000, significa que não há objeto no alcance do sensor (pulso muito longo).
            if(TCNT1 > 60000) break;
            else continue;
        } else {
            break; // Borda de descida encontrada (pulso terminou).
        }
    }

    if(i == 600000)
        return -2; // Timeout ou nenhum objeto (tempo esgotado esperando a borda de descida)

    // Borda de descida encontrada

    result = TCNT1; // O microcontrolador armazena o valor do tempo de pulso contado no registrador TCNT1.
                    // Então, estamos retornando este valor para a função principal para utilizá-lo posteriormente.

    // Parar Timer: Desativa o clock do Timer1.
    TCCR1B = 0x00;

    if(result > 60000)
        return -2; // Nenhum obstáculo (pulso muito longo)
    else
        return (result >> 1); // Retorna a duração do pulso (dividida por 2).
                              // Esta divisão é uma otimização/ajuste que depende do cálculo final da distância.
                              // Se 1 tick = 0.5us, e a distância = us / 58.0, então:
                              // distância = (ticks * 0.5) / 58.0 = ticks / 116.0.
                              // `result >> 1` é `result / 2`. Isso implicaria que a distância final é (result / 2) / ~29.0
                              // É importante testar e calibrar este fator.
}

// Pulsa 10 microsegundos no pino trigger, isso inicia o processo de medição de distância.
void signalPulse() {
    // Seta o pino TRIGGER HIGH.
    ULTRASONIC_PORT |= (1 << TRIGGER_PIN);
    _delay_us(10); // Espera 10 microsegundos.
    // Seta o pino TRIGGER LOW.
    ULTRASONIC_PORT &= ~(1 << TRIGGER_PIN);
}

uint16_t calcularDistancia() {
    // Redefine o contador do Timer1.
    TCNT1 = 0;

    // Limpa quaisquer flags de interrupção de Captura de Entrada e Transbordo pendentes do Timer1.
    // TIFR1 (Timer Interrupt Flag Register 1): Registrador de flags de interrupção do Timer1.
    // ICF1 (Input Capture Flag 1): Flag de Captura de Entrada.
    // TOV1 (Timer Overflow Flag 1): Flag de Transbordo do Timer.
    // Limpar uma flag é feito escrevendo '1' no bit correspondente.
    TIFR1 |= (1 << ICF1); // Limpa flag de Captura de Entrada
    TIFR1 |= (1 << TOV1); // Limpa flag de Transbordo

    // Envia o pulso de 10us para o pino Trigger.
    signalPulse();

    // Reutilizando a lógica de GetPulseWidth para a medição do pulso (polling).
    // Esta abordagem usa loops de polling para esperar pelas bordas, em vez de interrupções de Input Capture.
    long duracao_ticks_dividida = GetPulseWidth();

    if (duracao_ticks_dividida < 0) { // Verifica se houve erro/timeout na leitura
        return 0xFFFF; // Retorna um valor de erro (e.g., o máximo para uint16_t)
    }

    // O valor retornado por GetPulseWidth() é `result >> 1`, ou seja, TCNT1 / 2.
    // Se TCNT1 é o número bruto de ticks, e cada tick é 0.5us (com prescaler 8 e F_CPU 16MHz):
    // Duração real em microssegundos (us) = `duracao_ticks_dividida` * 2 * 0.5 = `duracao_ticks_dividida`
    // (Pois (TCNT1 / 2) * 2 * 0.5 = TCNT1 * 0.5)
    // Então, a `duracao_ticks_dividida` de `GetPulseWidth()` *JÁ É* a duração em microssegundos.
    // (Isto é uma interpretação baseada no `>>1` no retorno de GetPulseWidth. Calibre se necessário!)

    // Fórmula da distância em centímetros (cm):
    // Distância = (duração em microssegundos) / 58.0
    // (O 58.0 vem de: Velocidade do som = 343 m/s = 0.0343 cm/µs.
    // Como o som vai e volta, dividimos por 2: 0.0343 / 2 = 0.01715 cm/µs.
    // Então, 1 / 0.01715 ≈ 58.3. Usamos 58.0 para simplificação ou calibração.)
    uint16_t distance_cm = (uint16_t)((double)duracao_ticks_dividida / 58.0);

    return distance_cm;
}
