# Projeto Robô Seguidor de Linha

Este projeto implementa um robô seguidor de linha utilizando um microcontrolador AVR (ATmega328p), programado em C. O projeto foi desenvolvido para ser compilado com o WinAVR e simulado no Proteus.

## Observações IMPORTANTES

Os infravermelhos começam no Estado lógico alto (1) indicando que ele NÃO está sobre a linha preta ao clicar na etiqueta que está no pino de teste ele irá mudar seu status para 0 e mudará o sentido da rotação de uma roda.

O ultrasom é testado com um resistor que muda a sua resistencia com cliques, vc precisa aumentar ou diminuir lá durante a simulação para ver ele funcionando e parando a simulação. EM 1% é exatamente 15cm fazendo com que ele pare.

## Pré-requisitos

Para compilar e programar o microcontrolador, você precisará das seguintes ferramentas:

*   **WinAVR:** Um conjunto de ferramentas de desenvolvimento para microcontroladores Atmel AVR em Windows. Ele inclui o compilador `avr-gcc`, o `avr-objcopy` e outras utilidades. Você pode baixá-lo [aqui](https://winavr.sourceforge.net/).
*   **avrdude:** Um programa para programar (fazer o "flash") de microcontroladores AVR. O WinAVR já inclui o avrdude.
*   **Make:** Uma ferramenta para automatizar a compilação. O WinAVR já inclui o `make`.
*   **Proteus Design Suite:** Para simulação do circuito.

* **OBS** Seu computador deve ter esses programas no PATH do windows, ou então
vc irá precisar modificar o makefile para chamar eles a partir do caminho exato
que está instalado no seu computador

## Estrutura do Projeto

```
.
├── build/                # Arquivos de saída da compilação (binários, .hex)
├── src/                  # Código-fonte em C (.c, .h)
├── vendor/               # Bibliotecas de componentes de terceiros para o Proteus
├── line-follower.pdsprj  # Arquivo do projeto do Proteus
├── makefile              # Arquivo de configuração da compilação
└── README.md             # Este arquivo
```

## Como Compilar o Projeto

O `makefile` fornecido automatiza o processo de compilação. Para compilar o projeto, siga os passos abaixo:

1.  **Abra o Prompt de Comando (CMD) ou PowerShell** na pasta raiz do projeto.
2.  **Execute o comando `make`:**

    ```bash
    make
    ```

    Este comando irá:
    *   Criar o diretório `build/` se ele não existir.
    *   Compilar todos os arquivos `.c` da pasta `src/` para arquivos objeto `.o` dentro de `build/`.
    *   Lincar os arquivos objeto para criar um arquivo ELF (`.elf`) em `build/`.
    *   Converter o arquivo ELF para o formato HEX (`.hex`), que é o que será gravado no microcontrolador. O arquivo final será `build/projeto.hex`.

3.  **Para limpar os arquivos compilados**, você pode executar:
    ```bash
    make clean
    ```
    Isso removerá o diretório `build/` e todos os seus conteúdos.

## Como Programar o Microcontrolador (Flash)

Depois de compilar o projeto e gerar o arquivo `.hex`, você pode gravá-lo no ATmega328p usando o `avrdude`.

1.  **Conecte seu programador** (ex: USBasp, Arduino as ISP) ao seu PC e ao microcontrolador.
2.  **Identifique a porta serial** à qual seu programador está conectado (ex: `COM3`, `COM4`).
3.  **Atualize o `makefile`:**
    *   Abra o arquivo `makefile` em um editor de texto.
    *   Modifique a variável `AVRDUDE_PORT` para a porta serial correta. Por exemplo: `AVRDUDE_PORT = COM3`.
    *   Modifique a variável `AVRDUDE_PROGRAMMER` para o tipo do seu programador. O padrão está como `arduino`. Outros exemplos comuns são `usbasp`, `stk500v1`, etc.

4.  **Execute o comando `flash`:**
    ```bash
    make flash
    ```
    Este comando invocará o `avrdude` com as configurações do `makefile` para transferir o arquivo `build/projeto.hex` para a memória flash do microcontrolador.

## Como Usar o Arquivo .HEX no Proteus

Para simular o projeto, você precisa carregar o arquivo `.hex` compilado no componente do microcontrolador ATmega328p dentro do seu esquemático no Proteus.

1.  **Abra o projeto** `line-follower-em-c.pdsprj` no Proteus.
2.  **Dê um duplo-clique** no componente do microcontrolador ATmega328p para abrir a janela de propriedades (`Edit Component`).
3.  **Encontre o campo "Program File"**.
4.  **Clique no ícone da pasta** ao lado do campo para navegar até o arquivo `.hex`.
5.  **Selecione o arquivo** `projeto.hex` localizado na pasta `build/` do seu projeto.
6.  **Clique em "OK"** para salvar as alterações.

Agora, ao iniciar a simulação no Proteus, o ATmega328p executará o código que você compilou.

### Importando Componentes de Terceiros (Bibliotecas)

Este projeto pode utilizar bibliotecas de componentes customizados (como sensores infravermelhos ou o driver L298N) que não são padrão no Proteus. A pasta `vendor/proteus/` contém esses arquivos.

Para que o Proteus encontre e utilize essas bibliotecas, você precisa copiar os arquivos da pasta `vendor/proteus/` para a pasta de bibliotecas do Proteus.

O caminho de destino geralmente é:
`C:\Program Files (x86)\Labcenter Electronics\Proteus 8 Professional\DATA\LIBRARY`

Copie os arquivos `.LIB`, `.HEX` e `.IDX` das subpastas em `vendor/proteus/` para o diretório `LIBRARY` do Proteus. Após copiar, reinicie o Proteus para que ele carregue os novos componentes.

Em seguida olhe componente por componente (clicando em edit propriedades) para ver se o .HEX está configurado nele, se não estiver vc clica no componente, clica em edit properties, procura a propriedade do hex e cola o hex referente a esse sensor que está na pasta vendor:

## Ultrasom
./vendor/proteus/ultrasonic/UltraSonicTEP.HEX
## Infravermelhos
./vendor/proteus/infrared/InfraredSensorsTEP.HEX
