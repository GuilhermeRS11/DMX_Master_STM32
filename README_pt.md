# DMX Master STM32

## Visão Geral

O **DMX-Master-STM32** faz parte de um sistema integrado que inclui uma **interface gráfica para Windows**, desenvolvida em **PyQt**. Essa interface permite enviar comandos DMX, interagir com dispositivos RDM e visualizar frames em tempo real. A comunicação entre o software e a placa ocorre via **USB-C**, possibilitando o controle e monitoramento dos dispositivos conectados.

🔗 Link para o projeto da interface gráfica: [GUI_RDM_DMX_Master](https://github.com/GuilhermeRS11/GUI_RDM_DMX_Master/tree/master)

Além dessa integração com o software, a placa também possui sua **própria interface gráfica embutida**, exibida em um **display OLED de 0,96''** e controlada através de **quatro botões físicos**.

**Atualmente, o desenvolvimento está focado na parte de hardware, com a PCB já projetada e pronta. O firmware e a interface gráfica embarcada ainda estão em desenvolvimento.**

A PCB foi projetada utilizando o **Altium Designer 25.1.2**, enquanto o firmware está sendo desenvolvido na **STM32Cube Version: 1.14.1**.

## Interface Gráfica no Microcontrolador

A interface gráfica na própria placa permite que o usuário configure e controle dispositivos **DMX e RDM** diretamente, sem necessidade do software externo. Os menus interativos possibilitam:
- Ajuste de parâmetros DMX e RDM.
- Configuração de endereçamento dos dispositivos.
- Monitoramento das informações recebidas.
- Controle direto via botões físicos.

Dessa forma, se o software gráfico não estiver disponível, a placa pode ser alimentada por **5V** e utilizada de maneira independente.

## Sobre o Projeto DMX-Master-STM32

O **DMX-Master-STM32** é responsável por receber dados via **USB-C**, interpretá-los e convertê-los para o padrão DMX, permitindo a comunicação com dispositivos de iluminação e controle. Ele pode operar tanto conectado à GUI para PC quanto de forma autônoma.

### Recursos da PCB:
- **Conectividade USB-C** para comunicação com o software externo.
- **Regulador de tensão 5V -> 3.3V** para alimentação segura do circuito.
- **Conversor USB CH340G** para interface serial.
- **Microcontrolador STM32C031K6T6** para processamento e controle.
- **Conversor MAX485** para interface RS485, compatível com DMX.
- **4 botões físicos** para interação direta com a placa.
- **Display OLED 128x64 de 0.96''** com menus e ajustes manuais.

## Visualização do Esquemático

Aqui estão algumas imagens do esquemático do circuito do projeto:

![Schematic Circuit page 1](documents/images/schematic_1.png)
![Schematic Circuit page 2](documents/images/schematic_2.png)

## Visualização do Hardware

Aqui estão algumas imagens do modelo 3D da PCB:

![DMX-Master-STM32 - Front View](documents/images/3D_PCB_front_view.png)
![DMX-Master-STM32 - Back View](documents/images/3D_PCB_back_view.png)


---

Este repositório conterá o firmware do microcontrolador, os arquivos da PCB e demais documentos técnicos necessários para o desenvolvimento e uso do sistema.
