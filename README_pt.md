# 🎛️ DMX Master STM32

## 📌 Visão Geral

O **DMX-Master-STM32** faz parte de um sistema integrado que inclui uma **interface gráfica para Windows**, desenvolvida em **PyQt**. Essa interface permite enviar comandos DMX, interagir com dispositivos RDM e visualizar frames em tempo real. A comunicação entre o software e a placa ocorre via **USB-C**, possibilitando o controle e monitoramento dos dispositivos conectados.

🔗 **Link para o projeto da interface gráfica:** [GUI_RDM_DMX_Master](https://github.com/GuilhermeRS11/GUI_RDM_DMX_Master/tree/master)

Além dessa integração com o software, a placa também possui sua **própria interface gráfica embarcada**, exibida em um **display OLED de 0,96''** e controlada através de **quatro botões físicos**.

💡 **Status do Projeto:**  
✅ **Hardware pronto** (PCB projetada no **Altium Designer 25.1.2**).  
⚙️ **Firmware e interface embarcada em desenvolvimento** (**STM32Cube Version: 1.14.1**).  

---

## 🖥️ Interface Gráfica no Microcontrolador  

A interface gráfica na própria placa permite que o usuário configure e controle dispositivos **DMX e RDM** diretamente, sem necessidade do software externo.  

🛠 **Funcionalidades da interface embarcada:**  
- 🎚️ Ajuste de parâmetros **DMX e RDM**.  
- 📍 Configuração de **endereçamento** dos dispositivos.  
- 📡 Monitoramento das informações recebidas.  
- 🔘 Controle direto via botões físicos.  

Dessa forma, se o software gráfico Windows não estiver disponível, a placa pode ser alimentada por **5V** e utilizada de maneira independente.

---

## ⚙️ Sobre o Projeto DMX-Master-STM32

O **DMX-Master-STM32** é responsável por receber dados via **USB-C**, interpretá-los e convertê-los para o padrão DMX, permitindo a comunicação com dispositivos de iluminação e controle. Ele pode operar tanto conectado à GUI para PC quanto de forma autônoma.

### 🛠️ Recursos da PCB:
✅ **Conectividade USB-C** para comunicação com o software externo.  
✅ **Regulador de tensão 5V → 3.3V** para alimentação segura do circuito.  
✅ **Conversor USB CH340G** para interface serial.  
✅ **Microcontrolador STM32C031K6T6** para processamento e controle.  
✅ **Conversor MAX485** para interface RS485, compatível com DMX.  
✅ **4 botões físicos** para interação direta com a placa.  
✅ **Display OLED 128x64 (0.96'')** com menus e ajustes manuais.  

---

## 📡 Visualização do Esquemático  

Aqui estão algumas imagens do esquemático do circuito do projeto:  

![📜 Esquemático - Página 1](documents/images/schematic_1.png)  
![📜 Esquemático - Página 2](documents/images/schematic_2.png)  

---

## 🔧 Visualização do Hardware  

Aqui estão algumas imagens do modelo 3D da PCB:  

![🖼️ DMX-Master-STM32 - Vista Frontal](documents/images/3D_PCB_front_view.png)  
![🖼️ DMX-Master-STM32 - Vista Traseira](documents/images/3D_PCB_back_view.png)  

---

## 📜 Licença

Este projeto está licenciado sob a **[GPLv3](https://www.gnu.org/licenses/gpl-3.0.txt)**, permitindo o uso, modificação e distribuição do código, desde que as mesmas liberdades sejam garantidas.
 

---