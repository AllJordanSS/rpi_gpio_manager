# RPI_gpio_manager  
![ROS 2 Version](https://img.shields.io/badge/ROS%202-Humble-green)
![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)

---

Este pacote é responsável por gerenciar os GPIOs da Raspberry Pi 4 (modelo CM4) para a plataforma robótica **LYSA V2.1.X**, fornecendo um serviço ROS 2 para ativar ou desativar motores.

## 📌 Descrição

O pacote `rpi_gpio_manager` fornece:

- Um nó ROS 2 chamado `gpio_node`
- Um serviço chamado `/motor_power` (configurável)
- Controle dos pinos GPIO via arquivo de configuração: `params.yaml`
- Respostas customizáveis no YAML
- Limpeza automática dos GPIOs ao encerrar o nó

Esse projeto foi desenvolvido para facilitar o setup inicial da plataforma robótica LYSA, permitindo fácil adaptação para outros projetos com controle de GPIO via ROS 2.

---

## ⚙️ Funcionalidades principais

- Serviço ROS 2 baseado em `std_srvs/SetBool`
- Ativação/desativação de dois motores conectados a GPIOs
- Configurações carregadas dinamicamente via `params.yaml`
- Estado inicial dos pinos definido no YAML
- Mensagens de resposta configuráveis no YAML
- Compatível com ROS 2 Humble e arquitetura ARM64 (Raspberry Pi 4)

---

## 🧰 Requisitos

- ROS 2 Humble instalado
- Python 3.8 ou superior
- Acesso root ou permissão para usar GPIO
- Plataforma: Ubuntu Server 64-bit ou Raspberry Pi OS 64-bit

---

## 📥 Como baixar e instalar

### 1. Crie um workspace ROS 2 (se ainda não tiver):

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

### 2. Clone o repositório:

```bash
git clone https://gitlab.com/AllJordanSS/rpi_gpio_manager.git
```

### 3. Volte para a raiz do workspace e compile:

```bash
cd ..
colcon build --packages-select rpi_gpio_manager
source install/setup.bash
```

---

## ▶️ Como executar

Após compilar:

```bash
ros2 run rpi_gpio_manager gpio_node
```

### Chame o serviço para ligar ou desligar os motores:

```bash
# Ligar motores
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"
```

```bash
# Desligar motores
ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}"
```

> Observação: O nome do serviço pode ser alterado no `params.yaml`.

---

## 🛠️ Configuração via `params.yaml`

Você pode ajustar os seguintes parâmetros editando o arquivo:

```
rpi_gpio_manager/gpio_manager/config/params.yaml
```

Exemplo:

```yaml
gpio_config:
  pin_motor_1: 18
  pin_motor_2: 20
  service_name: "motor_power"
  response1: "Motores ligados com sucesso."
  response2: "Motores desligados e estado inicial restaurado."
  messages:
    true: "Motores ligados."
    false: "Motores desligados."
```

Ao modificar esses valores, você pode:

- Alterar os pinos utilizados
- Renomear o serviço ROS 2
- Mudar as mensagens de resposta do serviço

---

## 📁 Estrutura do Projeto

```
rpi_gpio_manager/
├── package.xml
├── CMakeLists.txt
├── setup.py
├── config/
│   └── params.yaml
└── rpi_gpio_manager/
    ├── __init__.py
    └── gpio_node.py
```

---

## 🧪 Como funciona

O nó `gpio_node.py`:

- Carrega os parâmetros do `params.yaml`
- Define os pinos BCM usados para controle de motores
- Levanta um serviço chamado `/motor_power` (ou outro, se alterado)
- Quando recebe `data: true`, liga o motor (GPIO HIGH/LOW)
- Quando recebe `data: false`, desliga o motor e retorna ao estado inicial
- Garante limpeza dos GPIOs ao finalizar o nó

---

## 📝 Exemplo de saída do nó

```bash
[INFO] [gpio_manager]: GPIO Manager iniciado...
[INFO] [gpio_manager]: Serviço '/motor_power' pronto.
[INFO] [gpio_manager]: Ativando motores...
[INFO] [gpio_manager]: Desligando motores...
[INFO] [gpio_manager]: GPIOs limpos.
```

---

## 💡 Dica: Modificando o nome do serviço

Se quiser alterar o nome do serviço, basta editar no `params.yaml`:

```yaml
service_name: "gpio_action"
```

E recompilar:

```bash
colcon build --packages-select rpi_gpio_manager
source install/setup.bash
```
---

## 👥 Contribuição

Contribuições são bem-vindas! Sinta-se à vontade para abrir issues, pull requests ou sugerir melhorias.

---

## © Licença

MIT License – veja o arquivo `LICENSE` para detalhes.

---

## 📈 Status do projeto

Em desenvolvimento ativo. Utilizado na versão LYSA V2.1.X.

---

## 👨‍💻 Autores e reconhecimento

Desenvolvido por:  
👤 Jordan Souza <xulipasouza@hotmail.com>  
🛠 AllJordanSS (GitLab)

Agradecimento especial aos testadores e colaboradores da comunidade ROS e GitLab.

---

## 📦 Roadmap futuro

- Integração com interface web para controle remoto
- Suporte a PWM e sensores via GPIO
---


