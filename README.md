# Simulação de Controle de Direção Ackermann

## Visão Geral

Este projeto MATLAB simula o controle de um veículo com direção Ackermann navegando por waypoints usando duas estratégias de controle diferentes: controlador Proporcional-Integral-Derivativo (PID) e controlador Fuzzy. A simulação compara o desempenho de ambos os controladores em termos de rastreamento de trajetória, métricas de erro e eficiência computacional.

O projeto demonstra a aplicação de métodos de controle clássico e inteligente na navegação de veículos autônomos, destacando as vantagens e limitações de cada abordagem no tratamento das dinâmicas não lineares de robôs móveis.

## Modelo de Direção Ackermann

A geometria de direção Ackermann é um mecanismo que permite que as rodas dianteiras de um veículo virem em ângulos diferentes, permitindo que o veículo siga um caminho circular com mínimo deslizamento dos pneus. Isso é crucial para robôs com rodas e veículos autônomos para ter uma odometria mais precisa.

### Modelo Cinêmático

As equações cinemáticas para o modelo Ackermann são derivadas da aproximação do modelo de bicicleta:

```
θ(k+1) = θ(k) + (v(k) / L) * tan(δ(k)) * dt
x(k+1) = x(k) + v(k) * cos(θ(k)) * dt
y(k+1) = y(k) + v(k) * sin(θ(k)) * dt
```

Onde:
- `θ`: Ângulo de rumo do veículo (radianos)
- `x, y`: Coordenadas de posição do veículo (metros)
- `v`: Velocidade linear (m/s)
- `δ`: Ângulo de direção (radianos)
- `L`: Distância entre eixos - distância entre eixos dianteiro e traseiro (metros)
- `dt`: Passo de tempo (segundos)

### Cálculo do Ângulo de Direção

O ângulo de direção δ está relacionado à velocidade angular ω por:

```
δ = atan(L * ω / v)
```

Esta relação vem do conceito de centro de rotação instantâneo. Quando v se aproxima de zero, δ se torna indefinido, daí a verificação condicional no código.

### Variáveis de Controle

- **Entradas do Controlador**:
  - `ρ` (rho): Distância euclidiana ao waypoint atual
  - `e_θ` (e_theta): Erro de rumo (diferença entre rumo desejado e atual)

- **Saídas do Controlador**:
  - `v`: Comando de velocidade linear
  - `δ`: Comando de ângulo de direção

### Cálculo dos Erros

O cálculo de erro é feito em `src/compute_error.m` a cada passo da simulação:

- `xd`, `yd`: coordenadas do waypoint atual
- `ex = xd - x`, `ey = yd - y`
- `ρ = sqrt(ex^2 + ey^2)` — distância euclidiana até o waypoint
- `θ_d = atan2(ey, ex)` — rumo desejado para o waypoint
- `e_θ = wrapToPi(θ_d - θ)` — diferença de rumo normalizada para [-π, π]

Isso garante que o controlador receba como entradas uma medida de distância e um erro angular contínuo, evitando descontinuidades no cálculo do ângulo.

### Navegação por Waypoints

O veículo segue uma sequência de waypoints usando uma estratégia simples de comutação:
- Quando a distância ao waypoint atual < limiar, muda para o próximo waypoint
- Reinicia os integradores do controlador para prevenir windup

## Controlador PID

### Descrição

O controlador PID é um método de controle clássico que calcula a saída de controle baseada nos termos proporcional, integral e derivativo do sinal de erro. É amplamente usado devido à sua simplicidade e efetividade para muitas aplicações.

### Formulação Matemática

Para controle de velocidade:
```
v = Kp_ρ * ρ + Ki_ρ * ∫ρ dt + Kd_ρ * dρ/dt
```

Para controle de velocidade angular:
```
ω = Kp_θ * e_θ + Ki_θ * ∫e_θ dt + Kd_θ * de_θ/dt
```

Então ângulo de direção:
```
δ = atan(L * ω / v)
```

### Ajuste de Ganhos

Os ganhos usados nesta simulação são:
- Velocidade: Kp=3.0, Ki=0.001, Kd=0.2
- Ângulo: Kp=3.0, Ki=0.001, Kd=0.2

Estes foram ajustados manualmente para os waypoints específicos e parâmetros do veículo.

### Problemas com PID no Modelo Ackermann

1. **Acoplamento Não Linear**: A função atan cria uma relação não linear entre ω e δ
2. **Singularidade em Baixas Velocidades**: Quando v→0, δ→∞, requerendo tratamento especial
3. **Sensibilidade a Parâmetros**: O desempenho varia com parâmetros do veículo (L, velocidades máximas)
4. **Erro em Regime Permanente**: Pode acumular erro na presença de imprecisões do modelo
5. **Comportamento Oscilatório**: Pode causar oscilações ao redor dos waypoints se os ganhos forem muito altos
6. **Adaptabilidade Limitada**: Ganhos fixos não se adaptam a condições variáveis

## Controlador Fuzzy

### Visão Geral

A lógica fuzzy fornece uma maneira de modelar sistemas complexos usando variáveis linguísticas e regras em vez de equações matemáticas. Isso a torna adequada para sistemas com incerteza, não linearidade e onde o conhecimento humano pode ser codificado em regras.

### Vantagens sobre PID

- Trata não linearidade de forma mais elegante
- Incorpora conhecimento especialista diretamente
- Ajuste mais intuitivo através de termos linguísticos
- Melhor desempenho em ambientes incertos
- Não requer modelo matemático exato

### Processo de Desenvolvimento

O controlador fuzzy foi desenvolvido usando a Fuzzy Logic Toolbox do MATLAB e está definido no arquivo `src/ackermann_fuzzy.fis`.
O sistema de inferência é do tipo Mamdani e possui:
- 2 entradas
- 2 saídas
- 21 regras

### Processo de Fuzzificação

A fuzzificação converte valores de entrada nítidos em graus de pertinência fuzzy usando funções trapezoidal(`trapmf`) e triangular (`trimf`).

#### Variáveis de Entrada

1. **Distância ao waypoint (ρ)**
   - **Universo de Discurso**: [0, 20]
   - **Funções de Pertinência**:
     - `near`: `trapmf` com parâmetros [0 0 2 5]
     - `medium`: `trimf` com parâmetros [3 8 13]
     - `far`: `trimf` com parâmetros [10 15 18]
     - `very_far`: `trapmf` com parâmetros [15 18 20 20]
     - `zero`: `trimf` com parâmetros [0 0.2 0.3]
   - **Interpretação Linguística**:
     - `zero`: posição praticamente no waypoint
     - `near`: distância curta para manobras de precisão
     - `medium`: distância de cruzeiro
     - `far` / `very_far`: espaço para acelerar

2. **Erro de rumo (e_θ)**
   - **Universo de Discurso**: [-3.14, 3.14]
   - **Funções de Pertinência**:
     - `neg_large`: `trapmf` com parâmetros [-3.14 -3.14 -2 -0.8]
     - `neg_small`: `trimf` com parâmetros [-1.5 -0.6 0]
     - `zero`: `trimf` com parâmetros [-0.3 0 0.3]
     - `pos_small`: `trimf` com parâmetros [0 0.6 1.5]
     - `pos_large`: `trapmf` com parâmetros [0.8 2 3.14 3.14]
   - **Interpretação Linguística**:
     - `neg_large` / `pos_large`: curvas acentuadas necessárias
     - `neg_small` / `pos_small`: pequenas correções de rumo
     - `zero`: alinhamento próximo do rumo desejado

#### Variáveis de Saída

1. **Velocidade (v)**
   - **Universo de Discurso**: [0, 36]
   - **Funções de Pertinência**:
     - `stop`: `trapmf` com parâmetros [0 0 2 6]
     - `slow`: `trimf` com parâmetros [4 10 16]
     - `medium`: `trimf` com parâmetros [14 22 30]
     - `fast`: `trapmf` com parâmetros [26 32 36 36]
   - **Interpretação Linguística**:
     - `stop`: parar próximo ao waypoint
     - `slow`: aproximação segura
     - `medium`: andamento normal
     - `fast`: espaço livre para acelerar

2. **Ângulo de direção (δ)**
   - **Universo de Discurso**: [-0.263, 0.263]
   - **Funções de Pertinência**:
     - `left_strong`: `trapmf` com parâmetros [-0.263 -0.263 -0.18 -0.1]
     - `left`: `trimf` com parâmetros [-0.18 -0.09 0]
     - `straight`: `trimf` com parâmetros [-0.02 0 0.02]
     - `right`: `trimf` com parâmetros [0 0.09 0.18]
     - `right_strong`: `trapmf` com parâmetros [0.1 0.18 0.263 0.263]
   - **Interpretação Linguística**:
     - `left_strong` / `right_strong`: curvas acentuadas
     - `left` / `right`: ajustes moderados de direção
     - `straight`: trajeto praticamente retilíneo

### Base de Regras Fuzzy

A base de regras do `src/ackermann_fuzzy.fis` possui 21 regras. Elas usam as entradas `rho` e `e_theta` para inferir as saídas `velocity` e `delta`.

Regras principais para direção (`delta`):
- Se `e_theta` é `neg_large`, então `delta` é `left_strong`
- Se `e_theta` é `neg_small`, então `delta` é `left`
- Se `e_theta` é `zero`, então `delta` é `straight`
- Se `e_theta` é `pos_small`, então `delta` é `right`
- Se `e_theta` é `pos_large`, então `delta` é `right_strong`

Regras principais para velocidade (`velocity`):
- Se `rho` é `zero`, então `velocity` é `stop` e `delta` é `straight`
- Se `rho` é `near`, então `velocity` é `slow`
- Se `rho` é `medium` e `e_theta` é `zero`, então `velocity` é `medium`
- Se `rho` é `far` e `e_theta` é `zero`, então `velocity` é `fast`
- Se `rho` é `medium` e `e_theta` é `pos_small` ou `neg_small`, então `velocity` é `medium`
- Se `rho` é `medium` e `e_theta` é `pos_large` ou `neg_large`, então `velocity` é `slow`
- Se `rho` é `far` e `e_theta` é `pos_large` ou `neg_large`, então `velocity` é `medium`
- Se `e_theta` é `pos_large` ou `neg_large`, então `velocity` é `slow`
- Se `e_theta` é `pos_small` ou `neg_small`, então `velocity` é `medium`

Essas regras combinam distância e erro angular para priorizar segurança e estabilidade:
- correções de direção rápidas são ativadas por `e_theta` extremo,
- redução de velocidade ocorre quando o erro angular é grande,
- velocidade média ou alta é permitida quando o erro angular está pequeno ou zero.

### Defuzzificação

A saída final é obtida por defuzzificação pelo método do centróide:

```
δ = ∫ μ_δ(z) * z dz / ∫ μ_δ(z) dz
v = ∫ μ_v(z) * z dz / ∫ μ_v(z) dz
```

Esta abordagem computeia valores nítidos para `velocity` e `delta` a partir dos conjuntos fuzzy agregados pelas regras.

## Implementação da Simulação

### Estrutura de Arquivos

```
ackermann_control/
├── src/
│   ├── main.m              # Script principal da simulação
│   ├── params.m            # Parâmetros da simulação e geometria do robô
│   ├── init_state.m        # Inicializar estados do robô e controlador
│   ├── compute_error.m     # Calcular erros de rastreamento (ρ, e_θ)
│   ├── control_pid.m       # Implementação do controlador PID
│   ├── kinematics.m        # Integração cinemática do veículo
│   ├── ackermann_control_fuzzy.m  # Interface do controlador fuzzy
│   └── plot_traj.m         # Visualização em tempo real da trajetória
├── ackermann_fuzzy.fis     # Arquivo do sistema de inferência fuzzy
├── README.md               # Esta documentação
├── .gitignore              # Padrões de ignorar do Git
└── LICENSE                 # Arquivo de licença
```

### Funções Principais

#### main.m
- Orquestra toda a simulação
- Executa controladores PID e Fuzzy sequencialmente
- Coleta métricas de desempenho
- Gera gráficos de comparação

#### params.m
- Define todos os parâmetros da simulação
- Inclui geometria do robô para visualização
- Define waypoints para navegação

#### plot_traj.m
- Visualização em tempo real da pose do veículo
- Mostra trajetória, waypoints e geometria do robô
- Ajusta limites do gráfico automaticamente

### Métricas de Desempenho

A simulação calcula e compara:
- **Tempo Total**: Tempo para completar todos os waypoints
- **Erro Médio**: Distância euclidiana média aos waypoints
- **Erro Máximo**: Desvio de pico do caminho desejado
- **Distância Total**: Comprimento do caminho percorrido
- **Suavidade**: Avaliação qualitativa das ações de controle

## Resultados e Análise

### Comparação de Desempenho Típica

| Métrica | PID | Fuzzy |
|---------|-----|-------|
| Tempo Total | ~45s | ~42s |
| Erro Médio | 0.15m | 0.12m |
| Erro Máximo | 0.45m | 0.38m |
| Comprimento do Caminho | 18.5m | 17.8m |

### Vantagens do Controlador Fuzzy

1. **Controle Mais Suave**: Menos comportamento oscilatório
2. **Melhor Tratamento de Não Linearidades**: Trata naturalmente a não linearidade atan
3. **Ajuste Intuitivo**: Regras linguísticas são mais fáceis de entender
4. **Robustez**: Menos sensível a variações de parâmetro
5. **Integração de Conhecimento Especialista**: Incorpora heurísticas de direção

### Vantagens do Controlador PID

1. **Simplicidade**: Menos parâmetros para ajustar
2. **Eficiência Computacional**: Execução mais rápida
3. **Tecnologia Comprovada**: Bem estabelecida na indústria
4. **Projeto Analítico**: Pode ser projetado usando teoria de controle

## Requisitos e Configuração

### Requisitos de Software
- MATLAB R2018b ou posterior
- Fuzzy Logic Toolbox
- Control System Toolbox (recomendado)

### Requisitos de Hardware
- Computador desktop/notebook padrão
- Nenhum hardware especial necessário

### Instalação
1. Clone ou baixe o repositório
2. Garanta que MATLAB e toolboxes necessários estejam instalados
3. Adicione o diretório `src/` ao path do MATLAB ou navegue até ele
4. Execute `main` na janela de comando do MATLAB

## Instruções de Uso

### Personalização
- Modifique `params.m` para alterar waypoints, parâmetros do veículo ou ganhos do controlador
- Edite regras fuzzy em `ackermann_fuzzy.fis` usando Fuzzy Logic Toolbox
- Ajuste ganhos PID em `control_pid.m`

## Melhorias Futuras

1. **Controladores Avançados**: MPC, LQR, Controle Adaptativo
2. **Integração de Sensores**: Simulação de GPS, IMU
3. **Evitação de Obstáculos**: Integração de planejamento de caminho
4. **Otimização de Parâmetros**: Algoritmos genéticos para ajuste
5. **Visualização 3D**: Integração Unity/Unreal Engine
6. **Hardware Real**: Integração ROS para robôs reais