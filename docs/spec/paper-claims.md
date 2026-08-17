# Inventário de promessas do paper

Manuscrito rastreado: **`ICECS_2026_Gemeos_Digitais-10.pdf`**, revisão `-10`.

> A. R. P. Domingues, C. J. D. Silva, F. da R. Lui, J. Maia, R. S. Cenço,
> C. A. M. Marcon, F. G. Moraes. *A Digital Twin Model and Architecture for
> Monitoring and Controlling Fleets of Autonomous Unmanned Surface Vehicles.*
> ICECS 2026.

Estrutura desta revisão, porque a numeração já mudou uma vez e voltará a mudar:

| Seção | Título |
|---|---|
| I | Introduction and Related Work |
| II | The Physical Unit: Jundiá Boat |
| III | The Digital Twin Architecture |
| **IV** | **The Digital Twin Model** — equações (1)–(6) |
| V | Validation, Results, and Discussion (A. Vessel DTI, B. Fleet DT) |
| VI | Conclusion |

Equações desta revisão. A eq. (2) carrega δ **e** π na mesma linha; revisões
anteriores as separavam, e é daí que vem qualquer mapa código↔paper defasado
em uma equação a partir da (3).

| Eq | Conteúdo |
|---|---|
| (1) | matrizes `Bᵢᵗ`, `Iᵢᵗ`, `Aᵢᵗ` |
| (2) | `Bᵢᵗ = δ(Bᵢᵗ⁻¹, Iᵢᵗ⁻¹, gᵢᵗ⁻¹)` e `Aᵢᵗ = π(Bᵢᵗ, gᵢᵗ)` |
| (3) | `Bᵢᵗ = δᵉ([Bᵢᵗ⁻¹; Bᵢᵗ⁻ⁿ], I_kᵗ⁻¹, g_kᵗ⁻¹), i ≤ j ≤ t−1` |
| (4) | `Fᵗ = [Bᵗ  δᵉ  cᵗ]` |
| (5) | `Fᵗ = Δ(Fᵗ⁻¹, Iᵗ⁻¹)` |
| (6) | `Fᵗ = Δᵉ([Fᵗ⁻¹; Fᵗ⁻ⁿ], Iᵗ⁻¹)` |

## Como ler a tabela

`status` é o estado do repositório, não do paper:

- **quita** — existe artefato que torna a afirmação verificável por `make test`
  ou `make bench`.
- **fronteira** — o repo expõe a interface; o sistema real vive fora
  (Ardupilot, WeBots, MCS, `lsa-pucrs/boat-digital-twin`).
- **diferido** — o próprio paper declara não entregue (§VI). Implementar
  **desalinha**.
- **pendente** — ainda não construído.

¹ **Verificado em 2026-08-17 nesta máquina.** WeBots R2025a e libmosquitto 2.0.22
foram instalados, e os dois adaptadores deixaram de ser código não linkado:

- `make webots` compilou o controller contra o SDK real **de primeira** — os
  headers-stub de `tools/stubs/` estavam corretos;
- o mundo Jundiá rodou **674.360 frames** sob o coordenador, viável o tempo
  todo, pior δ de 117 µs contra o orçamento de 125 ms;
- `--mode=realtime` renderizou sem um único erro de GL, que é a referência
  visual 3D do §I(iii);
- `make mqtt-test` fez ida e volta por um broker mosquitto real, com o estado
  chegando idêntico byte a byte.

O texto abaixo é o registro de por que essas linhas eram `fronteira` antes.

**Existir não é compilar.** Dois adaptadores — MQTT e WeBots — não podem ser
linkados sem seus SDKs, que não estão nesta máquina. `make syntax` os
type-checa contra headers-stub em `tools/stubs/`, o que separa "escrito" de
"sabidamente compila", mas não substitui um build real: um erro de premissa
sobre a API sobrevive ao stub.

Por isso C3 e C8 ficam em **fronteira**, não em `quita`: um arquivo que nunca
passou por um linker não é evidência de integração. Desde 2026-08-17 o mundo
WeBots, as malhas e o controller estão versionados e `tests/test_world.c` checa
que os dois concordam entre si — DEFs resolvem, malhas existem, o frame de
125 ms divide em passos inteiros de física. Isso é mais forte que antes e ainda
é mais fraco que o paper afirma. C2 fica em `quita` porque o
que ele afirma — a arquitetura assenta em MQTT — é a costura `transport.h` mais
o loopback, ambos testados, mais a config de bridge versionada; o cliente
mosquitto é a implementação daquela costura, não a afirmação.

C22 mudou de natureza mas não de status. O benchmark existe, roda e mede: três
mundos idênticos exceto no número de vasos, mediana de N execuções, e a
diferença entre eles.

Com amostras de 10 s, duas execuções discordaram **até no sinal** — uma deu o
segundo barco em −1,60%, que é fisicamente impossível e portanto ruído puro.
Com 25 s × 5 execuções (`FDT_CPU_SAMPLE_S=25 FDT_CPU_REPEATS=5`) a dispersão
caiu para menos de 1% e o resultado estabilizou. Duas execuções cuidadosas:

| mundo | vasos | CPU mediana | dispersão | 1º barco | 2º barco |
|---|---|---|---|---|---|
| run A | 0/1/2 | 8,76 / 10,28 / 10,80% | ≤2,08% | +1,52% | +0,52% |
| run B | 0/1/2 | 8,84 / 10,08 / 10,64% | ≤0,88% | **+1,24%** | +0,56% |

**O primeiro barco está medido**: +1,24% com piso de ruído de 0,88%, e as duas
execuções concordam dentro de 0,3 ponto. O paper publica 10%. Isso é
`[DIVERGE]` — uma discordância real, não ruído.

**O segundo barco continua sob o piso**: +0,56% contra 0,88% de dispersão. O
valor é compatível com o "< 1%" do paper, mas compatível não é medido, e o
relatório diz `[BOUNDARY]`.

**A forma da afirmação se reproduz**: primeiro casco entre duas e três vezes o
segundo, porque o renderizador, o mundo físico e o fluido são pagos uma vez só.
É essa forma que sustenta a arquitetura — uma frota é viável porque o vaso N+1
é quase de graça.

Sobre a discordância de 1,24% contra 10%: o número do paper é de outra máquina
e de uma configuração de renderização que não conhecemos. Este run usa
`--minimize`, que reduz o trabalho de desenho, e roda numa GPU que o §V-A não
descreve. Reproduzir o 10% exigiria a configuração deles, não uma correção
aqui.

O status desta tabela é gerado por `make report`, que percorre a lista e
imprime o que ela deveria dizer. Ele nunca edita a tabela: divergência entre as
duas é exatamente o que um humano precisa olhar.

## A. Abstract

| id | Afirmação | § | Artefato que quita | Status |
|---|---|---|---|---|
| C1 | "a fleet-level Digital Twin model and architecture for AUSVs" | Abstract | **roll-up**: quita quando C2–C19 estiverem em `quita` ou `fronteira` | quita |
| C2 | "The architecture relies on the MQTT protocol" | Abstract, §III | `transport.h` + loopback testados; `adapters/mqtt/` com round trip real via `make mqtt-test` | quita |
| C3 | "integrates the Ardupilot firmware, the WeBots simulator, and other simulation applications into a single system" | Abstract | `adapters/mavlink/` (testado), `adapters/webots/` — mundo + controller compilados contra o SDK real e executados | quita ¹ |
| C4 | "Link-budget modeling is validated in the context of the Jundiá Project's fleet" | Abstract, §V-A | `include/fleet_dt/linkbudget.h` + `tools/bench/link_budget` | quita |
| C5 | "introduces bandwidth regulators that guarantee QoS while maximizing the use of shared wireless links" | Abstract, §III | `include/fleet_dt/regulator.h` + `tests/test_regulator.c` | quita |

C5 é a contribuição declarada do paper. Sem ela o repo não quita o abstract.

## B. Features declaradas no §I

| id | Afirmação | Artefato | Status |
|---|---|---|---|
| C6 | "(i) it models fleets as a DTA composed of DTIs per-vessel" | `fdt_fleet_t` sobre `fdt_twin_t` | quita |
| C7 | "(ii) it supports running multiple simulations in parallel in the same DTE" | `adapters/sim/` — registro de simulações amarradas ao mesmo tick | quita |
| C8 | "(iii) it provides a near-real-time 3D visual reference for the mission operator" | mundo renderizado em `--mode=realtime`, controller escrevendo pose a 125 ms | quita ¹ |

## C. Arquitetura, §III

| id | Afirmação | Artefato | Status |
|---|---|---|---|
| C9 | brokers "connected in bridge mode to avoid service interruption during temporary connection instability" | `config/mosquitto/bridge.conf` versionado + teste de partição | quita |
| C10 | LSDT: pacotes pequenos, "e.g., 8 bytes per IMU axis", baixo overhead vs. 100 Mbps disponíveis | `linkbudget` alimentado pelo perfil LSDT | quita |
| C11 | HSDT: "Separating the camera feed from the MQTT infrastructure reduced latency while improving the DTI's response time" | `adapters/rtsp/` + fake testado; o codec nunca carrega imagem | quita |
| C12 | reguladores "overcome this problem by dropping the number of samples in the MQTT client", enquanto "real sensors continue sampling at their own pace, as this is necessary for control algorithms" | `regulator.h` + teste com dois caminhos (publicação decimada, controle intacto) | quita |

## D. Modelo, §IV

| id | Afirmação | Artefato | Status |
|---|---|---|---|
| C13 | "The simulation frequency is 8 Hz, i.e. δ is a hard real-time task with deadline of 125 ms" | `FDT_TICK_NS` + `tick.h` | quita |
| C14 | "a state (Bᵢᵗ) occupies 12 floating point values in memory, translating to 48 bytes"; "the queue would grow by 23 KB per minute elapsed, per vessel" | asserção estática `sizeof(fdt_state_t) == 48`; `fdt_queue_bytes(d) == 48*d`; 23040 B/min a 8 Hz | quita |
| C15 | "The DTI is *feasible* only if δ can be computed in less than \|t_k − t_{k−1}\| for any arbitrary k" | `feasibility.h` — tempo de δ **por vaso, por frame** | quita |
| C16 | "In a non-autonomous vehicle, actuation is absorbed by the state, i.e. `Aᵢᵗ ⊆ Bᵢᵗ`" | `fdt_twin_init_passive` | quita |
| C17 | "A fleet DT is *homogeneous* when δᵉ is the same for all vessels. Oppositely, heterogeneous fleet DTs require indexing δᵉ" | ponteiro `δᵉ` por twin | quita |
| C18 | "The coordinator (S) computes cᵗ from the vessel states it receives, in the same step in which it distributes gᵢᵗ" | `coordinator.h` + store de `Bᵗ` (cilindro da Fig. 4) | quita |
| C19 | Tabela I fixa 21 entradas de `Iᵗ`; unidades de p,q,r em rad/s e ângulos em "rad or deg" | `model.h`, campos nomeados com unidade | quita |

## E. Validação, §V

| id | Afirmação | Artefato | Status |
|---|---|---|---|
| C20 | "Due to the size of packets (48 KB payload plus camera feed frame), the bandwidth usage increased < 1% for an update window of 125 ms" | `tools/bench/link_budget` — payload de **48 KB**, não 48 bytes | quita |
| C21 | "MQTT introduced no notable latency, nor did WeBots' visual feedback (3D model) suffer from stuttering" | `tools/bench/jitter` sobre o transporte | quita |
| C22 | "running WeBots adds 10% CPU usage for the first boat and less than 1% for subsequent boats" | `tools/bench/bench_webots_cpu.c` + os três mundos — medido; a **forma** da afirmação se reproduz, os valores absolutos não, e nenhum incremento passa o piso de ruído deste host | fronteira |
| C23 | "Running δ in less than 125 ms is feasible. However, actuation is delivered late to the boat, as it has to travel back through the network" | duas medições distintas: tempo de δ (`feasibility`) e RTT de atuação (`bench/latency`) | quita |
| C24 | "we added a range of states to δᵉ, thereby enabling proactive operation, as in model predictive control (MPC)" | exemplo com janela de profundidade > 1 | quita |
| C25 | "the resources required to add more DTIs to the fleet are negligible (< 1% CPU usage per DTI)" | `tools/bench/scale` | quita |
| C26 | "hard-programming injectors to inject packets periodically could not keep the simulation pace for larger fleets (> 25 boats, same computer model)" — limite **dos injetores**, não do DTI | `tools/injector/` + `bench/scale` reportando os dois limites separados | quita |
| C27 | "the state of some boats was *partially* updated... the state of some boats was updated twice within the same simulation frame" | contadores de frame parcial e de update duplo, alimentados por número de sequência no **envelope** | quita |
| C28 | "Telemetry was collected from a real boat and compared to the pose and attitude estimation, using a Kalman filter to generate Bᵢᵗ. Data from sensors (Iᵢᵗ) were used unfiltered to achieve the lowest possible latency from sensing to the actuation path" | dois caminhos expressáveis: δᵉ filtrado, π sobre `I` bruto | quita |

## F. Diferido pelo §VI — **não implementar**

Construir qualquer um destes contradiz o texto publicado.

| id | Declarado como futuro | § |
|---|---|---|
| D1 | "there is no *formal* connection between the input model and the state transition function δᵉ" | VI |
| D2 | descrever fluidos e ML no modelo, "at least to synchronize them with the DTI pace (simulation tick)" | VI |
| D3 | "Using MQTT to transmit telemetry data from within the firmware" | VI |
| D4 | reguladores "directly in the firmware or the broker, e.g., as a QoS/real-time rule" | VI |
| D5 | "adopting the real-time transport protocol *under* MQTT" (RTP, ref. [18]) | VI |
| D6 | "Dropping late packets at the receiver" | VI |
| D7 | "Field validation campaign for the DTI fleet is on schedule at the time of writing" — resultado de frota é **HILS apenas** | V |

## G. Armadilhas registradas

Cada uma já produziu, ou produziria, um teste que passa e mente.

1. **48 bytes ≠ 48 KB.** O estado ocupa 48 **bytes** (§IV). O payload medido no
   §V-A é 48 **KB**. Grandezas independentes.
2. **O teto de ~25 barcos é dos injetores.** §V-B diz que os *injetores* não
   sustentaram o pace. Benchmark que reporte isso como limite do DTI deturpa o
   paper. Reporte os dois números separados.
3. **Tempo de δ ≠ latência de atuação.** §V-A afirma δ viável em <125 ms *e*
   atuação chegando atrasada. Duas medições.
4. **Timestamp: modelo vs. envelope.** A Tabela I não lista timestamp, então os
   tipos do modelo não carregam um. O **envelope de rede** carrega número de
   sequência, sem o qual a patologia C27 não é observável. Detectar e contar —
   **não corrigir**, porque o paper a deixa como questão aberta (C27, D6).
5. **`cᵗ` é entrada e saída do mesmo frame.** Eq. (4) o coloca dentro de `Fᵗ` e
   §IV diz que `S` o calcula "in the same step in which it distributes gᵢᵗ".

## H. Ambiguidades do manuscrito

Registradas aqui porque o código precisa escolher, e a escolha tem que ser
rastreável até uma linha do paper.

1. **Janela do δᵉ.** A eq. (3) escreve o colchete como `[Bᵢᵗ⁻¹; Bᵢᵗ⁻ⁿ]` — os `n`
   estados mais recentes — mas anexa a restrição `i ≤ j ≤ t−1`, e `i` e `j` não
   aparecem na equação. O texto resolve a favor de `n`: "The number of states to
   observe (n) is purely a design decision", e o limite `48d` é sobre
   profundidade. **Decisão: a API primária toma `n`.** `i ≤ j ≤ t−1` é tratado
   como notação residual.
2. **Subscrito trocado.** A mesma eq. (3) abre com `Bᵢᵗ` e `Bᵢᵗ⁻ⁿ`, e no meio
   passa a `I_kᵗ⁻¹`, `g_kᵗ⁻¹`. `i` e `k` designam o mesmo vaso.
3. **Unidade dos ângulos de atitude.** Tabela I dá φ, θ, ψ como "rad or deg" sem
   escolher, enquanto p, q, r são rad/s. **Decisão: graus para ângulos**,
   seguindo o `boat.h` de referência, com os campos nomeados
   `roll_deg`/`pitch_deg`/`yaw_deg` e `roll_rate_rps`/`pitch_rate_rps`/
   `yaw_rate_rps`. Consequência: qualquer δᵉ que integre taxa em ângulo cruza
   unidade, não soma cru.
4. **`Bᵢ¹` vs. `Bᵢ⁰`.** §IV diz "`Bᵢ¹` is the initial state of the iᵗʰ vessel,
   which must be a known starting state", enquanto a recorrência da eq. (3) pede
   um `t−1` anterior ao primeiro passo. O código trata o estado inicial como
   condição de contorno semeada, não como algo que δᵉ produz.
5. **O período dos 48 KB, e o que "increased < 1%" mede.** §V-A: "Due to the
   size of packets (48 KB payload plus camera feed frame), the bandwidth usage
   increased < 1% for an update window of 125 ms". Com 48 KB *por janela de
   125 ms*, a taxa é 48 × 1024 × 8 × 8 = 3,15 Mbps, ou **3,1%** de um enlace de
   100 Mbps — não fecha com "< 1%". Duas saídas, e o manuscrito não escolhe:
   ou os 48 KB são por segundo e não por janela (0,38%, que fecha), ou
   "increased" é razão contra o tráfego preexistente e não ocupação absoluta.
   **Decisão: o código não escolhe.** `fdt_link_utilization` calcula a leitura
   absoluta, `fdt_link_increase` a relativa, e o benchmark imprime as duas
   rotuladas. Nenhum teste asserta o "< 1%".

Itens 1, 2, 3 e 5 valem como correção para o manuscrito.

## I. Constraints de entrega (definidas pelo autor, 2026-08-17)

Valem para os três planos e sobrescrevem qualquer decisão anterior em contrário.

1. **Implementação em C.** Nenhuma parte do sistema é reimplementada em outra
   linguagem. Scripts de shell só orquestram; não calculam.
2. **Todo artefato de código é em inglês** — identificadores, comentários,
   docstrings, mensagens de saída, nomes de arquivo. Os documentos de
   planejamento e este spec permanecem em português. Cada símbolo público
   carrega uma docstring no header dizendo o que faz, o que devolve, e qual
   linha do paper ele quita.
3. **Todo benchmark produz artefato visual.** Cada medição escreve três
   arquivos em `results/`: `<name>.csv` com os dados brutos, `<name>.svg` com o
   gráfico, e `<name>.txt` com o relatório humano. O plotter é C puro emitindo
   SVG — sem gnuplot, sem matplotlib, sem dependência externa.
4. **As saídas são montadas num README.** `tools/report/` varre `results/` e
   gera a seção de resultados do `README.md`, com os SVGs embutidos e as
   tabelas de comparação contra os números publicados do §V.
