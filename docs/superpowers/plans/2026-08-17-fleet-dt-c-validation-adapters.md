# Fleet-DT C — Validação e adaptadores (§V + §III) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tornar os números do §V reprodutíveis por `make bench`, e expor as fronteiras de integração que o abstract cita — Ardupilot, WeBots, RTSP, MCS e as simulações paralelas do DTE.

**Architecture:** Benchmarks são binários em `tools/`, não testes: eles medem e imprimem, e o teste que os acompanha checa apenas invariantes que não dependem da máquina. Os adaptadores vivem em `adapters/`, cada um atrás de um alvo de Makefile próprio, para que `make lib && make test` continue sem dependência externa. O que depende de WeBots ou de hardware é declarado fronteira e documentado, não simulado com número inventado.

**Tech Stack:** C18. Opcionais e isolados: `libmosquitto` (Plano B), WeBots R2023b+ (`libController`), `c_library_v2` do MAVLink (header-only).

**Spec:** [`docs/spec/paper-claims.md`](../../spec/paper-claims.md)
**Depende de:** [Plano A](2026-08-17-fleet-dt-a-model-core.md) completo, [Plano B](2026-08-17-fleet-dt-b-transport-qos.md) tasks 1–6.

## Global Constraints

- Herda as constraints dos Planos A e B.
- **Um benchmark nunca asserta um número da máquina.** Ele imprime o medido e compara contra o do paper, marcando divergência como `DIVERGE`, não como falha. O que falha é invariante estrutural (contadores fecham, nenhum vaso perdido).
- **O teto de ~25 barcos é dos injetores** (Armadilha 2 do spec). Todo relatório de escala imprime dois números rotulados: capacidade do DTI e capacidade do injetor.
- **Tempo de δ e latência de atuação são medições separadas** (Armadilha 3). Nunca aparecem no mesmo número.
- Adaptador que precisa de SDK ausente compila para um alvo que **pula com aviso**, nunca quebra o build.

---

### Task 1: Injetor de telemetria sintética

Quita a primeira metade de **C26**.

**Files:**
- Create: `tools/injector/injector.h`, `tools/injector/injector.c`, `tools/injector/main.c`
- Test: `tests/test_injector.c`
- Modify: `Makefile` (alvo `tools`)

**Interfaces:**
- Consumes: `fdt_transport_t`, `fdt_env_t`, `fdt_enc_input`, `fdt_tick_t`, `fdt_reg_t`.
- Produces: `fdt_inj_t`; `fdt_inj_init(fdt_inj_t*, fdt_transport_t*, unsigned vessels, double hz, uint32_t seed) -> int`; `fdt_inj_tick(fdt_inj_t*) -> int` (vasos publicados neste tick, ou -1); `fdt_inj_published(const fdt_inj_t*) -> uint64_t`; `fdt_inj_missed_deadlines(const fdt_inj_t*) -> uint64_t`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_injector.c`:

```c
#include "../tools/injector/injector.h"
#include <fleet_dt/transport.h>
#include <assert.h>
#include <stdio.h>

static unsigned seen;
static void count_msg(const char *topic, const uint8_t *buf, size_t len,
                      void *user)
{
    (void)topic; (void)buf; (void)len; (void)user;
    seen++;
}

int main(void)
{
    /* §V-B: "Fleet DT was validated in simulation using injectors (MQTT
     * clients) that periodically published synthetic telemetry data into the
     * network, mimicking the real boats." */
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);

    fdt_inj_t inj;
    assert(fdt_inj_init(&inj, &tr, 4, 8.0, 12345) == 0);
    assert(fdt_inj_init(&inj, NULL, 4, 8.0, 1) == -1);
    assert(fdt_inj_init(&inj, &tr, 0, 8.0, 1) == -1);

    for (unsigned v = 0; v < 4; v++) {
        char topic[64];
        assert(fdt_topic(topic, sizeof topic, "lsdt", v) == 0);
        assert(tr.subscribe(tr.self, topic, count_msg, NULL) == 0);
    }

    seen = 0;
    assert(fdt_inj_tick(&inj) == 4);
    tr.poll(tr.self, 0);
    assert(seen == 4);                       /* um I^t por vaso, por tick */
    assert(fdt_inj_published(&inj) == 4);

    /* Sequência monotônica por vaso: é o que deixa o framesync do Plano B
     * distinguir pacote novo de pacote velho. */
    seen = 0;
    assert(fdt_inj_tick(&inj) == 4);
    tr.poll(tr.self, 0);
    assert(seen == 4);
    assert(fdt_inj_published(&inj) == 8);

    /* Mesma semente, mesma telemetria: a campanha do §V-B tem que ser
     * repetível entre execuções. */
    fdt_inj_t a, b;
    assert(fdt_inj_init(&a, &tr, 1, 8.0, 777) == 0);
    assert(fdt_inj_init(&b, &tr, 1, 8.0, 777) == 0);
    fdt_input_t ia, ib;
    fdt_inj_sample(&a, 0, &ia);
    fdt_inj_sample(&b, 0, &ib);
    assert(ia.wz_rps == ib.wz_rps && ia.vbat_v == ib.vbat_v);

    printf("test_injector: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.**

- [ ] **Step 3: Implementar.** `fdt_inj_sample(fdt_inj_t*, unsigned vessel, fdt_input_t *out)` gera as 21 entradas da Tabela I a partir de um PRNG xorshift32 semeado por `seed ^ vessel`, com faixas plausíveis para o barco Jundiá: `vbat_v` em torno de 18.5 V (§II), `ibat_a` positivo, `press_pa` perto de 101325, `gps_lat_deg`/`gps_lon_deg` na região de Porto Alegre. `fdt_inj_tick` codifica, envelopa com `seq` monotônico por vaso, e publica em `fleet/{v}/lsdt`. `main.c` é um laço com `fdt_tick_t` que aceita `--vessels`, `--hz`, `--seconds`, `--seed`.

- [ ] **Step 4: Rodar e ver passar.**

- [ ] **Step 5: Commit.** `git commit -m "feat: synthetic telemetry injector mimicking the real boats"`

---

### Task 2: Benchmark de escala

Quita **C25** e a segunda metade de **C26**. Aqui mora a Armadilha 2.

**Files:**
- Create: `tools/bench/bench_scale.c`
- Modify: `Makefile` (alvo `bench`)

**Interfaces:**
- Consumes: `fdt_fleet_t`, `fdt_coord_t`, `fdt_feas_t`, `fdt_inj_t`, `fdt_tick_t`.
- Produces: binário `tools/bench/bench_scale`, que aceita `--vessels N --frames F --window n`.

**Semeadura obrigatória antes do frame 0.** A Task 3 do Plano A rejeita um
passo com janela `n` quando a fila guarda menos de `n` estados, então o bench
chama `fdt_twin_seed` **`n` vezes por vaso** antes do laço. Sem isso,
`--window 4` retorna -1 no primeiro frame e o bench reporta uma falha que não
existe.

- [ ] **Step 1: Escrever o benchmark**

O laço, e o relatório que ele imprime:

```
== fleet-dt scale bench ==
vessels        : 25
frames         : 400
window depth   : 4
--- DTI side (what Section V-B calls negligible) ---
delta worst    : 0.412 ms   budget 125.000 ms
delta mean     : 0.087 ms
feasible       : yes
cpu per DTI    : 0.03 %     paper: < 1 %       [OK]
--- injector side (what actually capped the paper at ~25 boats) ---
injector ticks : 400
missed ticks   : 0
injector cap   : > 25 vessels on this machine
--- frame integrity (Section V-B pathology) ---
partial frames : 0
double updates : 0
stale packets  : 0
```

Duas seções rotuladas, porque o §V-B atribui o teto aos injetores: "*hard-programming
injectors to inject packets periodically could not keep the simulation pace for
larger fleets (> 25 boats, same computer model)*". Um relatório que juntasse os
dois números diria que o DTI satura em 25, que não é o que o paper afirma.

"cpu per DTI" é o tempo de δ agregado dividido pelo tempo de parede e pelo número
de vasos, e é comparado com o `< 1 %` do §V-B. Divergência imprime `[DIVERGE]`
com os dois valores; não falha.

- [ ] **Step 2: Estender o Makefile**

```make
BENCHSRC = $(wildcard tools/bench/bench_*.c)
BENCHBIN = $(BENCHSRC:.c=)

.PHONY: bench

tools/bench/bench_%: tools/bench/bench_%.c $(LIB) tools/injector/injector.o
	$(CC) $(CFLAGS) -Itools $^ -lm -o $@

bench: $(BENCHBIN)
	@for b in $(BENCHBIN); do echo "== $$b"; ./$$b; done
```

- [ ] **Step 3: Rodar.** Run: `make bench`. Expected: relatório acima, `feasible: yes`, integridade zerada com loopback (que não perde pacote).

- [ ] **Step 4: Commit.** `git commit -m "bench: DTI and injector scaling reported as separate ceilings"`

---

### Task 3: Benchmark de banda e link budget

Quita **C20**.

**Files:**
- Create: `tools/bench/bench_bandwidth.c`
- Create: `docs/link-budget.md`

**Interfaces:**
- Consumes: `fdt_link_t`, `fdt_stream_t`, `fdt_reg_t`.

- [ ] **Step 1: Escrever o benchmark.** Perfis de fluxo tirados do §II e do §III, um por linha do relatório:

| fluxo | payload | taxa | origem no paper |
|---|---|---|---|
| `imu` | 8 B × 3 eixos | 8 Hz | §III, "8 bytes per IMU axis" |
| `gps` | 12 B | 8 Hz | §II, GPS no NAVIO2 |
| `baro` | 4 B | 8 Hz | §II, barômetro de 10 cm |
| `power` | 8 B | 8 Hz | §II, sensores de tensão e corrente do Arduino Micro |
| `lsdt_agg` | 48 KB | 8 Hz | §V-A, "48 KB payload" |
| `hsdt_cam` | 250 MB/s | — | §III, "1080p30 generates approximately 250 MB/s of image data" |

O relatório imprime, para 1 e para 25 vasos, a utilização de um enlace de
100 Mbps (§III) com e sem os reguladores da Task 4 do Plano B, e a linha do
§V-A: `paper: < 1% increase over a 125 ms window`. O fluxo `hsdt_cam` aparece
numa seção própria, rotulada **fora do MQTT**, porque o §III o move para o
RTSP — somá-lo ao orçamento MQTT reproduziria um número que o paper não afirma.

- [ ] **Step 2: Escrever `docs/link-budget.md`** com a tabela acima, a conta de cada linha, e o registro explícito de que 48 KB (§V-A) e 48 bytes (§IV) são grandezas distintas.

- [ ] **Step 3: Rodar.** Run: `make bench`. Expected: relatório com as duas colunas de vasos e a linha de comparação com o paper.

- [ ] **Step 4: Commit.** `git commit -m "bench: link budget over the LSDT and HSDT stream profiles"`

---

### Task 4: Benchmark de latência — δ contra ida e volta

Quita **C23**. Armadilha 3 do spec.

**Files:**
- Create: `tools/bench/bench_latency.c`

**Interfaces:**
- Consumes: `fdt_feas_t`, `fdt_transport_t`, `fdt_env_t`.

- [ ] **Step 1: Escrever o benchmark.** Duas medições, impressas em blocos separados e nunca somadas:

```
--- delta compute time (Section IV feasibility) ---
worst          : 0.412 ms
budget         : 125.000 ms
feasible       : yes            paper: "Running delta in less than 125 ms is feasible"

--- actuation round trip (Section V-A) ---
I^t publish -> A^t delivered
worst          : 3.180 ms
mean           : 1.940 ms
note           : Section V-A observes actuation arriving late even when delta
                 is feasible, "as it has to travel back through the network".
                 This is the second measurement, not a component of the first.
```

Sobre loopback o RTT é sub-milissegundo; o valor só fica realista sobre o
adaptador MQTT. O binário aceita `--transport loop|mqtt` e imprime qual usou,
porque um número de loopback rotulado como rede seria enganoso.

- [ ] **Step 2: Rodar.** Run: `make bench`. Expected: dois blocos distintos.

- [ ] **Step 3: Commit.** `git commit -m "bench: delta compute time and actuation round trip as separate figures"`

---

### Task 5: Benchmark de jitter do frame

Quita **C21**, na parte que não depende do WeBots.

**Files:**
- Create: `tools/bench/bench_jitter.c`

- [ ] **Step 1: Escrever o benchmark.** Mede o desvio de cada frame em relação ao deadline de 125 ms ao longo de 400 frames com N vasos, e imprime p50, p95, p99, máximo e a contagem de overruns. §V-A afirma "MQTT introduced no notable latency, nor did WeBots' visual feedback (3D model) suffer from stuttering"; a metade do MQTT é mensurável aqui, e a metade do WeBots é fronteira — o relatório diz isso em uma linha em vez de inventar um número.

- [ ] **Step 2: Rodar e commitar.** `git commit -m "bench: frame jitter against the 125 ms deadline"`

---

### Task 6: Adaptador MAVLink — ingest do Ardupilot

Quita a parte "Ardupilot" de **C3**.

**Files:**
- Create: `adapters/mavlink/fdt_mavlink.h`, `adapters/mavlink/fdt_mavlink.c`
- Create: `docs/mavlink-mapping.md`
- Test: `tests/test_mavlink_map.c`
- Modify: `Makefile` (alvo `mavlink`)

**Interfaces:**
- Produces: `fdt_mav_ingest_t`; `fdt_mav_init(fdt_mav_ingest_t*) -> void`; `fdt_mav_on_scaled_imu(fdt_mav_ingest_t*, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag) -> void`; `fdt_mav_on_gps_raw(...)`; `fdt_mav_on_scaled_pressure(...)`; `fdt_mav_on_battery(...)`; `fdt_mav_on_local_ned(...)`; `fdt_mav_input(const fdt_mav_ingest_t*) -> const fdt_input_t*`; `fdt_mav_complete(const fdt_mav_ingest_t*) -> int`; e a saída `fdt_mav_actuation_to_rc(const fdt_actuation_t*, uint16_t *chan_throttle, uint16_t *chan_cage) -> void`.

O núcleo do trabalho é a tabela de mapeamento, e ela é o entregável:

| Tabela I | Unidade | Mensagem MAVLink | Campo | Conversão |
|---|---|---|---|---|
| `a_x, a_y, a_z` | m/s² | `SCALED_IMU2` | `xacc, yacc, zacc` | mG → m/s²: `× 9.80665 / 1000` |
| `ω_x, ω_y, ω_z` | rad/s | `SCALED_IMU2` | `xgyro, ygyro, zgyro` | mrad/s → rad/s: `/ 1000` |
| `m_x, m_y, m_z` | µT | `SCALED_IMU2` | `xmag, ymag, zmag` | mgauss → µT: `× 0.1` |
| `φ_gps, λ_gps` | deg | `GPS_RAW_INT` | `lat, lon` | `× 1e-7` |
| `h_gps` | m | `GPS_RAW_INT` | `alt` | mm → m: `/ 1000` |
| `v_N, v_E, v_D` | m/s | `LOCAL_POSITION_NED` | `vx, vy, vz` | direto |
| `P` | Pa | `SCALED_PRESSURE` | `press_abs` | hPa → Pa: `× 100` |
| `T` | °C | `SCALED_PRESSURE` | `temperature` | c°C → °C: `/ 100` |
| `V_b` | V | `BATTERY_STATUS` | `voltages[0]` | mV → V: `/ 1000` |
| `I_b` | A | `BATTERY_STATUS` | `current_battery` | cA → A: `/ 100` |
| `X_left, X_right` | — | — | — | **não vem do MAVLink**: caminho HSDT, §III |
| `τ` | % | `RC_CHANNELS_OVERRIDE` | `chan3_raw` | 0–100 % → 1000–2000 µs |
| `α` | rad | `RC_CHANNELS_OVERRIDE` | `chan1_raw` | ±0.6 rad → 1000–2000 µs |

- [ ] **Step 1: Escrever o teste.** Valores conhecidos por linha da tabela: `xacc = 1000` vira `9.80665 m/s²`; `lat = -300500000` vira `-30.05°`; `press_abs = 1013.25 hPa` vira `101325 Pa`; `voltages[0] = 18500` vira `18.5 V`; `throttle_pct = 50` vira `chan3_raw = 1500`; `cage_rad = 0` vira `chan1_raw = 1500`. `fdt_mav_complete` retorna 1 só depois que as cinco famílias de mensagem chegaram.

- [ ] **Step 2: Implementar** e escrever `docs/mavlink-mapping.md` com a tabela acima e a nota de que as duas vistas de câmera não trafegam por aqui.

- [ ] **Step 3: Rodar e commitar.** `git commit -m "feat: MAVLink ingest mapping the 21 Table I entries from Ardupilot"`

---

### Task 7: Módulo WeBots

Quita a parte "WeBots" de **C3** e **C8**.

**Files:**
- Create: `adapters/webots/fdt_webots_controller.c`, `adapters/webots/README.md`
- Modify: `Makefile` (alvo `webots`)

**Interfaces:**
- Consumes: `fdt_coord_t`, `fdt_tick_t`, `fdt_transport_t`, `fdt_framesync_t`.
- Produces: um controller WeBots que roda `δ` para N vasos.

§IV: "*A custom module within WeBots implements δ in C language and carries out
the dynamics of the model.*" O controller é essa afirmação virando código:

- `wb_robot_init()`, depois `wb_robot_step(125)` amarrado a `FDT_TICK_NS / 1e6`, para que o tick do WeBots **seja** o tick do modelo;
- assina `fleet/+/lsdt` pelo transporte, alimenta o `framesync`, e monta `I^t` por vaso;
- chama `fdt_coord_step` uma vez por tick;
- escreve `B^t` nos nós `Supervisor` de cada casco (`wb_supervisor_field_set_sf_vec3f` para posição, `set_sf_rotation` para atitude), que é a referência visual 3D do §I feature (iii);
- publica `A^t` em `fleet/{v}/act`.

`adapters/webots/README.md` documenta o mundo mínimo (dois `Robot` com
`Supervisor TRUE`), como apontar o `WEBOTS_HOME`, e registra que a medição de
CPU do §V-A ("10% for the first boat and less than 1% for subsequent boats") é
**fronteira**: depende do WeBots e não é reproduzida por `make bench`.

- [ ] **Step 1: Escrever o controller e o README.**
- [ ] **Step 2: Alvo de Makefile** que pula com aviso quando `$(WEBOTS_HOME)` não está definido.
- [ ] **Step 3: Commit.** `git commit -m "feat: WeBots controller running delta at the simulation tick"`

---

### Task 8: Fronteira HSDT — RTSP

Quita **C11**.

**Files:**
- Create: `adapters/rtsp/fdt_rtsp.h`, `adapters/rtsp/fdt_rtsp_fake.c`, `adapters/rtsp/README.md`
- Test: `tests/test_rtsp_fake.c`

**Interfaces:**
- Produces: `fdt_view_t {const void *data; size_t len; unsigned width, height; uint32_t seq;}`; `fdt_rtsp_t` com `open`, `next_frame`, `close`; `fdt_rtsp_fake(fdt_rtsp_t*, unsigned width, unsigned height) -> int` — gera frames sintéticos sem rede.

O ponto do §III é que este caminho **não passa pelo MQTT**: "*Separating the
camera feed from the MQTT infrastructure reduced latency while improving the
DTI's response time.*" A fronteira existe para que `fdt_input_t.x_left` e
`x_right` sejam preenchidos por aqui e não pelo codec do Plano B, e o teste
verifica exatamente isso: um `I^t` que atravessou o codec chega com
`FDT_VIEW_PRESENT`, e é o RTSP que troca o sentinela pela imagem real.

- [ ] **Step 1: Escrever o teste, implementar o fake, commitar.** `git commit -m "feat: HSDT camera boundary kept off the MQTT path"`

---

### Task 9: Registro de simulações do DTE

Quita **C7**.

**Files:**
- Create: `include/fleet_dt/dte.h`, `src/dte.c`
- Test: `tests/test_dte.c`

**Interfaces:**
- Produces: `typedef void (*fdt_sim_fn)(size_t tick, void *user)`; `fdt_dte_t`; `fdt_dte_init(fdt_dte_t*, fdt_sim_slot_t *slots, size_t cap) -> int`; `fdt_dte_register(fdt_dte_t*, const char *name, fdt_sim_fn, void *user) -> int`; `fdt_dte_tick(fdt_dte_t*) -> size_t`; `fdt_dte_count(const fdt_dte_t*) -> size_t`.

§I feature (ii): "*it supports running multiple simulations in parallel in the
same DTE*". §III lista o que roda ali além do WeBots: um simulador de fluidos e
aplicações de ML de processamento de imagem, detecção e desvio de obstáculo,
compondo o Application Space. O registro é o que amarra todas ao mesmo tick —
que é exatamente o que o §VI diz ainda **não** estar no modelo formal (item D2),
então o header registra a distinção: o DTE as sincroniza, o modelo não as
descreve.

- [ ] **Step 1: Teste** que registra três simulações, chama `fdt_dte_tick` cinco vezes, e verifica que cada uma viu os ticks 0..4 na ordem, que registrar além da capacidade retorna -1, e que o índice do tick é compartilhado.
- [ ] **Step 2: Implementar e commitar.** `git commit -m "feat: DTE registry ticking parallel simulations off one clock"`

---

### Task 10: Exemplo de dois caminhos — filtrado e cru

Quita **C28**.

**Files:**
- Create: `examples/two_paths.c`, e uma seção em `examples/README.md`

**Interfaces:**
- Consumes: `fdt_twin_t`, `fdt_delta_e_fn`, `fdt_pi_fn`.

§V-A: "*Telemetry was collected from a real boat and compared to the pose and
attitude estimation, using a Kalman filter to generate B_i^t. Data from sensors
(I_i^t) were used unfiltered to achieve the lowest possible latency from sensing
to the actuation path.*"

Os dois caminhos no mesmo frame: `δᵉ` roda um Kalman escalar de uma dimensão
sobre guinada, produzindo o `Bᵗ` filtrado que o operador vê; `π` decide a partir
de `Iᵗ` **cru**, guardado no contexto por vaso, e não do `Bᵗ` filtrado. O exemplo
imprime as duas trajetórias lado a lado para que a diferença — que é latência,
não precisão — fique visível.

- [ ] **Step 1: Escrever o exemplo, estender o README, commitar.** `git commit -m "example: filtered state for the operator, raw input for the actuation path"`

---

### Task 11: Fechamento — status das promessas

**Files:**
- Create: `tools/claims/claims_status.sh`
- Modify: `docs/spec/paper-claims.md` (coluna `status`), `README.md`

- [ ] **Step 1: Escrever o script.** Para cada id de `C1` a `C28` e `D1` a `D7`, um `grep` pelo artefato citado na tabela e uma linha de saída: `C5  quita     tests/test_regulator.c`. Ids `fronteira` e `diferido` são impressos como tal, sem procurar arquivo. O script **não** edita a tabela; ele imprime o que a tabela deveria dizer, e a divergência é para o humano resolver.

**C1 é roll-up, não item.** "a fleet-level Digital Twin model and architecture"
é a soma de C2 a C19, não um artefato próprio; procurar arquivo para ele deixa
a afirmação de topo do paper permanentemente em `pendente`. O script trata C1
como derivado: `quita` quando todo id de C2 a C19 está em `quita` ou
`fronteira`, e `pendente` caso contrário, imprimindo qual id o segura.

- [ ] **Step 2: Rodar, atualizar a coluna `status` do spec, e apontar o README para o script.**

- [ ] **Step 3: Rodar a suíte inteira do zero.**

Run: `make clean && make lib && make test && make examples && make bench && ./tools/claims/claims_status.sh`
Expected: todos os testes `ok`; nenhum id `C*` restante em `pendente` exceto os marcados `fronteira`.

- [ ] **Step 4: Commit.** `git commit -m "docs: claim status generator closing the paper coverage loop"`

---

## Self-Review

**Cobertura.** C3 Tasks 6, 7 · C7 Task 9 · C8 Task 7 · C11 Task 8 · C20 Task 3 ·
C21 Task 5 · C23 Task 4 · C25 Task 2 · C26 Tasks 1, 2 · C28 Task 10.
C22 permanece **fronteira** por construção: medir CPU do WeBots exige o WeBots.

**Armadilhas endereçadas.** Item 2 (teto dos injetores) é a estrutura do
relatório da Task 2. Item 3 (δ vs RTT) é a estrutura da Task 4. Item 1 (48 B vs
48 KB) reaparece documentado em `docs/link-budget.md`.

**Não construído, por decisão.** D1 a D7 do spec. A Task 9 encosta em D2 e o
header registra a fronteira: o DTE sincroniza as simulações paralelas, o modelo
formal não as descreve — que é precisamente o que o §VI declara faltar.
