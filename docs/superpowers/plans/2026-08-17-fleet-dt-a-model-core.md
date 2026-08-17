# Fleet-DT A — Núcleo do modelo (§IV) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implementar em C puro o modelo do §IV do paper — tipos, fila de estados, δ/δᵉ/π, agregado de frota, pacer de 125 ms, predicado de viabilidade, store de `Bᵗ` e coordenador `S` — com cada figura publicada checada por `make test`.

**Architecture:** Biblioteca estática sem dependência externa. A lib é dona da estrutura (fila, janela, frame, frota); a aplicação é dona da dinâmica (δᵉ e π entram por ponteiro de função). Toda memória é do chamador: nenhum `malloc`, nenhum estado de escopo de arquivo, para que uma frota de N vasos seja N structs independentes.

**Tech Stack:** C18, `gcc`, `make`, `ar`. POSIX.1-2008 `clock_nanosleep`/`clock_gettime` (glibc; a libc da Apple não fornece `TIMER_ABSTIME`). Testes são executáveis próprios, um por módulo, sem framework.

**Spec:** [`docs/spec/paper-claims.md`](../../spec/paper-claims.md)

## Global Constraints

- Manuscrito rastreado: revisão **`-10`**. Modelo no **§IV**, equações **(1)–(6)**. Toda referência em comentário ou doc cita seção e equação desta revisão.
- `CFLAGS = -std=c18 -Wall -Wextra -Werror -pedantic-errors -O2 -Iinclude`. Warning é erro.
- `sizeof(fdt_state_t)` é **48** por asserção estática. Não é medição, é contrato (C14).
- `Iᵗ` carrega exatamente as **21 entradas da Tabela I** — sem timestamp, porque a tabela não lista nenhum (Armadilha 4 do spec).
- Ângulos de atitude em **graus**, taxas em **rad/s**, campos nomeados com a unidade (Ambiguidade 3 do spec).
- Janela do δᵉ é parametrizada por **`n` estados mais recentes** (Ambiguidade 1 do spec), não por par `(i, j)`.
- Prefixo público único: `fdt_` / `FDT_`.
- Commits em Conventional Commits. Autoria é do usuário; nenhum trailer de coautoria de ferramenta.

---

### Task 1: Esqueleto de build e tipos do modelo (eq. 1 + Tabela I)

Quita **C14** e **C19**.

**Files:**
- Create: `Makefile`, `.gitignore`, `.github/workflows/ci.yml`
- Create: `include/fleet_dt/model.h`, `include/fleet_dt/version.h`, `src/version.c`
- Test: `tests/test_model.c`

**Interfaces:**
- Consumes: nada.
- Produces: `fdt_input_t`, `fdt_state_t`, `fdt_actuation_t`, `fdt_goal_t` (alias de `fdt_state_t`), `FDT_STATE_FLOATS` (= 12), `FDT_PAPER_REVISION`, `fdt_version(void) -> const char*`.

`src/version.c` existe nesta task por dois motivos. O primeiro é de build:
`LIBSRC = $(wildcard src/*.c)` com `src/` vazio deixa `LIBOBJ` vazio, e
`ar rcs` sobre zero membros produz um arquivo que algumas versões do GNU ar
recusam e outras aceitam vazio, quebrando o link do teste. O primeiro `.c` do
repositório precisa nascer junto do primeiro alvo. O segundo é de conteúdo: a
revisão do manuscrito fica pregada num símbolo, que é o que torna o
descolamento de numeração detectável em vez de silencioso.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_model.c`:

```c
#include <fleet_dt/model.h>
#include <fleet_dt/version.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* C14, §IV: "a state (B_i^t) occupies 12 floating point values in memory,
 * translating to 48 bytes". Asserção, não medição. */
_Static_assert(sizeof(fdt_state_t) == 48, "eq (1) state must be 48 bytes");
_Static_assert(sizeof(fdt_state_t) == FDT_STATE_FLOATS * sizeof(float),
               "state must be exactly its 12 floats, with no padding");

int main(void)
{
    /* Os 12 campos da eq. (1), na ordem da matriz do paper. */
    fdt_state_t b = {
        .lat_deg = 1, .lon_deg = 2, .alt_m = 3,
        .roll_deg = 4, .pitch_deg = 5, .yaw_deg = 6,
        .surge_mps = 7, .sway_mps = 8, .heave_mps = 9,
        .roll_rate_rps = 10, .pitch_rate_rps = 11, .yaw_rate_rps = 12,
    };
    assert(b.lat_deg == 1.0f && b.yaw_rate_rps == 12.0f);

    /* A^t_k da Tabela I: throttle e ângulo da gaiola. Sem leme. */
    fdt_actuation_t a = { .throttle_pct = 50.0f, .cage_rad = 0.1f };
    assert(a.throttle_pct == 50.0f);

    /* g^t_k "may have the same structure as B^t_k" (§IV). */
    fdt_goal_t g = { .yaw_deg = 90.0f };
    assert(g.yaw_deg == 90.0f);

    /* As 21 entradas da Tabela I, nenhuma a mais. */
    fdt_input_t in = {0};
    in.ax_mps2 = in.ay_mps2 = in.az_mps2 = 1.0f;
    in.wx_rps = in.wy_rps = in.wz_rps = 1.0f;
    in.mx_ut = in.my_ut = in.mz_ut = 1.0f;
    in.gps_lat_deg = in.gps_lon_deg = in.gps_alt_m = 1.0f;
    in.vn_mps = in.ve_mps = in.vd_mps = 1.0f;
    in.press_pa = in.temp_c = in.vbat_v = in.ibat_a = 1.0f;
    in.x_left = in.x_right = NULL;
    assert(in.ibat_a == 1.0f);

    /* A revisão do manuscrito fica pregada num símbolo: o mapa paper↔código
     * já se descolou uma vez, quando delta e pi deixaram de ser equações
     * separadas e viraram as duas metades da eq. (2). */
    assert(strcmp(FDT_PAPER_REVISION, "-10") == 0);
    assert(strstr(fdt_version(), FDT_PAPER_REVISION) != NULL);

    printf("%s\n", fdt_version());
    printf("sizeof(fdt_state_t) = %zu\n", sizeof(fdt_state_t));
    printf("sizeof(fdt_input_t) = %zu\n", sizeof(fdt_input_t));
    printf("test_model: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fatal error: fleet_dt/model.h: No such file or directory`.

- [ ] **Step 3: Escrever o Makefile, o .gitignore e o CI**

`Makefile`:

```make
CC      = gcc
CFLAGS  = -std=c18 -Wall -Wextra -Werror -pedantic-errors -O2 -Iinclude
LIB     = libfleetdt.a
LIBSRC  = $(wildcard src/*.c)
LIBOBJ  = $(LIBSRC:.c=.o)
TESTSRC = $(wildcard tests/test_*.c)
TESTBIN = $(TESTSRC:tests/test_%.c=fleet_dt_test_%)

.PHONY: all lib test clean

all: lib

lib: $(LIB)

$(LIB): $(LIBOBJ)
	ar rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

fleet_dt_test_%: tests/test_%.c $(LIB)
	$(CC) $(CFLAGS) $< $(LIB) -lm -o $@

test: $(TESTBIN)
	@for t in $(TESTBIN); do echo "== $$t"; ./$$t || exit 1; done

clean:
	rm -f $(LIBOBJ) $(LIB) $(TESTBIN)
```

`.gitignore`:

```
*.o
*.a
fleet_dt_test_*
```

`.github/workflows/ci.yml`:

```yaml
name: CI
on:
  push:
    branches: ["**"]
  pull_request:

jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v7
      - name: Build library with -Werror (BLOCKING)
        run: make lib
      - name: Run host unit tests (BLOCKING)
        run: make test
```

- [ ] **Step 4: Escrever `include/fleet_dt/version.h`, `src/version.c` e `include/fleet_dt/model.h`**

`include/fleet_dt/version.h`:

```c
#ifndef FLEET_DT_VERSION_H
#define FLEET_DT_VERSION_H

/* A revisão do manuscrito que este repositório rastreia. O modelo é o §IV e as
 * equações vão de (1) a (6); numa revisão anterior delta e pi eram equações
 * separadas, o que deslocava tudo a partir da (3). Pregar a revisão num
 * símbolo é o que torna o próximo deslocamento detectável. */
#define FDT_PAPER_REVISION "-10"
#define FDT_VERSION        "0.1.0"

/* "fleet-dt 0.1.0 (ICECS 2026 manuscript -10)" */
const char *fdt_version(void);

#endif /* FLEET_DT_VERSION_H */
```

`src/version.c`:

```c
#include <fleet_dt/version.h>

const char *fdt_version(void)
{
    return "fleet-dt " FDT_VERSION " (ICECS 2026 manuscript " FDT_PAPER_REVISION ")";
}
```

`include/fleet_dt/model.h`:

```c
#ifndef FLEET_DT_MODEL_H
#define FLEET_DT_MODEL_H

/* Tipos do modelo Fleet-DT, §IV de "A Digital Twin Model and Architecture for
 * Monitoring and Controlling Fleets of Autonomous Unmanned Surface Vehicles",
 * ICECS 2026, revisão -10.
 *
 * Unidades vêm da Tabela I. O paper mistura convenções — ângulos de atitude em
 * graus, taxas em rad/s — e a mistura é preservada, não normalizada, com o nome
 * de cada campo carregando sua unidade para que a mistura não seja aplicada por
 * acidente. Qualquer δ^e que integre taxa em ângulo cruza a unidade.
 *
 * fdt_state_t deriva de dt-daemon/include/boat.h, de Anderson Domingues em
 * lsa-pucrs/boat-digital-twin, repositório privado; isto é crédito, não um link
 * que o leitor possa seguir. */

/* Os 12 valores em ponto flutuante da eq. (1). §IV publica a largura que eles
 * ocupam, e test_model fixa sizeof(fdt_state_t) nela. */
#define FDT_STATE_FLOATS 12

/* I^t_k, eq. (1) e Tabela I: as leituras de sensor do vaso k no instante t.
 * Exatamente as 21 entradas que a Tabela I lista e nada além: a tabela não
 * declara timestamp, então este tipo também não. As duas vistas de câmera são
 * ponteiros opacos porque são imagens, não escalares; esta biblioteca nunca lê
 * através deles. */
typedef struct {
    float ax_mps2, ay_mps2, az_mps2;  /* a_x, a_y, a_z  aceleração linear, m/s^2 */
    float wx_rps,  wy_rps,  wz_rps;   /* w_x, w_y, w_z  velocidade angular, rad/s */
    float mx_ut,   my_ut,   mz_ut;    /* m_x, m_y, m_z  campo magnético, uT      */

    float gps_lat_deg;   /* phi_gps     latitude, graus                    */
    float gps_lon_deg;   /* lambda_gps  longitude, graus                   */
    float gps_alt_m;     /* h_gps       altitude sobre o nível do mar, m   */

    float vn_mps, ve_mps, vd_mps;     /* v_N, v_E, v_D  velocidade NED, m/s */

    float press_pa;      /* P      pressão atmosférica, Pa      */
    float temp_c;        /* T      temperatura, graus Celsius   */
    float vbat_v;        /* V_b    carga da bateria, volts      */
    float ibat_a;        /* I_b    consumo de corrente, amperes */

    const void *x_left;  /* X_left   vista da câmera estéreo, opaca */
    const void *x_right; /* X_right  vista da câmera estéreo, opaca */
} fdt_input_t;

/* B^t_k, eq. (1): o estado do gêmeo digital. Doze variáveis, na ordem em que a
 * matriz do paper as lista. Precisão simples em todas, porque §IV fixa a
 * largura do estado em 48 bytes e 12 * 4 = 48. */
typedef struct {
    float lat_deg;        /* phi     latitude do vaso, graus            */
    float lon_deg;        /* lambda  longitude do vaso, graus           */
    float alt_m;          /* h       altitude sobre o nível do mar, m   */

    float roll_deg;       /* varphi  ângulo de rolagem, graus           */
    float pitch_deg;      /* theta   ângulo de arfagem, graus           */
    float yaw_deg;        /* psi     ângulo de guinada, graus           */

    float surge_mps;      /* u       velocidade de avanço, m/s          */
    float sway_mps;       /* v       velocidade de deriva, m/s          */
    float heave_mps;      /* w       velocidade de arfagem, m/s         */

    float roll_rate_rps;  /* p       taxa de rolagem, rad/s             */
    float pitch_rate_rps; /* q       taxa de arfagem, rad/s             */
    float yaw_rate_rps;   /* r       taxa de guinada, rad/s             */
} fdt_state_t;

/* A^t_k, eq. (2) e Tabela I. O barco Jundiá esterça girando a gaiola de
 * propulsão; não tem leme. */
typedef struct {
    float throttle_pct;   /* tau    acelerador, por cento               */
    float cage_rad;       /* alpha  ângulo da gaiola de propulsão, rad  */
} fdt_actuation_t;

/* g^t_k, §IV: o objetivo de missão. O paper afirma que ele "may have the same
 * structure as B^t_k, similar to a setpoint in control systems", então é um
 * alias e não um tipo distinto. */
typedef fdt_state_t fdt_goal_t;

#endif /* FLEET_DT_MODEL_H */
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — imprime `sizeof(fdt_state_t) = 48` e `test_model: ok`.

- [ ] **Step 6: Commit**

```bash
git add Makefile .gitignore .github/workflows/ci.yml \
        include/fleet_dt/model.h include/fleet_dt/version.h src/version.c \
        tests/test_model.c
git commit -m "feat: model types of equation (1) and Table I"
```

---

### Task 2: Fila de estados e o limite `48d`

Quita a segunda metade de **C14**.

**Files:**
- Create: `include/fleet_dt/queue.h`, `src/queue.c`
- Test: `tests/test_queue.c`

**Interfaces:**
- Consumes: `fdt_state_t`.
- Produces: `fdt_queue_t`; `fdt_queue_init(fdt_queue_t*, fdt_state_t *storage, size_t cap) -> int`; `fdt_queue_push(fdt_queue_t*, const fdt_state_t*) -> void`; `fdt_queue_len/cap(const fdt_queue_t*) -> size_t`; `fdt_queue_at(const fdt_queue_t*, size_t i) -> const fdt_state_t*` com `i == 0` o **mais antigo**; `fdt_queue_newest(const fdt_queue_t*) -> const fdt_state_t*`; `fdt_queue_bytes(size_t cap) -> size_t`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_queue.c`:

```c
#include <fleet_dt/queue.h>
#include <assert.h>
#include <stdio.h>

int main(void)
{
    fdt_state_t storage[4];
    fdt_queue_t q;

    assert(fdt_queue_init(&q, storage, 4) == 0);
    assert(fdt_queue_init(&q, NULL, 4) == -1);
    assert(fdt_queue_init(&q, storage, 0) == -1);
    assert(fdt_queue_len(&q) == 0);
    assert(fdt_queue_newest(&q) == NULL);
    assert(fdt_queue_at(&q, 0) == NULL);

    for (int k = 0; k < 4; k++) {
        fdt_state_t b = { .yaw_deg = (float)k };
        fdt_queue_push(&q, &b);
    }
    assert(fdt_queue_len(&q) == 4);
    assert(fdt_queue_at(&q, 0)->yaw_deg == 0.0f);   /* mais antigo */
    assert(fdt_queue_newest(&q)->yaw_deg == 3.0f);  /* mais novo   */

    /* Cheia: o push derruba o mais antigo e a janela anda. */
    fdt_state_t b4 = { .yaw_deg = 4.0f };
    fdt_queue_push(&q, &b4);
    assert(fdt_queue_len(&q) == 4);
    assert(fdt_queue_at(&q, 0)->yaw_deg == 1.0f);
    assert(fdt_queue_newest(&q)->yaw_deg == 4.0f);
    assert(fdt_queue_at(&q, 4) == NULL);

    /* C14, §IV: a fila é limitada em 48d bytes por vaso. */
    for (size_t d = 0; d <= 64; d++) {
        assert(fdt_queue_bytes(d) == 48 * d);
    }

    /* "For a 125 ms period, the queue would grow by 23 KB per minute
     * elapsed, per vessel": 48 bytes * 8 Hz * 60 s. */
    size_t per_min = fdt_queue_bytes(8 * 60);
    assert(per_min == 23040);

    printf("queue storage, depth 4 = %zu bytes\n", fdt_queue_bytes(4));
    printf("bytes per minute at 8 Hz = %zu\n", per_min);
    printf("test_queue: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fleet_dt/queue.h: No such file or directory`.

- [ ] **Step 3: Escrever o header**

`include/fleet_dt/queue.h`:

```c
#ifndef FLEET_DT_QUEUE_H
#define FLEET_DT_QUEUE_H

#include <fleet_dt/model.h>
#include <stddef.h>

/* A fila limitada de state frames do §IV: "a custom Webots module implements
 * delta^e as a queue of state frames, allowing for past state inspection".
 *
 * Um state frame é um estado. A eq. (1) declara doze variáveis e nenhum campo
 * de tempo, então a fila guarda fdt_state_t direto e cada entrada custa os 48
 * bytes que o §IV cita. É isso que faz a aritmética do paper fechar: limitar a
 * fila a uma profundidade d a limita em 48d bytes por vaso.
 *
 * O armazenamento é do chamador, então uma frota de N barcos tem N filas
 * independentes, sem alocação e sem estado de escopo de arquivo. Cheia, um
 * push derruba o estado mais antigo. */
typedef struct {
    fdt_state_t *buf;   /* array de cap entradas, do chamador */
    size_t       cap;   /* capacidade em estados, > 0         */
    size_t       len;   /* estados guardados, <= cap          */
    size_t       head;  /* índice do mais antigo em buf       */
} fdt_queue_t;

/* Amarra q ao armazenamento. 0 em sucesso, -1 se storage é NULL ou cap é 0. */
int fdt_queue_init(fdt_queue_t *q, fdt_state_t *storage, size_t cap);

/* Anexa b, derrubando o mais antigo quando já está na capacidade. */
void fdt_queue_push(fdt_queue_t *q, const fdt_state_t *b);

/* Estados guardados agora. */
size_t fdt_queue_len(const fdt_queue_t *q);

/* Capacidade em estados. */
size_t fdt_queue_cap(const fdt_queue_t *q);

/* Estado no índice lógico i, onde 0 é o MAIS ANTIGO guardado. NULL quando i
 * está em len ou além. É o acessor da janela [B^{t-1}; B^{t-n}] da eq. (3). */
const fdt_state_t *fdt_queue_at(const fdt_queue_t *q, size_t i);

/* O estado mais recente, ou NULL com a fila vazia. */
const fdt_state_t *fdt_queue_newest(const fdt_queue_t *q);

/* Bytes que uma fila da capacidade dada ocupa: os 48d do §IV, que é como o
 * paper limita o crescimento por vaso. */
size_t fdt_queue_bytes(size_t cap);

#endif /* FLEET_DT_QUEUE_H */
```

- [ ] **Step 4: Escrever `src/queue.c`**

```c
#include <fleet_dt/queue.h>

int fdt_queue_init(fdt_queue_t *q, fdt_state_t *storage, size_t cap)
{
    if (q == NULL || storage == NULL || cap == 0) {
        return -1;
    }
    q->buf  = storage;
    q->cap  = cap;
    q->len  = 0;
    q->head = 0;
    return 0;
}

void fdt_queue_push(fdt_queue_t *q, const fdt_state_t *b)
{
    if (q == NULL || b == NULL || q->buf == NULL) {
        return;
    }
    if (q->len < q->cap) {
        q->buf[(q->head + q->len) % q->cap] = *b;
        q->len++;
    } else {
        /* Cheia: sobrescreve o mais antigo e avança a janela. */
        q->buf[q->head] = *b;
        q->head = (q->head + 1) % q->cap;
    }
}

size_t fdt_queue_len(const fdt_queue_t *q) { return (q == NULL) ? 0 : q->len; }
size_t fdt_queue_cap(const fdt_queue_t *q) { return (q == NULL) ? 0 : q->cap; }

const fdt_state_t *fdt_queue_at(const fdt_queue_t *q, size_t i)
{
    if (q == NULL || q->buf == NULL || i >= q->len) {
        return NULL;
    }
    return &q->buf[(q->head + i) % q->cap];
}

const fdt_state_t *fdt_queue_newest(const fdt_queue_t *q)
{
    if (q == NULL || q->len == 0) {
        return NULL;
    }
    return fdt_queue_at(q, q->len - 1);
}

size_t fdt_queue_bytes(size_t cap)
{
    return cap * sizeof(fdt_state_t);
}
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — imprime `bytes per minute at 8 Hz = 23040`.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/queue.h src/queue.c tests/test_queue.c
git commit -m "feat: state frame queue bounded at 48d bytes per vessel"
```

---

### Task 3: δ, δᵉ, π e o DTI (eqs. 2 e 3)

Quita **C17**, **C24** e a Ambiguidade 1 do spec.

**Files:**
- Create: `include/fleet_dt/transition.h`, `src/transition.c`
- Test: `tests/test_transition.c`

**Interfaces:**
- Consumes: `fdt_queue_t`, `fdt_state_t`, `fdt_input_t`, `fdt_goal_t`, `fdt_actuation_t`.
- Produces:
  - `typedef void (*fdt_delta_e_fn)(const fdt_queue_t *q, size_t n, const fdt_input_t *in, const fdt_goal_t *g_prev, fdt_state_t *out, void *ctx, void *fleet_ctx)` — `n` é a profundidade da janela da eq. (3): os `n` estados mais recentes, acessíveis por `fdt_window_at(q, n, k)` com `k == 0` o mais recente.
  - `typedef void (*fdt_pi_fn)(const fdt_state_t *b, const fdt_goal_t *g_now, fdt_actuation_t *out, void *ctx, void *fleet_ctx)`
  - `fdt_window_at(const fdt_queue_t *q, size_t n, size_t k) -> const fdt_state_t*`
  - `fdt_twin_t`; `fdt_twin_init(fdt_twin_t*, fdt_state_t *storage, size_t cap, fdt_delta_e_fn, fdt_pi_fn, void *ctx) -> int`; `fdt_twin_seed(fdt_twin_t*, const fdt_state_t*) -> int`; `fdt_twin_depth(const fdt_twin_t*) -> size_t`; `fdt_twin_newest(const fdt_twin_t*) -> const fdt_state_t*`.
  - **Duas aridades, de propósito.** `fdt_twin_step_ctx(fdt_twin_t*, const fdt_input_t*, const fdt_goal_t *g_prev, const fdt_goal_t *g_now, size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out, void *fleet_ctx) -> int` tem **8** argumentos e é a que a frota chama, porque só a frota tem `cᵗ`. `fdt_twin_step(...)` tem os mesmos **7** primeiros e delega passando `NULL` como `fleet_ctx` — é o caso avulso. Escreva as duas; o teste usa as duas.

**Nota para quem executar:** `probe_pi` neste teste lê `fleet_ctx` como
`const float*`, e o `demo_pi` do teste da Task 5 o lê como `demo_ctx_t*`. É
intencional: `cᵗ` é opaco à biblioteca, e cada aplicação escolhe o próprio
tipo. Não uniformize os dois arquivos.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_transition.c`:

```c
#include <fleet_dt/transition.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* delta^e que reporta a profundidade que recebeu e integra a taxa de guinada.
 * A eq. (2) é o caso n == 1. */
static size_t seen_n;

static void probe_delta_e(const fdt_queue_t *q, size_t n,
                          const fdt_input_t *in, const fdt_goal_t *g_prev,
                          fdt_state_t *out, void *ctx, void *fleet_ctx)
{
    (void)g_prev; (void)ctx; (void)fleet_ctx;
    seen_n = n;

    /* k == 0 é o mais recente da janela: o B^{t-1} da eq. (3). */
    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
    out->yaw_deg += (in != NULL) ? in->wz_rps : 0.0f;

    /* k == n-1 é o mais antigo da janela: o B^{t-n}. */
    const fdt_state_t *oldest = fdt_window_at(q, n, n - 1);
    out->pitch_deg = (oldest != NULL) ? oldest->yaw_deg : -1.0f;
    assert(fdt_window_at(q, n, n) == NULL); /* fora da janela */
}

static void probe_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                     fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;
    const float *ceiling = (const float *)fleet_ctx;
    out->cage_rad     = g_now->yaw_deg - b->yaw_deg;
    out->throttle_pct = (ceiling != NULL) ? *ceiling : 100.0f;
}

int main(void)
{
    fdt_state_t storage[8];
    fdt_twin_t  tw;
    fdt_state_t b;
    fdt_actuation_t a;
    fdt_goal_t g = { .yaw_deg = 10.0f };
    fdt_input_t in = { .wz_rps = 1.0f };

    assert(fdt_twin_init(&tw, storage, 8, probe_delta_e, probe_pi, NULL) == 0);
    assert(fdt_twin_init(&tw, storage, 8, NULL, probe_pi, NULL) == -1);

    /* Sem semear, a recorrência não tem B^{t-1}: todo n falha. */
    assert(fdt_twin_depth(&tw) == 0);
    assert(fdt_twin_step(&tw, &in, &g, &g, 1, &b, &a) == -1);

    /* B_i^1, §IV: "the initial state of the i^th vessel, which must be a
     * known starting state". Condição de contorno, não produto de delta^e. */
    fdt_state_t b0 = {0};
    assert(fdt_twin_seed(&tw, &b0) == 0);
    assert(fdt_twin_depth(&tw) == 1);

    /* n == 0 não é janela. n maior que o guardado é rejeitado. */
    assert(fdt_twin_step(&tw, &in, &g, &g, 0, &b, &a) == -1);
    assert(fdt_twin_step(&tw, &in, &g, &g, 2, &b, &a) == -1);

    /* Eq. (2): o caso n == 1. */
    assert(fdt_twin_step(&tw, &in, &g, &g, 1, &b, &a) == 0);
    assert(seen_n == 1);
    assert(fabsf(b.yaw_deg - 1.0f) < 1e-6f);
    assert(fdt_twin_depth(&tw) == 2);

    /* Eq. (3) com profundidade > 1: C24, o que habilita MPC. */
    assert(fdt_twin_step(&tw, &in, &g, &g, 2, &b, &a) == 0);
    assert(seen_n == 2);
    assert(fabsf(b.pitch_deg - 0.0f) < 1e-6f); /* B^{t-n} é o semeado */
    assert(fabsf(b.yaw_deg - 2.0f) < 1e-6f);

    /* Eq. (2) roda sob g^{t-1}, eq. (2, segunda metade) sob g^t. Um frame de
     * re-alvo consome objetivos diferentes nas duas. */
    fdt_goal_t g_prev = { .yaw_deg = 0.0f };
    fdt_goal_t g_now  = { .yaw_deg = 90.0f };
    assert(fdt_twin_step(&tw, &in, &g_prev, &g_now, 1, &b, &a) == 0);
    assert(fabsf(a.cage_rad - (90.0f - b.yaw_deg)) < 1e-6f);

    /* fleet_ctx chega a pi como c^t. */
    float ceiling = 60.0f;
    assert(fdt_twin_step_ctx(&tw, &in, &g, &g, 1, &b, &a, &ceiling) == 0);
    assert(fabsf(a.throttle_pct - 60.0f) < 1e-6f);

    /* Falha não enfileira nem escreve saída. */
    size_t before = fdt_twin_depth(&tw);
    assert(fdt_twin_step(&tw, &in, &g, &g, 999, &b, &a) == -1);
    assert(fdt_twin_depth(&tw) == before);

    printf("test_transition: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fleet_dt/transition.h: No such file or directory`.

- [ ] **Step 3: Escrever o header**

`include/fleet_dt/transition.h`:

```c
#ifndef FLEET_DT_TRANSITION_H
#define FLEET_DT_TRANSITION_H

#include <fleet_dt/model.h>
#include <fleet_dt/queue.h>
#include <stddef.h>

/* delta^e, eq. (3):
 *   B^t_i = delta^e([B^{t-1}_i; B^{t-n}_i], I^{t-1}_k, g^{t-1}_k)
 *
 * A janela é a dos n estados mais recentes, que é o que o colchete da equação
 * escreve e o que o texto do §IV confirma: "The number of states to observe
 * (n) is purely a design decision". A restrição "i <= j <= t-1" anexada à
 * mesma equação usa índices que não aparecem nela; docs/spec/paper-claims.md,
 * Ambiguidade 1, registra a escolha por n.
 *
 * A eq. (2) é o caso n == 1, então não existe delta separado.
 *
 * A biblioteca é dona da fila; a aplicação é dona da dinâmica, e é por isso
 * que isto é ponteiro de função e não rotina de biblioteca.
 *
 * q          a fila, já com o histórico; nunca NULL
 * n          profundidade da janela, 1 <= n <= fdt_queue_len(q)
 * in         I^{t-1}_k, pode ser NULL quando não há entrada nova
 * g_prev     g^{t-1}_k, o objetivo vigente em t-1; nunca NULL
 * out        B^t_i, escrito pela implementação; nunca NULL
 * ctx        contexto por gêmeo, repassado intacto
 * fleet_ctx  c^t da eq. (4) quando uma frota dirige o passo, NULL avulso */
typedef void (*fdt_delta_e_fn)(const fdt_queue_t *q, size_t n,
                               const fdt_input_t *in, const fdt_goal_t *g_prev,
                               fdt_state_t *out, void *ctx, void *fleet_ctx);

/* pi, eq. (2): A^t_i = pi(B^t_i, g^t_i).
 *
 * g_now é o objetivo vigente em t, que não é necessariamente o g_prev que a
 * transição consumiu: num frame de re-alvo a transição roda sob o objetivo
 * antigo enquanto a decisão já decide sob o novo. */
typedef void (*fdt_pi_fn)(const fdt_state_t *b, const fdt_goal_t *g_now,
                          fdt_actuation_t *out, void *ctx, void *fleet_ctx);

/* Estado k da janela dos n mais recentes, com k == 0 o MAIS RECENTE, isto é o
 * B^{t-1} do colchete, e k == n-1 o mais antigo, o B^{t-n}. NULL quando k está
 * em n ou além, ou quando a fila tem menos de n estados. Ordem oposta à de
 * fdt_queue_at de propósito: a fila é indexada por idade absoluta, a janela
 * pela distância ao presente, que é como a eq. (3) a escreve. */
const fdt_state_t *fdt_window_at(const fdt_queue_t *q, size_t n, size_t k);

/* Um Digital Twin Instance. */
typedef struct {
    fdt_queue_t    q;
    fdt_delta_e_fn delta_e;
    fdt_pi_fn      pi;
    void          *ctx;
} fdt_twin_t;

/* Amarra tw ao armazenamento do chamador e à dinâmica da aplicação.
 * 0 em sucesso, -1 com argumento obrigatório NULL ou cap 0. */
int fdt_twin_init(fdt_twin_t *tw, fdt_state_t *storage, size_t cap,
                  fdt_delta_e_fn delta_e, fdt_pi_fn pi, void *ctx);

/* Estabelece B^1_i, o estado inicial que o §IV exige seja conhecido. A eq. (3)
 * precisa de um B^{t-1} para apontar, então a recorrência não parte de fila
 * vazia. 0 em sucesso, -1 com tw desamarrado ou b NULL. Semear duas vezes
 * apenas anexa, que é como uma execução que restaura histórico salvo prepara
 * sua fila. */
int fdt_twin_seed(fdt_twin_t *tw, const fdt_state_t *b);

/* Um frame de simulação: roda delta^e sobre os n mais recentes para produzir
 * B^t_i, enfileira, e roda pi sobre esse mesmo estado para produzir A^t_i.
 *
 * Retorna 0 em sucesso, -1 com tw desamarrado, com g_prev, g_now, b_out ou
 * a_out NULL, com n == 0, ou com n > fdt_twin_depth(tw). Em falha nada é
 * enfileirado e nenhuma saída é escrita. */
int fdt_twin_step_ctx(fdt_twin_t *tw, const fdt_input_t *in,
                      const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                      size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out,
                      void *fleet_ctx);

/* O caso avulso de fdt_twin_step_ctx: sem frota, logo sem c^t. */
int fdt_twin_step(fdt_twin_t *tw, const fdt_input_t *in,
                  const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                  size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out);

/* Estados que este gêmeo guarda. Não é tamanho de janela: o maior n que um
 * passo pode pedir é exatamente este valor. */
size_t fdt_twin_depth(const fdt_twin_t *tw);

/* O estado mais recente, ou NULL se o gêmeo não foi semeado nem passou. */
const fdt_state_t *fdt_twin_newest(const fdt_twin_t *tw);

#endif /* FLEET_DT_TRANSITION_H */
```

- [ ] **Step 4: Escrever `src/transition.c`**

```c
#include <fleet_dt/transition.h>

const fdt_state_t *fdt_window_at(const fdt_queue_t *q, size_t n, size_t k)
{
    size_t len = fdt_queue_len(q);
    if (n == 0 || n > len || k >= n) {
        return NULL;
    }
    /* k conta do presente para trás; a fila conta da idade para a frente. */
    return fdt_queue_at(q, len - 1 - k);
}

int fdt_twin_init(fdt_twin_t *tw, fdt_state_t *storage, size_t cap,
                  fdt_delta_e_fn delta_e, fdt_pi_fn pi, void *ctx)
{
    if (tw == NULL || delta_e == NULL || pi == NULL) {
        return -1;
    }
    if (fdt_queue_init(&tw->q, storage, cap) != 0) {
        return -1;
    }
    tw->delta_e = delta_e;
    tw->pi      = pi;
    tw->ctx     = ctx;
    return 0;
}

int fdt_twin_seed(fdt_twin_t *tw, const fdt_state_t *b)
{
    if (tw == NULL || tw->delta_e == NULL || tw->pi == NULL || b == NULL) {
        return -1;
    }
    if (fdt_queue_cap(&tw->q) == 0) {
        return -1;
    }
    fdt_queue_push(&tw->q, b);  /* B^1_i */
    return 0;
}

int fdt_twin_step_ctx(fdt_twin_t *tw, const fdt_input_t *in,
                      const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                      size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out,
                      void *fleet_ctx)
{
    if (tw == NULL || tw->delta_e == NULL || tw->pi == NULL) {
        return -1;
    }
    if (g_prev == NULL || g_now == NULL || b_out == NULL || a_out == NULL) {
        return -1;
    }
    /* Um gêmeo não semeado guarda zero estados e falha aqui para todo n. */
    if (n == 0 || n > fdt_queue_len(&tw->q)) {
        return -1;
    }

    fdt_state_t b = {0};
    tw->delta_e(&tw->q, n, in, g_prev, &b, tw->ctx, fleet_ctx);
    fdt_queue_push(&tw->q, &b);

    fdt_actuation_t a = {0};
    tw->pi(&b, g_now, &a, tw->ctx, fleet_ctx);

    *b_out = b;
    *a_out = a;
    return 0;
}

int fdt_twin_step(fdt_twin_t *tw, const fdt_input_t *in,
                  const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                  size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out)
{
    return fdt_twin_step_ctx(tw, in, g_prev, g_now, n, b_out, a_out, NULL);
}

size_t fdt_twin_depth(const fdt_twin_t *tw)
{
    return (tw == NULL) ? 0 : fdt_queue_len(&tw->q);
}

const fdt_state_t *fdt_twin_newest(const fdt_twin_t *tw)
{
    return (tw == NULL) ? NULL : fdt_queue_newest(&tw->q);
}
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — `test_transition: ok`.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/transition.h src/transition.c tests/test_transition.c
git commit -m "feat: delta, delta^e and pi over an n-deep window, equations (2) and (3)"
```

---

### Task 4: Gêmeo não-autônomo, `Aᵢᵗ ⊆ Bᵢᵗ`

Quita **C16**.

**Files:**
- Modify: `include/fleet_dt/transition.h`, `src/transition.c`
- Test: `tests/test_transition.c` (acrescenta ao `main` existente)

**Interfaces:**
- Consumes: `fdt_twin_t`, `fdt_delta_e_fn`.
- Produces: `fdt_twin_init_passive(fdt_twin_t*, fdt_state_t *storage, size_t cap, fdt_delta_e_fn, void *ctx) -> int` — instala um π interno que projeta a atuação para fora do estado em vez de decidir.

- [ ] **Step 1: Escrever o teste que falha**

Acrescentar antes do `printf` final de `tests/test_transition.c`:

```c
    /* C16, §IV: "In a non-autonomous vehicle, actuation is absorbed by the
     * state, i.e. A^t_i subset of B^t_i." Sem pi: a atuação é lida do estado.
     * O barco Jundiá esterça pela gaiola, e o estado carrega a guinada, então
     * a projeção é: throttle do avanço, gaiola da taxa de guinada. */
    fdt_state_t pstorage[4];
    fdt_twin_t  passive;
    assert(fdt_twin_init_passive(&passive, pstorage, 4, probe_delta_e, NULL) == 0);
    fdt_state_t p0 = { .surge_mps = 2.0f, .yaw_rate_rps = 0.5f };
    assert(fdt_twin_seed(&passive, &p0) == 0);

    fdt_input_t zero_in = {0};
    fdt_state_t pb;
    fdt_actuation_t pa;
    assert(fdt_twin_step(&passive, &zero_in, &g, &g, 1, &pb, &pa) == 0);
    assert(fabsf(pa.throttle_pct - pb.surge_mps) < 1e-6f);
    assert(fabsf(pa.cage_rad - pb.yaw_rate_rps) < 1e-6f);
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `implicit declaration of function 'fdt_twin_init_passive'`, que com `-Werror` é erro de compilação.

- [ ] **Step 3: Declarar no header**

Acrescentar a `include/fleet_dt/transition.h`, antes do `#endif`:

```c
/* O caso não-autônomo do §IV: "In a non-autonomous vehicle, actuation is
 * absorbed by the state, i.e. A^t_i subset of B^t_i". Não há decisão a tomar,
 * então não há pi a fornecer: a atuação é uma projeção do estado, e este
 * inicializador instala essa projeção. Um vaso teleoperado é assim — quem
 * decide é o operador, e o gêmeo apenas reflete o que o casco está fazendo. */
int fdt_twin_init_passive(fdt_twin_t *tw, fdt_state_t *storage, size_t cap,
                          fdt_delta_e_fn delta_e, void *ctx);
```

- [ ] **Step 4: Implementar em `src/transition.c`**

```c
/* A projeção de A^t_i para fora de B^t_i. A Tabela I dá tau em porcento e
 * alpha em radianos; o estado carrega avanço em m/s e taxa de guinada em
 * rad/s. A gaiola sai da taxa de guinada sem conversão porque as duas são
 * radianos; o acelerador sai do avanço, que é a grandeza que ele comanda. */
static void absorbed_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                        fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)g_now; (void)ctx; (void)fleet_ctx;
    out->throttle_pct = b->surge_mps;
    out->cage_rad     = b->yaw_rate_rps;
}

int fdt_twin_init_passive(fdt_twin_t *tw, fdt_state_t *storage, size_t cap,
                          fdt_delta_e_fn delta_e, void *ctx)
{
    return fdt_twin_init(tw, storage, cap, delta_e, absorbed_pi, ctx);
}
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/transition.h src/transition.c tests/test_transition.c
git commit -m "feat: non-autonomous twin, where actuation is absorbed by the state"
```

---

### Task 5: Frota `Fᵗ`, Δ e Δᵉ (eqs. 4, 5, 6)

Quita **C6**, **C17** e a entrega de `cᵗ`.

**Files:**
- Create: `include/fleet_dt/fleet.h`, `src/fleet.c`
- Test: `tests/test_fleet.c`

**Interfaces:**
- Consumes: `fdt_twin_t`, `fdt_twin_step_ctx`, `fdt_twin_depth`.
- Produces: `fdt_fleet_t` com campos `twins`, `n`, `ctx`; `fdt_fleet_init(fdt_fleet_t*, fdt_twin_t *twins, size_t n, void *ctx) -> int`; `fdt_fleet_step(fdt_fleet_t*, const fdt_input_t *ins, const fdt_goal_t *goals_prev, const fdt_goal_t *goals_now, size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out) -> int`; `fdt_fleet_size(const fdt_fleet_t*) -> size_t`; `fdt_fleet_twin(const fdt_fleet_t*, size_t k) -> fdt_twin_t*`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_fleet.c`:

```c
#include <fleet_dt/fleet.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct { float gain; } vessel_ctx_t;
typedef struct { float ceiling_pct; int seen_by_delta; } demo_ctx_t;

/* Duas dinâmicas distintas: C17, "heterogeneous fleet DTs require indexing
 * delta^e". A frota heterogênea não precisa de máquina extra — cada gêmeo
 * carrega seu próprio ponteiro. */
static void delta_gain(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                       const fdt_goal_t *g_prev, fdt_state_t *out,
                       void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    const vessel_ctx_t *vc = (const vessel_ctx_t *)ctx;
    demo_ctx_t *fc = (demo_ctx_t *)fleet_ctx;
    if (fc != NULL) {
        fc->seen_by_delta = 1;  /* c^t chega ao delta^e, não só ao pi */
    }
    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
    out->yaw_deg += ((in != NULL) ? in->wz_rps : 0.0f) * vc->gain;
}

static void delta_hold(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                       const fdt_goal_t *g_prev, fdt_state_t *out,
                       void *ctx, void *fleet_ctx)
{
    (void)in; (void)g_prev; (void)ctx; (void)fleet_ctx;
    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
}

static void demo_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                    fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;
    const demo_ctx_t *fc = (const demo_ctx_t *)fleet_ctx;
    out->cage_rad     = g_now->yaw_deg - b->yaw_deg;
    out->throttle_pct = (fc != NULL) ? fc->ceiling_pct : 100.0f;
}

int main(void)
{
    fdt_state_t st[2][8];
    fdt_twin_t  twins[2];
    vessel_ctx_t vc[2] = { { .gain = 1.0f }, { .gain = 2.0f } };

    assert(fdt_twin_init(&twins[0], st[0], 8, delta_gain, demo_pi, &vc[0]) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 8, delta_hold, demo_pi, &vc[1]) == 0);

    demo_ctx_t fc = { .ceiling_pct = 60.0f, .seen_by_delta = 0 };
    fdt_fleet_t f;
    assert(fdt_fleet_init(&f, twins, 2, &fc) == 0);
    assert(fdt_fleet_init(&f, NULL, 2, &fc) == -1);
    assert(fdt_fleet_init(&f, twins, 0, &fc) == -1);
    assert(fdt_fleet_size(&f) == 2);
    assert(fdt_fleet_twin(&f, 1) == &twins[1]);
    assert(fdt_fleet_twin(&f, 2) == NULL);

    fdt_input_t ins[2]   = { { .wz_rps = 1.0f }, { .wz_rps = 1.0f } };
    fdt_goal_t  goals[2] = { { .yaw_deg = 10.0f }, { .yaw_deg = 20.0f } };
    fdt_state_t bs[2];
    fdt_actuation_t as[2];

    /* Sem semear, nenhum vaso passa. */
    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == -1);

    fdt_state_t b0 = {0};
    assert(fdt_twin_seed(&twins[0], &b0) == 0);

    /* Um vaso semeado e outro não: a eq. (5) avança a frota como um frame, e
     * uma janela que o vaso 1 não satisfaz deixa o vaso 0 intocado. */
    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == -1);
    assert(fdt_twin_depth(&twins[0]) == 1);

    assert(fdt_twin_seed(&twins[1], &b0) == 0);
    assert(fdt_fleet_step(&f, ins, goals, goals, 1, bs, as) == 0);

    /* delta^e indexado: o vaso 1 segura, o vaso 0 integra com ganho 1. */
    assert(fabsf(bs[0].yaw_deg - 1.0f) < 1e-6f);
    assert(fabsf(bs[1].yaw_deg - 0.0f) < 1e-6f);

    /* c^t da eq. (4) chega aos dois lados do frame. */
    assert(fc.seen_by_delta == 1);
    assert(fabsf(as[0].throttle_pct - 60.0f) < 1e-6f);

    /* ins NULL é o caso sem entrada nova; não é erro. */
    assert(fdt_fleet_step(&f, NULL, goals, goals, 1, bs, as) == 0);
    assert(fabsf(bs[0].yaw_deg - 1.0f) < 1e-6f);

    /* Eq. (6), Delta^e com profundidade maior que um. */
    assert(fdt_fleet_step(&f, ins, goals, goals, 3, bs, as) == 0);
    assert(fdt_fleet_step(&f, ins, goals, goals, 99, bs, as) == -1);

    printf("test_fleet: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fleet_dt/fleet.h: No such file or directory`.

- [ ] **Step 3: Escrever o header**

`include/fleet_dt/fleet.h`:

```c
#ifndef FLEET_DT_FLEET_H
#define FLEET_DT_FLEET_H

#include <fleet_dt/transition.h>
#include <stddef.h>

/* F^t, eq. (4): F^t = [B^t  delta^e  c^t].
 *
 * B^t é o conjunto dos estados dos vasos, guardado dentro da fila de cada
 * gêmeo. delta^e é o conjunto das transições estendidas, guardado por gêmeo,
 * então uma frota heterogênea não precisa de máquina extra: basta dar
 * ponteiros diferentes aos gêmeos, e uma frota homogênea dá o mesmo (C17).
 * c^t é contexto de frota da aplicação, por exemplo distância entre vasos. O
 * §IV põe seu cálculo no coordenador, que o deriva dos estados que recebe no
 * mesmo passo em que distribui g^t_i. Esta biblioteca nunca o lê:
 * fdt_fleet_step o entrega a todo delta^e e a todo pi que dirige, como o
 * argumento fleet_ctx. É assim que c^t é entrada e saída de um mesmo frame na
 * eq. (5) — a transição o enxerga, e a aplicação o atualiza entre frames. */
typedef struct {
    fdt_twin_t *twins;  /* array de n gêmeos, do chamador */
    size_t      n;      /* vasos na frota, > 0            */
    void       *ctx;    /* c^t, opaco a esta biblioteca   */
} fdt_fleet_t;

/* Amarra f aos gêmeos do chamador, que já devem estar inicializados.
 * 0 em sucesso, -1 com twins NULL ou n igual a 0. */
int fdt_fleet_init(fdt_fleet_t *f, fdt_twin_t *twins, size_t n, void *ctx);

/* O Delta da eq. (5), estendido conforme a eq. (6): um frame de frota. Passa
 * todo vaso com sua própria entrada e seus próprios objetivos, sobre a mesma
 * profundidade de janela n.
 *
 * ins         array de n entradas, I^{t-1}; NULL quando nenhum vaso tem
 *             entrada nova, caso em que todo gêmeo enxerga NULL
 * goals_prev  array de n objetivos g^{t-1}_i, consumidos pela eq. (3)
 * goals_now   array de n objetivos g^t_i, consumidos pela eq. (2). Passe o
 *             mesmo array duas vezes quando a missão não re-alveja.
 * n           profundidade da janela da eq. (6)
 *
 * Todo vaso é checado antes que qualquer vaso passe, então uma janela que a
 * fila de algum vaso não satisfaz deixa a frota inteira intocada — a eq. (5)
 * avança a frota como um frame, não como n frames independentes.
 *
 * 0 quando todo vaso passou, -1 com argumento rejeitado ou com algum vaso
 * guardando menos de n estados. Em -1 nenhum vaso passa. */
int fdt_fleet_step(fdt_fleet_t *f, const fdt_input_t *ins,
                   const fdt_goal_t *goals_prev, const fdt_goal_t *goals_now,
                   size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out);

/* Vasos na frota. */
size_t fdt_fleet_size(const fdt_fleet_t *f);

/* Gêmeo no índice k, ou NULL quando k está no tamanho da frota ou além. */
fdt_twin_t *fdt_fleet_twin(const fdt_fleet_t *f, size_t k);

#endif /* FLEET_DT_FLEET_H */
```

- [ ] **Step 4: Escrever `src/fleet.c`**

```c
#include <fleet_dt/fleet.h>

int fdt_fleet_init(fdt_fleet_t *f, fdt_twin_t *twins, size_t n, void *ctx)
{
    if (f == NULL || twins == NULL || n == 0) {
        return -1;
    }
    f->twins = twins;
    f->n     = n;
    f->ctx   = ctx;
    return 0;
}

int fdt_fleet_step(fdt_fleet_t *f, const fdt_input_t *ins,
                   const fdt_goal_t *goals_prev, const fdt_goal_t *goals_now,
                   size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out)
{
    if (f == NULL || f->twins == NULL || f->n == 0) {
        return -1;
    }
    if (goals_prev == NULL || goals_now == NULL ||
        b_out == NULL || a_out == NULL || n == 0) {
        return -1;
    }

    /* Uma janela é compartilhada por vasos cujas filas podem ter comprimentos
     * diferentes, então todo vaso é checado antes que qualquer vaso passe. */
    for (size_t k = 0; k < f->n; k++) {
        const fdt_twin_t *tw = &f->twins[k];
        if (tw->delta_e == NULL || tw->pi == NULL || n > fdt_twin_depth(tw)) {
            return -1;
        }
    }

    for (size_t k = 0; k < f->n; k++) {
        const fdt_input_t *in = (ins != NULL) ? &ins[k] : NULL;
        if (fdt_twin_step_ctx(&f->twins[k], in, &goals_prev[k], &goals_now[k],
                              n, &b_out[k], &a_out[k], f->ctx) != 0) {
            return -1;
        }
    }
    return 0;
}

size_t fdt_fleet_size(const fdt_fleet_t *f)
{
    return (f == NULL) ? 0 : f->n;
}

fdt_twin_t *fdt_fleet_twin(const fdt_fleet_t *f, size_t k)
{
    if (f == NULL || f->twins == NULL || k >= f->n) {
        return NULL;
    }
    return &f->twins[k];
}
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — `test_fleet: ok`.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/fleet.h src/fleet.c tests/test_fleet.c
git commit -m "feat: fleet aggregate, equations (4), (5) and (6)"
```

---

### Task 6: Pacer de 125 ms

Quita **C13**.

**Files:**
- Create: `include/fleet_dt/tick.h`, `src/tick.c`
- Test: `tests/test_tick.c`

**Interfaces:**
- Consumes: nada da lib.
- Produces: `FDT_TICK_NS` (= 125000000L); `fdt_tick_t`; `fdt_tick_start(fdt_tick_t*, long period_ns) -> void`; `fdt_tick_wait(fdt_tick_t*) -> void`; `fdt_tick_overruns(const fdt_tick_t*) -> uint64_t`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_tick.c`:

```c
#include <fleet_dt/tick.h>
#include <assert.h>
#include <stdio.h>

static double elapsed_ms(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) * 1e3 +
           (double)(b->tv_nsec - a->tv_nsec) / 1e6;
}

int main(void)
{
    /* C13, §IV: "The simulation frequency is 8 Hz, i.e. delta is a hard
     * real-time task with deadline of 125 ms." */
    assert(FDT_TICK_NS == 125000000L);

    fdt_tick_t tk;
    fdt_tick_start(&tk, FDT_TICK_NS);
    assert(fdt_tick_overruns(&tk) == 0);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    const int frames = 8;
    for (int i = 0; i < frames; i++) {
        fdt_tick_wait(&tk);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);

    double per_frame = elapsed_ms(&t0, &t1) / frames;
    printf("measured period: %.3f ms over %d frames\n", per_frame, frames);
    /* Deadline absoluto: o jitter de um frame não desloca o próximo, então a
     * média fica presa ao período mesmo sob agendamento ruidoso. */
    assert(per_frame > 120.0 && per_frame < 135.0);
    assert(fdt_tick_overruns(&tk) == 0);

    /* Período inválido cai no padrão do paper. */
    fdt_tick_t bad;
    fdt_tick_start(&bad, 0);
    assert(bad.period == FDT_TICK_NS);

    /* Trabalho que estoura o orçamento conta overrun e re-arma a partir de
     * agora, em vez de girar recuperando um atraso que não recupera. */
    fdt_tick_t fast;
    fdt_tick_start(&fast, 1000000L); /* 1 ms */
    struct timespec busy = { .tv_sec = 0, .tv_nsec = 20000000L };
    nanosleep(&busy, NULL);
    fdt_tick_wait(&fast);
    assert(fdt_tick_overruns(&fast) == 1);

    printf("test_tick: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fleet_dt/tick.h: No such file or directory`.

- [ ] **Step 3: Escrever o header**

`include/fleet_dt/tick.h`:

```c
#ifndef FLEET_DT_TICK_H
#define FLEET_DT_TICK_H

#include <stdint.h>
#include <time.h>

/* §III e §IV: "The DTI updates every 125 ms (this paper); each update in the
 * real world corresponds to a single increment in k", e "The simulation
 * frequency is 8 Hz, i.e. delta is a hard real-time task with deadline of
 * 125 ms". Declarado uma vez, aqui. */
#define FDT_TICK_NS 125000000L

/* Um pacer de deadline absoluto. Guardar o próximo deadline em vez de medir o
 * tempo decorrido a cada frame faz com que o jitter de agendamento não se
 * acumule ao longo dos frames. */
typedef struct {
    struct timespec next;     /* deadline absoluto do próximo frame     */
    long            period;   /* período do frame, ns, > 0              */
    uint64_t        overruns; /* frames cujo trabalho estourou o período*/
} fdt_tick_t;

/* Arma o pacer e põe o primeiro deadline a um período daqui. Período menor ou
 * igual a zero é substituído por FDT_TICK_NS. */
void fdt_tick_start(fdt_tick_t *tk, long period_ns);

/* Dorme até o próximo deadline e o avança um período. Quando o deadline já
 * passou, retorna na hora, conta um overrun e re-arma a partir do instante
 * atual, para que o pacer não gire recuperando um atraso irrecuperável. */
void fdt_tick_wait(fdt_tick_t *tk);

/* Frames que perderam o deadline desde fdt_tick_start. */
uint64_t fdt_tick_overruns(const fdt_tick_t *tk);

#endif /* FLEET_DT_TICK_H */
```

- [ ] **Step 4: Escrever `src/tick.c`**

```c
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/tick.h>
#include <errno.h>

#define NS_PER_S 1000000000L

static void advance(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= NS_PER_S) {
        ts->tv_nsec -= NS_PER_S;
        ts->tv_sec  += 1;
    }
}

static int passed(const struct timespec *deadline, const struct timespec *now)
{
    if (now->tv_sec != deadline->tv_sec) {
        return now->tv_sec > deadline->tv_sec;
    }
    return now->tv_nsec > deadline->tv_nsec;
}

void fdt_tick_start(fdt_tick_t *tk, long period_ns)
{
    if (tk == NULL) {
        return;
    }
    tk->period   = (period_ns > 0) ? period_ns : FDT_TICK_NS;
    tk->overruns = 0;
    clock_gettime(CLOCK_MONOTONIC, &tk->next);
    advance(&tk->next, tk->period);
}

void fdt_tick_wait(fdt_tick_t *tk)
{
    if (tk == NULL || tk->period <= 0) {
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (passed(&tk->next, &now)) {
        tk->overruns++;
        tk->next = now;
        advance(&tk->next, tk->period);
        return;
    }

    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tk->next, NULL)
           == EINTR) {
        /* Sinal interrompeu; o deadline não mudou, então retoma. */
    }
    advance(&tk->next, tk->period);
}

uint64_t fdt_tick_overruns(const fdt_tick_t *tk)
{
    return (tk == NULL) ? 0 : tk->overruns;
}
```

O teste usa `nanosleep`, então `tests/test_tick.c` precisa de `#define _POSIX_C_SOURCE 200809L` na primeira linha. Acrescente-o na Step 1 se o build reclamar.

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — imprime `measured period: 125.0xx ms over 8 frames`.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/tick.h src/tick.c tests/test_tick.c
git commit -m "feat: 125 ms frame pacer with absolute deadlines"
```

---

### Task 7: Predicado de viabilidade por vaso

Quita **C15** e a primeira metade de **C23**. A Armadilha 3 do spec mora aqui: isto mede o tempo de δ, **não** a latência de atuação.

**Files:**
- Create: `include/fleet_dt/feasibility.h`, `src/feasibility.c`
- Test: `tests/test_feasibility.c`

**Interfaces:**
- Consumes: `FDT_TICK_NS`.
- Produces: `fdt_feas_t`; `fdt_feas_init(fdt_feas_t*, long budget_ns) -> void`; `fdt_feas_begin(fdt_feas_t*) -> void`; `fdt_feas_end(fdt_feas_t*) -> long` (ns do último δ); `fdt_feas_ok(const fdt_feas_t*) -> int`; `fdt_feas_violations(const fdt_feas_t*) -> uint64_t`; `fdt_feas_worst_ns(const fdt_feas_t*) -> long`; `fdt_feas_frames(const fdt_feas_t*) -> uint64_t`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_feasibility.c`:

```c
#define _POSIX_C_SOURCE 200809L
#include <fleet_dt/feasibility.h>
#include <assert.h>
#include <stdio.h>
#include <time.h>

static void burn_ms(long ms)
{
    struct timespec ts = { .tv_sec = ms / 1000,
                           .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

int main(void)
{
    /* C15, §IV: "The DTI is feasible only if delta can be computed in less
     * than |t_k - t_{k-1}| for any arbitrary k; otherwise, it is merely a
     * time-bounded simulation model." Por vaso, por frame. */
    fdt_feas_t fs;
    fdt_feas_init(&fs, FDT_TICK_NS);
    assert(fdt_feas_ok(&fs) == 1);          /* zero frames é vacuamente viável */
    assert(fdt_feas_frames(&fs) == 0);

    fdt_feas_begin(&fs);
    burn_ms(5);
    long d = fdt_feas_end(&fs);
    assert(d > 0);
    assert(fdt_feas_frames(&fs) == 1);
    assert(fdt_feas_ok(&fs) == 1);
    assert(fdt_feas_violations(&fs) == 0);
    assert(fdt_feas_worst_ns(&fs) == d);

    /* "for any arbitrary k": um único estouro derruba a viabilidade, e ela
     * não volta. Não é média — é o pior caso. */
    fdt_feas_t tight;
    fdt_feas_init(&tight, 1000000L);        /* orçamento de 1 ms */
    fdt_feas_begin(&tight);
    burn_ms(10);
    fdt_feas_end(&tight);
    assert(fdt_feas_ok(&tight) == 0);
    assert(fdt_feas_violations(&tight) == 1);
    assert(fdt_feas_worst_ns(&tight) > 1000000L);

    fdt_feas_begin(&tight);
    fdt_feas_end(&tight);                   /* frame rápido não redime */
    assert(fdt_feas_ok(&tight) == 0);

    /* Orçamento inválido cai no deadline do paper. */
    fdt_feas_t dflt;
    fdt_feas_init(&dflt, 0);
    assert(dflt.budget_ns == FDT_TICK_NS);

    printf("worst delta: %ld ns, budget %ld ns\n",
           fdt_feas_worst_ns(&fs), FDT_TICK_NS);
    printf("test_feasibility: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fleet_dt/feasibility.h: No such file or directory`.

- [ ] **Step 3: Escrever o header**

`include/fleet_dt/feasibility.h`:

```c
#ifndef FLEET_DT_FEASIBILITY_H
#define FLEET_DT_FEASIBILITY_H

#include <fleet_dt/tick.h>
#include <stdint.h>

/* O predicado do §IV: "The DTI is feasible only if delta can be computed in
 * less than |t_k - t_{k-1}| for any arbitrary k; otherwise, it is merely a
 * time-bounded simulation model."
 *
 * Duas leituras que o paper mantém separadas e este tipo também: o tempo de
 * computar delta, medido aqui, e a latência de entrega da atuação, que o §V-A
 * observa ser alta mesmo com delta viável porque a atuação volta pela rede.
 * Esta medição não diz nada sobre a segunda.
 *
 * "for any arbitrary k" é pior caso, não média: uma única violação torna o
 * DTI inviável e nenhum frame rápido posterior o redime. */
typedef struct {
    long            budget_ns;   /* |t_k - t_{k-1}|, > 0        */
    struct timespec begin;       /* início do delta em curso    */
    long            last_ns;     /* duração do último delta     */
    long            worst_ns;    /* pior duração observada      */
    uint64_t        frames;      /* deltas medidos              */
    uint64_t        violations;  /* deltas que estouraram       */
} fdt_feas_t;

/* Arma o monitor. Orçamento menor ou igual a zero vira FDT_TICK_NS. */
void fdt_feas_init(fdt_feas_t *fs, long budget_ns);

/* Marca o início de um delta. */
void fdt_feas_begin(fdt_feas_t *fs);

/* Marca o fim e retorna a duração em nanossegundos, ou -1 se fs é NULL. */
long fdt_feas_end(fdt_feas_t *fs);

/* 1 quando nenhum delta medido estourou o orçamento. Vacuamente 1 antes do
 * primeiro frame. */
int fdt_feas_ok(const fdt_feas_t *fs);

/* Deltas que estouraram o orçamento. */
uint64_t fdt_feas_violations(const fdt_feas_t *fs);

/* Pior duração de delta observada, em nanossegundos. */
long fdt_feas_worst_ns(const fdt_feas_t *fs);

/* Deltas medidos. */
uint64_t fdt_feas_frames(const fdt_feas_t *fs);

#endif /* FLEET_DT_FEASIBILITY_H */
```

- [ ] **Step 4: Escrever `src/feasibility.c`**

```c
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/feasibility.h>

#define NS_PER_S 1000000000L

void fdt_feas_init(fdt_feas_t *fs, long budget_ns)
{
    if (fs == NULL) {
        return;
    }
    fs->budget_ns  = (budget_ns > 0) ? budget_ns : FDT_TICK_NS;
    fs->begin.tv_sec = 0;
    fs->begin.tv_nsec = 0;
    fs->last_ns    = 0;
    fs->worst_ns   = 0;
    fs->frames     = 0;
    fs->violations = 0;
}

void fdt_feas_begin(fdt_feas_t *fs)
{
    if (fs == NULL) {
        return;
    }
    clock_gettime(CLOCK_MONOTONIC, &fs->begin);
}

long fdt_feas_end(fdt_feas_t *fs)
{
    if (fs == NULL) {
        return -1;
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long ns = (long)(now.tv_sec - fs->begin.tv_sec) * NS_PER_S +
              (now.tv_nsec - fs->begin.tv_nsec);

    fs->last_ns = ns;
    fs->frames++;
    if (ns > fs->worst_ns) {
        fs->worst_ns = ns;
    }
    if (ns >= fs->budget_ns) {
        fs->violations++;
    }
    return ns;
}

int fdt_feas_ok(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : (fs->violations == 0);
}

uint64_t fdt_feas_violations(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->violations;
}

long fdt_feas_worst_ns(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->worst_ns;
}

uint64_t fdt_feas_frames(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->frames;
}
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — `test_feasibility: ok`.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/feasibility.h src/feasibility.c tests/test_feasibility.c
git commit -m "feat: per-vessel feasibility predicate over the delta compute time"
```

---

### Task 8: Store de `Bᵗ` e coordenador `S`

Quita **C18** — o cilindro e a caixa amarela da Fig. 4.

**Files:**
- Create: `include/fleet_dt/coordinator.h`, `src/coordinator.c`
- Test: `tests/test_coordinator.c`

**Interfaces:**
- Consumes: `fdt_state_t`, `fdt_goal_t`, `fdt_fleet_t`, `fdt_fleet_step`.
- Produces:
  - `fdt_store_t`; `fdt_store_init(fdt_store_t*, fdt_state_t *slots, size_t n) -> int`; `fdt_store_put(fdt_store_t*, size_t k, const fdt_state_t*) -> int`; `fdt_store_get(const fdt_store_t*, size_t k) -> const fdt_state_t*`; `fdt_store_size(const fdt_store_t*) -> size_t`.
  - `typedef void (*fdt_ctx_fn)(const fdt_store_t *bt, void *fleet_ctx, void *user)`
  - `typedef void (*fdt_plan_fn)(const fdt_store_t *bt, const void *fleet_ctx, fdt_goal_t *goals_out, size_t n, void *user)`
  - `fdt_coord_t`; `fdt_coord_init(fdt_coord_t*, fdt_fleet_t*, fdt_store_t*, fdt_ctx_fn, fdt_plan_fn, void *user) -> int`; `fdt_coord_step(fdt_coord_t*, const fdt_input_t *ins, size_t n, fdt_goal_t *goals_prev, fdt_goal_t *goals_now, fdt_state_t *b_out, fdt_actuation_t *a_out) -> int`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_coordinator.c`:

```c
#include <fleet_dt/coordinator.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct { float spread_deg; int ctx_calls; } coord_ctx_t;

static void delta_step(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                       const fdt_goal_t *g_prev, fdt_state_t *out,
                       void *ctx, void *fleet_ctx)
{
    (void)g_prev; (void)ctx; (void)fleet_ctx;
    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};
    out->yaw_deg += (in != NULL) ? in->wz_rps : 0.0f;
}

static void pi_hold(const fdt_state_t *b, const fdt_goal_t *g_now,
                    fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx; (void)fleet_ctx;
    out->cage_rad     = g_now->yaw_deg - b->yaw_deg;
    out->throttle_pct = 50.0f;
}

/* c^t, §IV: derivado dos estados que o coordenador recebe. Aqui, a dispersão
 * de guinada da frota, que é o análogo da distância entre vasos do paper. */
static void compute_ctx(const fdt_store_t *bt, void *fleet_ctx, void *user)
{
    (void)user;
    coord_ctx_t *c = (coord_ctx_t *)fleet_ctx;
    float lo = 1e30f, hi = -1e30f;
    for (size_t k = 0; k < fdt_store_size(bt); k++) {
        float y = fdt_store_get(bt, k)->yaw_deg;
        if (y < lo) lo = y;
        if (y > hi) hi = y;
    }
    c->spread_deg = hi - lo;
    c->ctx_calls++;
}

/* "in the same step in which it distributes g^t_i": o plano enxerga c^t que
 * acabou de ser calculado. */
static void plan_goals(const fdt_store_t *bt, const void *fleet_ctx,
                       fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)user;
    const coord_ctx_t *c = (const coord_ctx_t *)fleet_ctx;
    for (size_t k = 0; k < n; k++) {
        goals_out[k].yaw_deg = fdt_store_get(bt, k)->yaw_deg + c->spread_deg;
    }
}

int main(void)
{
    fdt_state_t slots[2];
    fdt_store_t bt;
    assert(fdt_store_init(&bt, slots, 2) == 0);
    assert(fdt_store_init(&bt, NULL, 2) == -1);
    assert(fdt_store_size(&bt) == 2);

    fdt_state_t s = { .yaw_deg = 7.0f };
    assert(fdt_store_put(&bt, 0, &s) == 0);
    assert(fdt_store_put(&bt, 2, &s) == -1);
    assert(fabsf(fdt_store_get(&bt, 0)->yaw_deg - 7.0f) < 1e-6f);
    assert(fdt_store_get(&bt, 2) == NULL);

    fdt_state_t st[2][4];
    fdt_twin_t twins[2];
    assert(fdt_twin_init(&twins[0], st[0], 4, delta_step, pi_hold, NULL) == 0);
    assert(fdt_twin_init(&twins[1], st[1], 4, delta_step, pi_hold, NULL) == 0);
    fdt_state_t z = {0};
    assert(fdt_twin_seed(&twins[0], &z) == 0);
    assert(fdt_twin_seed(&twins[1], &z) == 0);

    coord_ctx_t cc = {0};
    fdt_fleet_t f;
    assert(fdt_fleet_init(&f, twins, 2, &cc) == 0);

    fdt_coord_t co;
    assert(fdt_coord_init(&co, &f, &bt, compute_ctx, plan_goals, NULL) == 0);
    assert(fdt_coord_init(&co, NULL, &bt, compute_ctx, plan_goals, NULL) == -1);

    fdt_input_t ins[2]  = { { .wz_rps = 1.0f }, { .wz_rps = 3.0f } };
    fdt_goal_t  gp[2]   = {0};
    fdt_goal_t  gn[2]   = {0};
    fdt_state_t bs[2];
    fdt_actuation_t as[2];

    assert(fdt_coord_step(&co, ins, 1, gp, gn, bs, as) == 0);

    /* O store recebeu B^t dos dois vasos. */
    assert(fabsf(fdt_store_get(&bt, 0)->yaw_deg - 1.0f) < 1e-6f);
    assert(fabsf(fdt_store_get(&bt, 1)->yaw_deg - 3.0f) < 1e-6f);
    /* c^t foi calculado uma vez, a partir desses estados. */
    assert(cc.ctx_calls == 1);
    assert(fabsf(cc.spread_deg - 2.0f) < 1e-6f);
    /* g^t saiu no mesmo passo, já enxergando c^t. */
    assert(fabsf(gn[0].yaw_deg - 3.0f) < 1e-6f);
    assert(fabsf(gn[1].yaw_deg - 5.0f) < 1e-6f);

    /* O g^t deste frame vira o g^{t-1} do próximo. */
    assert(fdt_coord_step(&co, ins, 1, gp, gn, bs, as) == 0);
    assert(fabsf(gp[0].yaw_deg - 3.0f) < 1e-6f);

    printf("test_coordinator: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `make test`
Expected: FAIL — `fleet_dt/coordinator.h: No such file or directory`.

- [ ] **Step 3: Escrever o header**

`include/fleet_dt/coordinator.h`:

```c
#ifndef FLEET_DT_COORDINATOR_H
#define FLEET_DT_COORDINATOR_H

#include <fleet_dt/fleet.h>
#include <stddef.h>

/* O cilindro B^t da Figura 4: os estados recém-gerados são "sent to the MCS
 * and stored in a database". Um slot por vaso, sempre o mais recente, porque é
 * disso que o coordenador precisa para calcular c^t. Armazenamento do
 * chamador, como em toda esta biblioteca. */
typedef struct {
    fdt_state_t *slots;
    size_t       n;
} fdt_store_t;

int fdt_store_init(fdt_store_t *st, fdt_state_t *slots, size_t n);
int fdt_store_put(fdt_store_t *st, size_t k, const fdt_state_t *b);
const fdt_state_t *fdt_store_get(const fdt_store_t *st, size_t k);
size_t fdt_store_size(const fdt_store_t *st);

/* c^t, §IV: "The coordinator S computes c^t from the vessel states it
 * receives." A aplicação decide o que é c^t — o exemplo do paper é informação
 * contextual entre vasos, como distância. Escreve através de fleet_ctx. */
typedef void (*fdt_ctx_fn)(const fdt_store_t *bt, void *fleet_ctx, void *user);

/* g^t_i, §IV: "The coordinator (S) processes goals (g) received from the MCS
 * and subsequently distributes g^t_i per vessel DTI", e o faz "in the same
 * step in which it distributes", ou seja enxergando o c^t recém-calculado.
 * Escreve n objetivos em goals_out. */
typedef void (*fdt_plan_fn)(const fdt_store_t *bt, const void *fleet_ctx,
                            fdt_goal_t *goals_out, size_t n, void *user);

/* O S da Figura 4. */
typedef struct {
    fdt_fleet_t *fleet;
    fdt_store_t *bt;
    fdt_ctx_fn   ctx_fn;
    fdt_plan_fn  plan_fn;
    void        *user;
} fdt_coord_t;

/* 0 em sucesso, -1 com qualquer argumento obrigatório NULL, ou quando o store
 * e a frota discordam no número de vasos. */
int fdt_coord_init(fdt_coord_t *co, fdt_fleet_t *fleet, fdt_store_t *bt,
                   fdt_ctx_fn ctx_fn, fdt_plan_fn plan_fn, void *user);

/* Um frame coordenado, na ordem da Figura 4:
 *   1. a frota passa sob g^{t-1} e g^t (eqs. 5 e 6);
 *   2. os B^t resultantes entram no store;
 *   3. S calcula c^t a partir do store;
 *   4. S distribui o próximo g^t, já enxergando c^t.
 *
 * goals_now é entrada e saída: entra como o g^t deste frame e sai como o g^t
 * do próximo. goals_prev recebe o g^t deste frame, para que a chamada seguinte
 * o consuma como g^{t-1} sem que o chamador precise copiar nada.
 *
 * 0 em sucesso, -1 quando qualquer argumento é NULL ou o passo de frota falha.
 * Em -1 o store e os objetivos ficam intocados. */
int fdt_coord_step(fdt_coord_t *co, const fdt_input_t *ins, size_t n,
                   fdt_goal_t *goals_prev, fdt_goal_t *goals_now,
                   fdt_state_t *b_out, fdt_actuation_t *a_out);

#endif /* FLEET_DT_COORDINATOR_H */
```

- [ ] **Step 4: Escrever `src/coordinator.c`**

```c
#include <fleet_dt/coordinator.h>

int fdt_store_init(fdt_store_t *st, fdt_state_t *slots, size_t n)
{
    if (st == NULL || slots == NULL || n == 0) {
        return -1;
    }
    st->slots = slots;
    st->n     = n;
    for (size_t k = 0; k < n; k++) {
        st->slots[k] = (fdt_state_t){0};
    }
    return 0;
}

int fdt_store_put(fdt_store_t *st, size_t k, const fdt_state_t *b)
{
    if (st == NULL || st->slots == NULL || b == NULL || k >= st->n) {
        return -1;
    }
    st->slots[k] = *b;
    return 0;
}

const fdt_state_t *fdt_store_get(const fdt_store_t *st, size_t k)
{
    if (st == NULL || st->slots == NULL || k >= st->n) {
        return NULL;
    }
    return &st->slots[k];
}

size_t fdt_store_size(const fdt_store_t *st)
{
    return (st == NULL) ? 0 : st->n;
}

int fdt_coord_init(fdt_coord_t *co, fdt_fleet_t *fleet, fdt_store_t *bt,
                   fdt_ctx_fn ctx_fn, fdt_plan_fn plan_fn, void *user)
{
    if (co == NULL || fleet == NULL || bt == NULL ||
        ctx_fn == NULL || plan_fn == NULL) {
        return -1;
    }
    if (fdt_fleet_size(fleet) != fdt_store_size(bt)) {
        return -1;
    }
    co->fleet   = fleet;
    co->bt      = bt;
    co->ctx_fn  = ctx_fn;
    co->plan_fn = plan_fn;
    co->user    = user;
    return 0;
}

int fdt_coord_step(fdt_coord_t *co, const fdt_input_t *ins, size_t n,
                   fdt_goal_t *goals_prev, fdt_goal_t *goals_now,
                   fdt_state_t *b_out, fdt_actuation_t *a_out)
{
    if (co == NULL || co->fleet == NULL || co->bt == NULL) {
        return -1;
    }
    if (goals_prev == NULL || goals_now == NULL ||
        b_out == NULL || a_out == NULL) {
        return -1;
    }

    if (fdt_fleet_step(co->fleet, ins, goals_prev, goals_now,
                       n, b_out, a_out) != 0) {
        return -1;
    }

    size_t vessels = fdt_fleet_size(co->fleet);
    for (size_t k = 0; k < vessels; k++) {
        fdt_store_put(co->bt, k, &b_out[k]);
    }

    /* c^t a partir dos estados recebidos, depois g^t enxergando esse c^t: as
     * duas metades do mesmo passo, §IV. */
    co->ctx_fn(co->bt, co->fleet->ctx, co->user);

    /* O g^t que a frota acabou de consumir vira o g^{t-1} do próximo frame,
     * antes de goals_now ser sobrescrito com o objetivo novo. */
    for (size_t k = 0; k < vessels; k++) {
        goals_prev[k] = goals_now[k];
    }
    co->plan_fn(co->bt, co->fleet->ctx, goals_now, vessels, co->user);
    return 0;
}
```

- [ ] **Step 5: Rodar e ver passar**

Run: `make test`
Expected: PASS — `test_coordinator: ok`.

- [ ] **Step 6: Commit**

```bash
git add include/fleet_dt/coordinator.h src/coordinator.c tests/test_coordinator.c
git commit -m "feat: B^t store and the coordinator S of Figure 4"
```

---

### Task 9: Daemon de exemplo com janela profunda e viabilidade

Quita **C24** de forma visível, e fecha o Plano A com um binário que roda.

**Files:**
- Create: `examples/daemon.c`, `examples/README.md`
- Modify: `Makefile` (alvo `examples`), `.gitignore`

**Interfaces:**
- Consumes: tudo das Tasks 1–8.
- Produces: binário `examples/daemon`.

- [ ] **Step 1: Estender o Makefile**

Acrescentar ao `Makefile`, e incluir `examples` na linha `.PHONY`:

```make
examples: examples/daemon

examples/daemon: examples/daemon.c $(LIB)
	$(CC) $(CFLAGS) $< $(LIB) -lm -o $@
```

E na regra `clean`, acrescentar `examples/daemon` à lista de `rm -f`. Em
`.gitignore`, acrescentar `examples/daemon`.

- [ ] **Step 2: Escrever `examples/daemon.c`**

```c
/* daemon.c — uma frota de dois barcos passando a 125 ms sob um coordenador.
 *
 * Derivado de dt-daemon/daemon.c, de Anderson Domingues em
 * lsa-pucrs/boat-digital-twin, commit 90c5ab5. Repositório privado, então isto
 * é crédito, não um link que o leitor possa seguir.
 *
 * A dinâmica aqui é placeholder: a guinada é extrapolada da janela e a decisão
 * esterça proporcionalmente ao erro. O ponto do arquivo é a malha e a
 * amarração, não a física.
 *
 * A janela tem profundidade FDT_WINDOW e não 1, porque é isso que o §V-A
 * chama de operação proativa: "we added a range of states to delta^e, thereby
 * enabling proactive operation, as in model predictive control (MPC)". Com
 * n == 1 a promessa fica invisível na demonstração. */
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/coordinator.h>
#include <fleet_dt/feasibility.h>
#include <fleet_dt/tick.h>

#include <stdio.h>
#include <stdlib.h>

#define NBOATS     2
#define CAP        16
#define FRAMES     40
#define FDT_WINDOW 4
#define RAD2DEG    (180.0f / 3.14159265358979323846f)

static fdt_state_t storage[NBOATS][CAP];
static fdt_state_t bt_slots[NBOATS];
static fdt_twin_t  twins[NBOATS];
static fdt_feas_t  feas[NBOATS];

typedef struct { float spread_deg; float throttle_ceiling_pct; } fleet_ctx_t;

/* delta^e sobre os n mais recentes: extrapola a tendência da janela em vez de
 * só integrar o último frame. É a leitura preditiva que a janela habilita. */
static void demo_delta_e(const fdt_queue_t *q, size_t n, const fdt_input_t *in,
                         const fdt_goal_t *g_prev, fdt_state_t *out,
                         void *ctx, void *fleet_ctx)
{
    (void)g_prev; (void)ctx; (void)fleet_ctx;

    const fdt_state_t *newest = fdt_window_at(q, n, 0);
    const fdt_state_t *oldest = fdt_window_at(q, n, n - 1);
    *out = (newest != NULL) ? *newest : (fdt_state_t){0};

    float rate = (in != NULL) ? in->wz_rps : 0.0f;
    out->yaw_rate_rps = rate;

    /* rad/s num campo em graus: cruza a unidade, não soma cru. */
    float dt_s = (float)FDT_TICK_NS / 1e9f;
    out->yaw_deg += rate * dt_s * RAD2DEG;

    /* Tendência sobre a janela, o termo que n > 1 acrescenta. */
    if (newest != NULL && oldest != NULL && n > 1) {
        float trend = (newest->yaw_deg - oldest->yaw_deg) / (float)(n - 1);
        out->pitch_deg = trend;   /* exposto para o operador ver a derivada */
    }
}

static void demo_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                    fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;
    const fleet_ctx_t *fc = (const fleet_ctx_t *)fleet_ctx;
    out->cage_rad     = (g_now->yaw_deg - b->yaw_deg) * 0.01f;
    out->throttle_pct = (fc != NULL) ? fc->throttle_ceiling_pct : 100.0f;
}

static void demo_ctx(const fdt_store_t *bt, void *fleet_ctx, void *user)
{
    (void)user;
    fleet_ctx_t *fc = (fleet_ctx_t *)fleet_ctx;
    float lo = 1e30f, hi = -1e30f;
    for (size_t k = 0; k < fdt_store_size(bt); k++) {
        float y = fdt_store_get(bt, k)->yaw_deg;
        if (y < lo) lo = y;
        if (y > hi) hi = y;
    }
    fc->spread_deg = hi - lo;
    /* Uma política de manutenção de estação derivaria o teto da dispersão. */
    fc->throttle_ceiling_pct = (fc->spread_deg > 20.0f) ? 40.0f : 60.0f;
}

static void demo_plan(const fdt_store_t *bt, const void *fleet_ctx,
                      fdt_goal_t *goals_out, size_t n, void *user)
{
    (void)bt; (void)fleet_ctx; (void)user;
    const float targets[NBOATS] = { 90.0f, 45.0f };
    for (size_t k = 0; k < n; k++) {
        goals_out[k].yaw_deg = targets[k];
    }
}

int main(void)
{
    fdt_state_t b0 = {0};

    for (size_t k = 0; k < NBOATS; k++) {
        if (fdt_twin_init(&twins[k], storage[k], CAP,
                          demo_delta_e, demo_pi, NULL) != 0) {
            fprintf(stderr, "twin %zu failed to init\n", k);
            return EXIT_FAILURE;
        }
        /* B^1_i: a janela de profundidade FDT_WINDOW precisa de FDT_WINDOW
         * estados antes do primeiro passo, então semeia essa quantidade. */
        for (int s = 0; s < FDT_WINDOW; s++) {
            if (fdt_twin_seed(&twins[k], &b0) != 0) {
                fprintf(stderr, "twin %zu failed to seed\n", k);
                return EXIT_FAILURE;
            }
        }
        fdt_feas_init(&feas[k], FDT_TICK_NS);
    }

    fleet_ctx_t fc = { .spread_deg = 0.0f, .throttle_ceiling_pct = 60.0f };

    fdt_fleet_t fleet;
    fdt_store_t bt;
    fdt_coord_t coord;
    if (fdt_fleet_init(&fleet, twins, NBOATS, &fc) != 0 ||
        fdt_store_init(&bt, bt_slots, NBOATS) != 0 ||
        fdt_coord_init(&coord, &fleet, &bt, demo_ctx, demo_plan, NULL) != 0) {
        fprintf(stderr, "fleet wiring failed\n");
        return EXIT_FAILURE;
    }

    fdt_input_t ins[NBOATS] = {0};
    fdt_goal_t  gp[NBOATS]  = {0};
    fdt_goal_t  gn[NBOATS]  = {0};
    fdt_state_t bs[NBOATS];
    fdt_actuation_t as[NBOATS];

    ins[0].wz_rps = 0.20f;
    ins[1].wz_rps = 0.35f;

    fdt_tick_t tk;
    fdt_tick_start(&tk, FDT_TICK_NS);

    for (int frame = 0; frame < FRAMES; frame++) {
        fdt_feas_begin(&feas[0]);
        int rc = fdt_coord_step(&coord, ins, FDT_WINDOW, gp, gn, bs, as);
        fdt_feas_end(&feas[0]);
        if (rc != 0) {
            fprintf(stderr, "fleet step failed at frame %d\n", frame);
            return EXIT_FAILURE;
        }

        printf("k=%3d  boat0 yaw=%7.3f cage=%+6.3f  "
               "boat1 yaw=%7.3f cage=%+6.3f  spread=%6.3f\n",
               frame,
               (double)bs[0].yaw_deg, (double)as[0].cage_rad,
               (double)bs[1].yaw_deg, (double)as[1].cage_rad,
               (double)fc.spread_deg);

        fdt_tick_wait(&tk);
    }

    printf("frames=%d  overruns=%llu  feasible=%d  worst_delta=%ld ns  "
           "held0=%zu  queue_bytes0=%zu\n",
           FRAMES,
           (unsigned long long)fdt_tick_overruns(&tk),
           fdt_feas_ok(&feas[0]),
           fdt_feas_worst_ns(&feas[0]),
           fdt_twin_depth(&twins[0]),
           fdt_queue_bytes(CAP));

    return EXIT_SUCCESS;
}
```

- [ ] **Step 3: Escrever `examples/README.md`**

```markdown
# Exemplos

`daemon.c` passa uma frota de dois barcos a 125 ms por 40 frames, sob um
coordenador que calcula `cᵗ` e distribui `gᵗ`. A dinâmica é placeholder; o
arquivo existe para mostrar a malha e a amarração.

    make examples
    ./examples/daemon

Espere `overruns=0` e `feasible=1`. Overrun diferente de zero numa malha deste
tamanho aponta para a máquina, não para o código.

A janela tem profundidade 4, não 1. O §V-A justifica a janela dizendo que ela
habilita operação proativa, "as in model predictive control (MPC)"; com
profundidade 1 essa promessa não aparece na demonstração. É por isso que cada
gêmeo é semeado 4 vezes antes do primeiro passo: a eq. (3) exige que a fila
tenha `n` estados.

`queue_bytes0` é `48 * 16 = 768` bytes — o limite `48d` do §IV na capacidade
deste exemplo.

`worst_delta` é o pior tempo de computar δ, o predicado de viabilidade do §IV.
Não é a latência de atuação: o §V-A observa que a atuação chega atrasada
mesmo com δ viável, porque volta pela rede. São duas medições distintas, e esta
é a primeira.

Derivado de `dt-daemon/daemon.c`, de Anderson Domingues em
`lsa-pucrs/boat-digital-twin`. Repositório privado, então isto é crédito, não um
link que o leitor possa seguir.
```

- [ ] **Step 4: Compilar e rodar**

Run: `make examples && ./examples/daemon`
Expected: 40 linhas de frame e uma linha final com `overruns=0  feasible=1  worst_delta=<pequeno> ns  held0=16  queue_bytes0=768`. A execução leva 5 s, porque 40 frames a 125 ms.

- [ ] **Step 5: Commit**

```bash
git add examples/daemon.c examples/README.md Makefile .gitignore
git commit -m "feat: example daemon with a deep window and a feasibility monitor"
```

---

### Task 10: README e mapa paper↔código

**Files:**
- Create: `README.md`, `CHANGELOG.md`, `LICENSE`, `CONTRIBUTING.md`, `SECURITY.md`, `CODE_OF_CONDUCT.md`, `.github/CODEOWNERS`, `.github/dependabot.yml`
- Create: `docs/paper-to-code.md`

**Interfaces:**
- Consumes: os nomes públicos das Tasks 1–9.
- Produces: nada de código.

- [ ] **Step 1: Escrever `docs/paper-to-code.md`**

Uma linha por símbolo do §IV. Conteúdo exato:

```markdown
# Paper para código

Manuscrito rastreado: revisão `-10`. Modelo no **§IV**, equações **(1)–(6)**.
Se a numeração do manuscrito mudar, esta tabela e os comentários do código
mudam junto — foi exatamente esse descolamento que já aconteceu uma vez.

| Paper | Significado | Tipo ou função | Arquivo |
|---|---|---|---|
| `Iᵢᵗ` | entrada do ambiente, Tabela I | `fdt_input_t` | `include/fleet_dt/model.h` |
| `Bᵢᵗ` | estado do gêmeo, eq. (1) | `fdt_state_t` | `include/fleet_dt/model.h` |
| `Aᵢᵗ` | atuação, eq. (2) | `fdt_actuation_t` | `include/fleet_dt/model.h` |
| `gᵢᵗ` | objetivo de missão | `fdt_goal_t` | `include/fleet_dt/model.h` |
| `δ` | transição, eq. (2) | `fdt_twin_step` com `n == 1` | `src/transition.c` |
| `π` | decisão, eq. (2) | `fdt_pi_fn` | `include/fleet_dt/transition.h` |
| `δᵉ` | transição estendida, eq. (3) | `fdt_delta_e_fn` sobre os `n` mais recentes | `include/fleet_dt/transition.h` |
| `[Bᵗ⁻¹; Bᵗ⁻ⁿ]` | a janela do colchete | `fdt_window_at(q, n, k)`, `k == 0` é `Bᵗ⁻¹` | `src/transition.c` |
| `Bᵢ¹` | estado inicial conhecido, §IV | `fdt_twin_seed` | `src/transition.c` |
| fila de state frames | §IV; guarda `fdt_state_t` direto | `fdt_queue_t` | `include/fleet_dt/queue.h` |
| `48d` bytes | limite da fila na profundidade `d` | `fdt_queue_bytes` | `src/queue.c` |
| `Aᵢᵗ ⊆ Bᵢᵗ` | vaso não-autônomo, §IV | `fdt_twin_init_passive` | `src/transition.c` |
| `Fᵗ` | frota, eq. (4) | `fdt_fleet_t` | `include/fleet_dt/fleet.h` |
| `Δ` | transição de frota, eq. (5) | `fdt_fleet_step` | `src/fleet.c` |
| `Δᵉ` | transição estendida de frota, eq. (6) | `fdt_fleet_step` com `n > 1` | `src/fleet.c` |
| `cᵗ` | contexto de frota, eq. (4) | `fdt_fleet_t.ctx`, entregue a todo `δᵉ` e todo `π` | `include/fleet_dt/fleet.h` |
| `S` | coordenador, Fig. 4 | `fdt_coord_t` | `include/fleet_dt/coordinator.h` |
| banco de `Bᵗ` | cilindro da Fig. 4 | `fdt_store_t` | `include/fleet_dt/coordinator.h` |
| viabilidade | §IV, δ em menos de \|t_k − t_{k−1}\| | `fdt_feas_t` | `include/fleet_dt/feasibility.h` |
| `Δt` | período do frame, 125 ms | `FDT_TICK_NS` | `include/fleet_dt/tick.h` |

`δ` não tem função própria: `fdt_twin_step` roda `δᵉ` sobre a janela em toda
chamada, e a eq. (2) é o que essa chamada faz quando `n == 1`. Procurar por
`fdt_delta` neste repositório não encontra nada.

Não há tipo de frame. A eq. (1) declara doze variáveis e nenhum campo de tempo,
então uma entrada enfileirada **é** um estado, e é isso que faz
`fdt_queue_bytes(d)` valer exatamente `48d`. O pacer de 125 ms é o runtime que
dirige os frames; não faz parte do estado.

Divergências e ambiguidades registradas em
[`docs/spec/paper-claims.md`](spec/paper-claims.md), seções G e H.
```

- [ ] **Step 2: Escrever o `README.md`**

Seções, nesta ordem, e nada além: título e uma frase de escopo; a nota de que
o repo rastreia a revisão `-10` e que o modelo é o §IV; `## Build` com
`make lib`, `make test`, `make examples`; `## Exemplo` apontando para
`examples/README.md`; `## Paper para código` com um link para
`docs/paper-to-code.md` e nada duplicado dele; `## Cobertura das promessas`
com um link para `docs/spec/paper-claims.md` e a frase de que `status`
naquele arquivo é do repositório, não do paper; `## Origem` creditando
`dt-daemon/include/boat.h` de Anderson Domingues em
`lsa-pucrs/boat-digital-twin`, com a nota de que o repositório é privado.

- [ ] **Step 3: Escrever os arquivos de higiene**

`CHANGELOG.md` no formato Keep a Changelog 1.1.0 com uma seção
`## [Unreleased]` e um `### Added` listando: tipos do modelo da eq. (1) e
Tabela I; fila de state frames limitada em `48d`; δ, δᵉ e π sobre janela de
profundidade `n`; gêmeo não-autônomo; agregado de frota das eqs. (4)–(6);
pacer de 125 ms; predicado de viabilidade; store de `Bᵗ` e coordenador `S`;
daemon de exemplo.

`LICENSE`: MIT, detentor `Cássio Jones Dhein da Silva`, ano 2026.

`.github/CODEOWNERS`: `* @cassiojones`.

`.github/dependabot.yml`: ecossistema `github-actions`, diretório `/`,
intervalo `weekly`.

`CONTRIBUTING.md`: como rodar `make test`, a exigência de Conventional
Commits, e a regra de que toda mudança que toque um número publicado no paper
precisa citar seção e equação da revisão `-10`.

`SECURITY.md`: canal de reporte e a nota de que a biblioteca não faz I/O de
rede nem alocação dinâmica, então a superfície é a do chamador.

`CODE_OF_CONDUCT.md`: Contributor Covenant 2.1, contato
`cassiojonesdhein@gmail.com`.

- [ ] **Step 4: Verificar que a suíte inteira passa do zero**

Run: `make clean && make lib && make test && make examples && ./examples/daemon | tail -1`
Expected: todos os testes `ok` e a linha final do daemon com `overruns=0  feasible=1`.

- [ ] **Step 5: Commit**

```bash
git add README.md CHANGELOG.md LICENSE CONTRIBUTING.md SECURITY.md \
        CODE_OF_CONDUCT.md .github/CODEOWNERS .github/dependabot.yml \
        docs/paper-to-code.md
git commit -m "docs: paper-to-code map for revision -10 and repository hygiene"
```

---

## Self-Review

**Cobertura do spec, Plano A.** C6 Task 5 · C13 Task 6 · C14 Tasks 1 e 2 ·
C15 Task 7 · C16 Task 4 · C17 Task 5 · C18 Task 8 · C19 Task 1 · C24 Tasks 3
e 9. Ambiguidades 1, 3 e 4 do spec resolvidas nas Tasks 3, 1 e 3. Restante
dos claims fica nos Planos B e C.

**Consistência de tipos.** `fdt_twin_step` tem aridade de 7 argumentos
(sem `fleet_ctx`) e `fdt_twin_step_ctx` de 8; `fdt_fleet_step` usa a variante
`_ctx`. `fdt_window_at` conta do presente para trás e `fdt_queue_at` conta da
idade para a frente — ordens opostas de propósito, documentadas no header.
`fdt_goal_t` é alias de `fdt_state_t` em todas as tasks.

**Fronteiras.** Nada aqui toca rede, MQTT ou WeBots. `Iᵗ` chega pronto e `Aᵗ`
sai pronto; quem os move é o Plano B.
