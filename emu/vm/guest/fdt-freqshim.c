/*
 * fdt-freqshim.so: oscillator frequency error under chronyd (paper R4, Section VI E1; design item I9).
 *
 * All VMs of the emulation host count time with the same host oscillator. To give a vessel VM the
 * oscillator of its own computer, the frequency error e(t) of fdt_clock_schedule.py (holds and ramps of
 * +-20 ppm over minutes) has to act on the guest clock below chronyd, as a drift that chronyd must
 * measure and correct, not as a setting of chronyd. chronyd sets the kernel frequency with absolute
 * values (adjtimex ADJ_FREQUENCY, and ADJ_TICK), so a frequency written by another process would last
 * only until the next update of chronyd. This library, preloaded into chronyd (systemd drop-in
 * chrony-freqshim.conf), adds e(t) to every frequency that chronyd writes and removes it from every
 * frequency that the kernel reports to chronyd; fdt-clockerr.py writes e(t) into the shared page and
 * re-applies chronyd's last frequency plus the new e(t) to the kernel whenever e(t) changes. The kernel
 * clock then runs at (chronyd's correction + e(t)), and chronyd sees only its own correction.
 *
 * Shared page (FDT_FREQ_SHM, default /run/fdt-clock/freq.shm, created by whichever process starts first;
 * the layout is mirrored by FREQ_SHM in fdt-clockerr.py):
 *   off  0 u32 magic 'FDTF'      off  4 u32 version 1
 *   off  8 i64 err               injected error, kernel scaled ppm (ppm * 65536), written by the injector
 *   off 16 i64 chrony_freq       last freq that chronyd requested (scaled ppm), written here
 *   off 24 i64 chrony_tick       last tick that chronyd requested (us)
 *   off 32 u64 gen               incremented here before and after every frequency write of chronyd
 *   off 40 i64 applied_err       the err of the last kernel write (here or by the injector)
 *   off 48 i32 chrony_valid      1 after the first frequency write of chronyd
 *   off 56 u64 writes            frequency writes of chronyd
 * The page is mapped in the constructor, before chronyd enables its seccomp filter and drops its
 * privileges; afterwards the library only reads and writes memory and calls adjtimex.
 *
 * Build: cc -O2 -Wall -shared -fPIC -o fdt-freqshim.so fdt-freqshim.c -ldl
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/timex.h>
#include <time.h>
#include <unistd.h>

#define FDT_MAGIC 0x46544446u /* "FDTF" */
#define MAX_FREQ (500L << 16) /* kernel limit: 500 ppm */

struct fdt_freq_shm {
	uint32_t magic, version;
	int64_t err, chrony_freq, chrony_tick;
	uint64_t gen;
	int64_t applied_err;
	int32_t chrony_valid, pad;
	uint64_t writes;
};

_Static_assert(sizeof(struct fdt_freq_shm) == 64, "shared page layout");

static struct fdt_freq_shm *g;
static int (*real_adjtimex)(struct timex *);
static int (*real_ntp_adjtime)(struct timex *);
static int (*real_clock_adjtime)(clockid_t, struct timex *);

__attribute__((constructor)) static void fdt_shim_init(void)
{
	real_adjtimex = (int (*)(struct timex *))dlsym(RTLD_NEXT, "adjtimex");
	real_ntp_adjtime = (int (*)(struct timex *))dlsym(RTLD_NEXT, "ntp_adjtime");
	real_clock_adjtime = (int (*)(clockid_t, struct timex *))dlsym(RTLD_NEXT, "clock_adjtime");
	const char *path = getenv("FDT_FREQ_SHM");
	if (!path || !*path)
		path = "/run/fdt-clock/freq.shm";
	if (!strcmp(path, "none"))
		return;
	char dir[256];
	strncpy(dir, path, sizeof dir - 1);
	dir[sizeof dir - 1] = 0;
	char *slash = strrchr(dir, '/');
	if (slash && slash != dir) {
		*slash = 0;
		mkdir(dir, 0755);
	}
	int fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
	if (fd < 0)
		return;
	struct stat st;
	if (fstat(fd, &st) == 0 && st.st_size < 4096 && ftruncate(fd, 4096) != 0) {
		close(fd);
		return;
	}
	void *p = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (p == MAP_FAILED)
		return;
	g = p;
	if (__atomic_load_n(&g->magic, __ATOMIC_ACQUIRE) != FDT_MAGIC) {
		g->version = 1;
		__atomic_store_n(&g->magic, FDT_MAGIC, __ATOMIC_RELEASE);
	}
}

/* the frequency part of one adjtimex call of chronyd, around the real call */
static int shim(struct timex *t, int (*call)(void *, struct timex *), void *ctx)
{
	if (!g)
		return call(ctx, t);
	int has_freq = (t->modes & ADJ_FREQUENCY) && t->modes != ADJ_OFFSET_SS_READ &&
		       t->modes != ADJ_OFFSET_SINGLESHOT;
	int64_t err = 0;
	if (has_freq) {
		long req = t->freq;
		err = __atomic_load_n(&g->err, __ATOMIC_ACQUIRE);
		__atomic_add_fetch(&g->gen, 1, __ATOMIC_ACQ_REL);
		__atomic_store_n(&g->chrony_freq, (int64_t)req, __ATOMIC_RELEASE);
		if (t->modes & ADJ_TICK)
			__atomic_store_n(&g->chrony_tick, (int64_t)t->tick, __ATOMIC_RELEASE);
		__atomic_store_n(&g->chrony_valid, 1, __ATOMIC_RELEASE);
		long f = req + (long)err;
		if (f > MAX_FREQ)
			f = MAX_FREQ;
		if (f < -MAX_FREQ)
			f = -MAX_FREQ;
		t->freq = f;
		int r = call(ctx, t);
		if (r != -1)
			__atomic_store_n(&g->applied_err, err, __ATOMIC_RELEASE);
		__atomic_add_fetch(&g->gen, 1, __ATOMIC_ACQ_REL);
		__atomic_add_fetch(&g->writes, 1, __ATOMIC_RELAXED);
		t->freq -= (long)err;
		return r;
	}
	int r = call(ctx, t);
	if (r != -1)
		t->freq -= (long)__atomic_load_n(&g->applied_err, __ATOMIC_ACQUIRE);
	return r;
}

static int call_adjtimex(void *ctx, struct timex *t)
{
	(void)ctx;
	return real_adjtimex ? real_adjtimex(t) : -1;
}

static int call_ntp_adjtime(void *ctx, struct timex *t)
{
	(void)ctx;
	return real_ntp_adjtime ? real_ntp_adjtime(t) : -1;
}

static int call_clock_adjtime(void *ctx, struct timex *t)
{
	clockid_t id = *(clockid_t *)ctx;
	return real_clock_adjtime ? real_clock_adjtime(id, t) : -1;
}

int adjtimex(struct timex *t)
{
	return shim(t, call_adjtimex, NULL);
}

int ntp_adjtime(struct timex *t)
{
	return shim(t, call_ntp_adjtime, NULL);
}

int clock_adjtime(clockid_t id, struct timex *t)
{
	if (id != CLOCK_REALTIME)
		return real_clock_adjtime ? real_clock_adjtime(id, t) : -1;
	return shim(t, call_clock_adjtime, &id);
}
