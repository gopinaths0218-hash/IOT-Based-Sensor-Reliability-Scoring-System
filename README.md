/*
 * IoT-Based Sensor Reliability Scoring System
 * -------------------------------------------
 * Scores each sensor (0-100) using five weighted factors:
 *   1. Availability  - fraction of samples actually received
 *   2. Validity      - fraction of samples inside the physical range
 *   3. Stability     - low noise (std-dev) means higher score
 *   4. Consensus     - closeness to the median of peer sensors
 *   5. Battery       - remaining battery level
 *
 * Build: gcc sensor_reliability.c -o sensor_reliability -lm
 * Run  : ./sensor_reliability
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define NUM_SENSORS 5
#define NUM_SAMPLES 20
#define MISSING     -9999.0   /* marker for a lost packet */

/* Weights (sum = 1.0) */
#define W_AVAIL  0.25
#define W_VALID  0.20
#define W_STABLE 0.20
#define W_CONS   0.25
#define W_BATT   0.10

typedef struct {
    int    id;
    char   name[20];
    double min_valid, max_valid;       /* physical operating range   */
    double max_noise;                  /* std-dev considered "worst" */
    double battery;                    /* percent, 0-100             */
    double data[NUM_SAMPLES];          /* readings                   */
    /* computed factors */
    double availability, validity, stability, consensus, score;
} Sensor;

static double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int cmp_double(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static double median(double *arr, int n) {
    if (n == 0) return 0.0;
    qsort(arr, n, sizeof(double), cmp_double);
    return (n % 2) ? arr[n / 2] : (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
}

/* Mean of valid (received) samples */
static double sensor_mean(const Sensor *s, int *count) {
    double sum = 0; int c = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
        if (s->data[i] != MISSING) { sum += s->data[i]; c++; }
    if (count) *count = c;
    return c ? sum / c : 0.0;
}

static void compute_availability_validity(Sensor *s) {
    int received = 0, valid = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        if (s->data[i] == MISSING) continue;
        received++;
        if (s->data[i] >= s->min_valid && s->data[i] <= s->max_valid) valid++;
    }
    s->availability = (double)received / NUM_SAMPLES;
    s->validity     = received ? (double)valid / received : 0.0;
}

static void compute_stability(Sensor *s) {
    int n; double mean = sensor_mean(s, &n);
    if (n < 2) { s->stability = 0.0; return; }
    double var = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
        if (s->data[i] != MISSING) var += pow(s->data[i] - mean, 2);
    double sd = sqrt(var / (n - 1));
    s->stability = clamp(1.0 - sd / s->max_noise, 0.0, 1.0);
}

/* Compare each sensor's mean with the median of all sensor means */
static void compute_consensus(Sensor *sensors, int n) {
    double means[NUM_SENSORS];
    for (int i = 0; i < n; i++) means[i] = sensor_mean(&sensors[i], NULL);

    double tmp[NUM_SENSORS];
    memcpy(tmp, means, sizeof(means));
    double med = median(tmp, n);

    for (int i = 0; i < n; i++) {
        double range = sensors[i].max_valid - sensors[i].min_valid;
        double dev = fabs(means[i] - med) / (0.1 * range); /* 10% of range = worst */
        sensors[i].consensus = clamp(1.0 - dev, 0.0, 1.0);
    }
}

static void compute_score(Sensor *s) {
    s->score = 100.0 * (W_AVAIL  * s->availability +
                        W_VALID  * s->validity     +
                        W_STABLE * s->stability    +
                        W_CONS   * s->consensus    +
                        W_BATT   * (s->battery / 100.0));
}

static const char *grade(double score) {
    if (score >= 85) return "HIGH (Trusted)";
    if (score >= 65) return "MEDIUM (Monitor)";
    if (score >= 40) return "LOW (Recalibrate)";
    return "CRITICAL (Replace)";
}

/* Simulated sensor readings around a base temperature of 25 C */
static void load_demo_data(Sensor *s) {
    srand(42);
    const char *names[NUM_SENSORS] = {"TempNode-A", "TempNode-B", "TempNode-C",
                                      "TempNode-D", "TempNode-E"};
    double noise[NUM_SENSORS]   = {0.2, 0.3, 2.5, 0.4, 0.3};   /* C is noisy   */
    double offset[NUM_SENSORS]  = {0.0, 0.1, 0.0, 6.0, 0.0};   /* D has drifted*/
    double batt[NUM_SENSORS]    = {90, 75, 60, 55, 15};
    double lossp[NUM_SENSORS]   = {0.00, 0.05, 0.10, 0.05, 0.40}; /* E drops packets */

    for (int i = 0; i < NUM_SENSORS; i++) {
        s[i].id = i + 1;
        strcpy(s[i].name, names[i]);
        s[i].min_valid = -10.0; s[i].max_valid = 60.0;
        s[i].max_noise = 3.0;
        s[i].battery = batt[i];
        for (int j = 0; j < NUM_SAMPLES; j++) {
            double r = (double)rand() / RAND_MAX;
            if (r < lossp[i]) { s[i].data[j] = MISSING; continue; }
            double n = ((double)rand() / RAND_MAX - 0.5) * 2.0 * noise[i];
            s[i].data[j] = 25.0 + offset[i] + n;
        }
    }
    s[2].data[5] = 120.0;   /* out-of-range spike on sensor C */
}

int main(void) {
    Sensor sensors[NUM_SENSORS];
    memset(sensors, 0, sizeof(sensors));
    load_demo_data(sensors);

    for (int i = 0; i < NUM_SENSORS; i++) {
        compute_availability_validity(&sensors[i]);
        compute_stability(&sensors[i]);
    }
    compute_consensus(sensors, NUM_SENSORS);
    for (int i = 0; i < NUM_SENSORS; i++) compute_score(&sensors[i]);

    printf("\n=== IoT Sensor Reliability Report ===\n");
    printf("%-3s %-11s %6s %6s %6s %6s %5s | %6s  %s\n",
           "ID", "Sensor", "Avail", "Valid", "Stabl", "Cons", "Batt", "Score", "Status");
    printf("--------------------------------------------------------------------------\n");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Sensor *s = &sensors[i];
        printf("%-3d %-11s %5.0f%% %5.0f%% %5.0f%% %5.0f%% %4.0f%% | %6.1f  %s\n",
               s->id, s->name,
               s->availability * 100, s->validity * 100, s->stability * 100,
               s->consensus * 100, s->battery, s->score, grade(s->score));
    }

    /* Best / worst sensor */
    int best = 0, worst = 0;
    for (int i = 1; i < NUM_SENSORS; i++) {
        if (sensors[i].score > sensors[best].score)  best = i;
        if (sensors[i].score < sensors[worst].score) worst = i;
    }
    printf("\nMost reliable : %s (%.1f)\n", sensors[best].name,  sensors[best].score);
    printf("Least reliable: %s (%.1f)\n\n", sensors[worst].name, sensors[worst].score);
    return 0;
}

