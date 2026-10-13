// OSH022 - Adaptive Disk Request Scheduler
// Event-driven simulator comparing FCFS, SSTF, SCAN, C-SCAN, LOOK, C-LOOK
// against an adaptive scheduler that switches policy from live queue signals.
//
// Build:  g++ -O2 -std=c++17 adaptive_disk_sched.cpp -o disksim
// Run:    ./disksim            (writes results.csv, modes.csv, sweep.csv, timeline.csv)

#include <bits/stdc++.h>
using namespace std;

struct Req {
    int id;
    double arr;   // arrival time (ms)
    int cyl;      // target cylinder
    double fin;   // completion time (ms)
};

struct Disk {
    int ncyl;        // number of cylinders
    double settle;   // fixed cost of any non-zero seek (ms)
    double perCyl;   // seek cost per cylinder travelled (ms)
    double rot;      // rotational latency + transfer per request (ms)
};

enum Algo { FCFS = 0, SSTF, SCAN, CSCAN, LOOK, CLOOK, ADAPT, NALGO };
static const char* ALGO_NAME[] = {"FCFS", "SSTF", "SCAN", "C-SCAN", "LOOK", "C-LOOK", "ADAPTIVE"};

struct Pick {
    int idx;      // index into the pending queue
    long travel;  // cylinders the head moves to reach it (incl. edge trips)
};

// ---------------------------------------------------------------- primitives
static int pickOldest(const vector<Req*>& p) {
    int b = 0;
    for (int i = 1; i < (int)p.size(); i++)
        if (p[i]->arr < p[b]->arr || (p[i]->arr == p[b]->arr && p[i]->id < p[b]->id)) b = i;
    return b;
}

static int pickNearest(const vector<Req*>& p, int head) {
    int b = 0;
    for (int i = 1; i < (int)p.size(); i++) {
        int di = abs(p[i]->cyl - head), db = abs(p[b]->cyl - head);
        if (di < db || (di == db && p[i]->arr < p[b]->arr)) b = i;
    }
    return b;
}

// nearest request at or beyond the head in direction dir (+1 up, -1 down); -1 if none
static int nearestInDir(const vector<Req*>& p, int head, int dir) {
    int b = -1;
    for (int i = 0; i < (int)p.size(); i++) {
        long off = (long)(p[i]->cyl - head) * dir;
        if (off < 0) continue;
        if (b < 0 || off < (long)(p[b]->cyl - head) * dir ||
            (off == (long)(p[b]->cyl - head) * dir && p[i]->arr < p[b]->arr)) b = i;
    }
    return b;
}

static int pickLowest(const vector<Req*>& p) {
    int b = 0;
    for (int i = 1; i < (int)p.size(); i++)
        if (p[i]->cyl < p[b]->cyl) b = i;
    return b;
}

static int pickHighest(const vector<Req*>& p) {
    int b = 0;
    for (int i = 1; i < (int)p.size(); i++)
        if (p[i]->cyl > p[b]->cyl) b = i;
    return b;
}

// ----------------------------------------------------------------- scheduler
static double AGE_OVERRIDE = -1;   // set only by the sensitivity sweep in main()

struct Scheduler {
    Algo algo;
    Disk d;
    int dir = +1;          // current direction (SCAN, LOOK, adaptive)
    int sweepDir = +1;     // fixed sweep direction (C-SCAN, C-LOOK)

    // ---- adaptive scheduler: tunables
    double alphaQ = 0.2;        // EWMA weight for queue length
    double alphaW = 0.1;        // EWMA weight for response time
    double qBusy = 4.0;         // smoothed queue length that separates light from busy
    double hyst = 0.25;         // hysteresis band around that boundary
    double zoneHalf = 0.05;     // cluster window = median +/- 5% of the disk
    double zoneFrac = 0.70;     // share of queue inside the window to call it clustered
    double ageFactor = 4.0;     // starvation limit = ageFactor x typical response time
    double ageFloor = 80.0;     // ...but never below this (ms)

    // ---- adaptive scheduler: state
    enum Mode { M_SSTF = 0, M_LOOK, M_CLUSTER, M_RESCUE, NMODE };
    int level = 0;              // 0 light, 1 busy
    double qe = 0, we = 0;      // smoothed queue length / response time
    int rescueId = -1;          // request currently being rescued from starvation
    int lastMode = -1;
    long modeUse[NMODE] = {0, 0, 0, 0};
    long rescues = 0, switches = 0;

    Scheduler(Algo a, Disk dk, int startDir = +1) : algo(a), d(dk), dir(startDir), sweepDir(startDir) {
        if (AGE_OVERRIDE > 0) ageFactor = AGE_OVERRIDE;
    }

    void onComplete(double resp) { we = (we == 0) ? resp : (1 - alphaW) * we + alphaW * resp; }

    Pick look(const vector<Req*>& p, int head) {
        int i = nearestInDir(p, head, dir);
        if (i < 0) { dir = -dir; i = nearestInDir(p, head, dir); }
        return {i, labs((long)p[i]->cyl - head)};
    }

    Pick scan(const vector<Req*>& p, int head) {
        int i = nearestInDir(p, head, dir);
        if (i >= 0) return {i, labs((long)p[i]->cyl - head)};
        int edge = dir > 0 ? d.ncyl - 1 : 0;       // run to the edge, then reverse
        dir = -dir;
        i = nearestInDir(p, head, dir);
        return {i, labs((long)edge - head) + labs((long)edge - p[i]->cyl)};
    }

    Pick clook(const vector<Req*>& p, int head) {
        int i = nearestInDir(p, head, sweepDir);
        if (i >= 0) return {i, labs((long)p[i]->cyl - head)};
        i = sweepDir > 0 ? pickLowest(p) : pickHighest(p);  // jump back to the far end of the queue
        return {i, labs((long)head - p[i]->cyl)};
    }

    Pick cscan(const vector<Req*>& p, int head) {
        int i = nearestInDir(p, head, sweepDir);
        if (i >= 0) return {i, labs((long)p[i]->cyl - head)};
        long top = d.ncyl - 1;                      // run to one end, return to the other, sweep again
        if (sweepDir > 0) { i = pickLowest(p); return {i, (top - head) + top + p[i]->cyl}; }
        i = pickHighest(p);
        return {i, (long)head + top + (top - p[i]->cyl)};
    }

    bool clustered(const vector<Req*>& p) const {
        if (p.size() < 4) return false;
        vector<int> c(p.size());
        for (size_t i = 0; i < p.size(); i++) c[i] = p[i]->cyl;
        nth_element(c.begin(), c.begin() + c.size() / 2, c.end());
        int med = c[c.size() / 2], win = (int)(zoneHalf * d.ncyl), in = 0;
        for (auto* r : p) if (abs(r->cyl - med) <= win) in++;
        return in >= zoneFrac * p.size();
    }

    Pick adaptive(const vector<Req*>& p, int head, double now) {
        // 1. MONITOR: smooth the queue length so one burst does not flip the policy
        qe = (1 - alphaQ) * qe + alphaQ * (double)p.size();

        // 2. CLASSIFY load, with hysteresis so the level does not thrash at a boundary
        if (level == 0 && qe > qBusy * (1 + hyst)) level = 1;
        else if (level == 1 && qe < qBusy * (1 - hyst)) level = 0;

        // 3. STARVATION GUARD: rescue any request that has waited far beyond normal
        int tgt = -1;
        if (rescueId >= 0)
            for (int i = 0; i < (int)p.size(); i++) if (p[i]->id == rescueId) tgt = i;
        if (tgt < 0) {
            rescueId = -1;
            int o = pickOldest(p);
            double limit = max(ageFloor, ageFactor * we);
            if (now - p[o]->arr > limit) { tgt = o; rescueId = p[o]->id; rescues++; }
        }

        int mode;
        Pick pk;
        if (tgt >= 0) {
            // sweep toward the starving request, serving everything on the way
            mode = M_RESCUE;
            if (p[tgt]->cyl != head) dir = p[tgt]->cyl > head ? +1 : -1;
            int i = nearestInDir(p, head, dir);
            pk = {i, labs((long)p[i]->cyl - head)};
        } else if (level == 0) {
            mode = M_SSTF;                           // short queue: minimise seek
            int i = pickNearest(p, head);
            pk = {i, labs((long)p[i]->cyl - head)};
        } else if (clustered(p)) {
            mode = M_CLUSTER;                        // hot zone: stay local, guard covers outliers
            int i = pickNearest(p, head);
            pk = {i, labs((long)p[i]->cyl - head)};
        } else {
            mode = M_LOOK;                           // busy and spread out: elevator sweep
            pk = look(p, head);
        }

        if (mode != M_LOOK && p[pk.idx]->cyl != head) dir = p[pk.idx]->cyl > head ? +1 : -1;
        if (p[pk.idx]->id == rescueId) rescueId = -1;
        if (lastMode >= 0 && mode != lastMode) switches++;
        lastMode = mode;
        modeUse[mode]++;
        return pk;
    }

    Pick pick(const vector<Req*>& p, int head, double now) {
        switch (algo) {
            case FCFS:  { int i = pickOldest(p);        return {i, labs((long)p[i]->cyl - head)}; }
            case SSTF:  { int i = pickNearest(p, head); return {i, labs((long)p[i]->cyl - head)}; }
            case SCAN:  return scan(p, head);
            case CSCAN: return cscan(p, head);
            case LOOK:  return look(p, head);
            case CLOOK: return clook(p, head);
            default:    return adaptive(p, head, now);
        }
    }
};

// ---------------------------------------------------------------- simulation
struct Stats {
    double totalMove = 0, avgSeek = 0, meanResp = 0, p95 = 0, maxResp = 0, sdResp = 0, thr = 0;
    double modeShare[Scheduler::NMODE] = {0, 0, 0, 0};
    double rescues = 0, switches = 0;
    double meanWait = 0, maxWait = 0, endTime = 0;
};

struct TimelinePoint { double t; int q; int mode; };

static Stats simulate(vector<Req> reqs, Algo algo, Disk d, int head, int startDir = +1,
                      vector<TimelinePoint>* tl = nullptr, vector<int>* order = nullptr,
                      vector<int>* idOrder = nullptr) {
    sort(reqs.begin(), reqs.end(), [](const Req& a, const Req& b) {
        return a.arr < b.arr || (a.arr == b.arr && a.id < b.id);
    });
    Scheduler s(algo, d, startDir);
    vector<Req*> pending;
    size_t next = 0;
    int n = (int)reqs.size(), done = 0;
    double now = 0, move = 0;
    vector<double> resp;
    resp.reserve(n);
    double waitSum = 0, waitMax = 0;

    while (done < n) {
        while (next < reqs.size() && reqs[next].arr <= now) pending.push_back(&reqs[next++]);
        if (pending.empty()) { now = reqs[next].arr; continue; }   // idle until next arrival

        Pick pk = s.pick(pending, head, now);
        Req* r = pending[pk.idx];
        if (tl) tl->push_back({now, (int)pending.size(), s.lastMode});
        if (order) order->push_back(r->cyl);
        if (idOrder) idOrder->push_back(r->id);
        waitSum += now - r->arr;
        waitMax = max(waitMax, now - r->arr);

        move += pk.travel;
        now += (pk.travel > 0 ? d.settle + d.perCyl * pk.travel : 0.0) + d.rot;
        r->fin = now;
        head = r->cyl;
        resp.push_back(r->fin - r->arr);
        s.onComplete(r->fin - r->arr);
        pending[pk.idx] = pending.back();
        pending.pop_back();
        done++;
    }

    Stats st;
    st.meanWait = n ? waitSum / n : 0;
    st.maxWait = waitMax;
    st.endTime = now;
    st.totalMove = move;
    st.avgSeek = move / n;
    double sum = 0, sq = 0;
    for (double x : resp) { sum += x; st.maxResp = max(st.maxResp, x); }
    st.meanResp = sum / n;
    for (double x : resp) sq += (x - st.meanResp) * (x - st.meanResp);
    st.sdResp = sqrt(sq / n);
    sort(resp.begin(), resp.end());
    st.p95 = resp[(size_t)(0.95 * (n - 1))];
    st.thr = n / (now - reqs.front().arr) * 1000.0;
    long tot = 0;
    for (int m = 0; m < Scheduler::NMODE; m++) tot += s.modeUse[m];
    if (tot) for (int m = 0; m < Scheduler::NMODE; m++) st.modeShare[m] = 100.0 * s.modeUse[m] / tot;
    st.rescues = s.rescues;
    st.switches = s.switches;
    return st;
}

// ----------------------------------------------------------------- workloads
using Rng = mt19937;

static void addPoisson(vector<Req>& v, int n, double ratePerSec, double& t, Rng& g,
                       function<int(Rng&)> cylOf) {
    exponential_distribution<double> gap(ratePerSec / 1000.0);
    for (int i = 0; i < n; i++) { t += gap(g); v.push_back({(int)v.size(), t, cylOf(g), -1}); }
}

static function<int(Rng&)> uniformCyl(int ncyl) {
    return [ncyl](Rng& g) { return uniform_int_distribution<int>(0, ncyl - 1)(g); };
}

// hotFrac of requests land in [lo, hi]; the rest are spread over the whole disk
static function<int(Rng&)> hotCyl(int ncyl, int lo, int hi, double hotFrac) {
    return [=](Rng& g) {
        if (uniform_real_distribution<double>(0, 1)(g) < hotFrac)
            return uniform_int_distribution<int>(lo, hi)(g);
        return uniform_int_distribution<int>(0, ncyl - 1)(g);
    };
}

// stream in [lo, hi] plus a few requests parked far away in [flo, fhi]
static function<int(Rng&)> farCyl(int lo, int hi, int flo, int fhi, double farFrac) {
    return [=](Rng& g) {
        if (uniform_real_distribution<double>(0, 1)(g) < farFrac)
            return uniform_int_distribution<int>(flo, fhi)(g);
        return uniform_int_distribution<int>(lo, hi)(g);
    };
}

// sequential runs of neighbouring cylinders (a file being read); seqFrac < 1 mixes in random requests
static function<int(Rng&)> seqCyl(int ncyl, double seqFrac) {
    int at = 0, left = 0, step = 1;
    return [=](Rng& g) mutable {
        auto U = [&](int lo, int hi) { return uniform_int_distribution<int>(lo, hi)(g); };
        if (seqFrac < 1.0 && uniform_real_distribution<double>(0, 1)(g) >= seqFrac) return U(0, ncyl - 1);
        if (left <= 0 || at > ncyl - 1) {
            at = U(0, ncyl - 1); left = U(6, 16);
            step = uniform_real_distribution<double>(0, 1)(g) < 0.8 ? 1 : 2;
        }
        int c = min(at, ncyl - 1);
        at += step; left--;
        return c;
    };
}

static void addBursts(vector<Req>& v, int n, int burst, double gapMs, double& t, Rng& g, int ncyl) {
    auto cyl = uniformCyl(ncyl);
    for (int i = 0; i < n;) {
        for (int k = 0; k < burst && i < n; k++, i++)
            v.push_back({(int)v.size(), t + uniform_real_distribution<double>(0, 5)(g), cyl(g), -1});
        t += gapMs;
    }
}

static const int NWORK = 9;
static const char* WORK_NAME[NWORK] = {"Light uniform", "Heavy uniform", "Hot zone",
                                       "Bursty", "Starvation stress", "Phase-changing",
                                       "Sequential", "Random", "Mixed"};

static vector<Req> makeWorkload(int w, int ncyl, unsigned seed) {
    Rng g(seed * 7919u + w * 104729u + 17u);
    vector<Req> v;
    double t = 0;
    switch (w) {
        case 0: addPoisson(v, 2000, 40, t, g, uniformCyl(ncyl)); break;
        case 1: addPoisson(v, 2000, 120, t, g, uniformCyl(ncyl)); break;
        case 2: addPoisson(v, 2000, 120, t, g, hotCyl(ncyl, 2000, 2400, 0.85)); break;
        case 3: addBursts(v, 2000, 40, 400, t, g, ncyl); break;
        case 4: addPoisson(v, 2000, 185, t, g, farCyl(1000, 1300, 4500, 4999, 0.05)); break;
        case 6: addPoisson(v, 2000, 100, t, g, seqCyl(ncyl, 1.0)); break;
        case 7: addPoisson(v, 2000, 80, t, g, uniformCyl(ncyl)); break;
        case 8: addPoisson(v, 2000, 100, t, g, seqCyl(ncyl, 0.5)); break;
        default:
            addPoisson(v, 500, 40, t, g, uniformCyl(ncyl));
            addPoisson(v, 500, 120, t, g, uniformCyl(ncyl));
            addPoisson(v, 500, 120, t, g, hotCyl(ncyl, 2000, 2400, 0.85));
            addBursts(v, 500, 40, 400, t, g, ncyl);
    }
    return v;
}

// ------------------------------------------------------------------ CLI modes
// --verify: read test cases from stdin, print each algorithm's service order (used to
// check this engine against the browser prototype's engine)
static int verifyMode() {
    char tag[16];
    int ncyl, head, dir, n;
    double settle, perCyl, rot;
    while (scanf("%15s %d %d %d %lf %lf %lf %d", tag, &ncyl, &head, &dir, &settle, &perCyl, &rot, &n) == 8) {
        vector<Req> v(n);
        for (int i = 0; i < n; i++) {
            if (scanf("%d %lf %d", &v[i].id, &v[i].arr, &v[i].cyl) != 3) return 1;
            v[i].fin = -1;
        }
        Disk d{ncyl, settle, perCyl, rot};
        printf("CASE\n");
        for (int a = 0; a < NALGO; a++) {
            vector<int> ids;
            Stats s = simulate(v, (Algo)a, d, head, dir, nullptr, nullptr, &ids);
            printf("%s %.0f %.17g", ALGO_NAME[a], s.totalMove, s.endTime);
            for (int id : ids) printf(" %d", id);
            printf("\n");
        }
    }
    return 0;
}

// --queue "98,183,37" --head 53 --cyl 200 [--dir down]: run one queue through every algorithm
static int queueMode(const vector<int>& q, int head, int ncyl, int dir) {
    Disk d{ncyl, 0.8, 14.0 / max(1, ncyl - 1), 4.2};
    vector<Req> v;
    for (size_t i = 0; i < q.size(); i++) v.push_back({(int)i, 0.0, q[i], -1});
    printf("%zu requests, head at %d, cylinders 0..%d, starting %s\n\n", q.size(), head, ncyl - 1,
           dir > 0 ? "up" : "down");
    printf("%-9s %14s %13s %13s   service order\n", "algorithm", "head movement", "avg wait ms", "max wait ms");
    const int W = 61;
    auto col = [&](int c) { return ncyl > 1 ? (int)lround((double)c * (W - 1) / (ncyl - 1)) : 0; };
    vector<vector<int>> orders(NALGO);
    for (int a = 0; a < NALGO; a++) {
        Stats s = simulate(v, (Algo)a, d, head, dir, nullptr, &orders[a]);
        printf("%-9s %14.0f %13.1f %13.1f  ", ALGO_NAME[a], s.totalMove, s.meanWait, s.maxWait);
        for (int c : orders[a]) printf(" %d", c);
        printf("\n");
    }
    for (int a = 0; a < NALGO; a++) {
        printf("\n%s head movement (one line per request served; edge trips not drawn)\n", ALGO_NAME[a]);
        printf("        0%*d\n", W - 1, ncyl - 1);
        int prev = head;
        printf("  %5d |%s*\n", head, string(col(head), ' ').c_str());
        for (int c : orders[a]) {
            int x0 = col(prev), x1 = col(c);
            string line(W, ' ');
            for (int x = min(x0, x1); x <= max(x0, x1); x++) line[x] = '-';
            line[x1] = '*';
            while (!line.empty() && line.back() == ' ') line.pop_back();
            printf("  %5d |%s\n", c, line.c_str());
            prev = c;
        }
    }
    return 0;
}

// ---------------------------------------------------------------------- main
int main(int argc, char** argv) {
    if (argc > 1 && string(argv[1]) == "--verify") return verifyMode();
    if (argc > 1 && string(argv[1]) == "--queue") {
        vector<int> q;
        int head = 53, ncyl = 200, dir = +1;
        string list = argc > 2 ? argv[2] : "";
        for (char& ch : list) if (ch == ',' || ch == ';') ch = ' ';
        stringstream ss(list);
        for (int x; ss >> x;) q.push_back(x);
        for (int i = 3; i + 1 < argc; i += 2) {
            string k = argv[i], val = argv[i + 1];
            if (k == "--head") head = stoi(val);
            else if (k == "--cyl") ncyl = stoi(val);
            else if (k == "--dir") dir = (val == "down" || val == "-1") ? -1 : +1;
        }
        if (q.empty()) { printf("Give at least one cylinder, e.g. --queue \"98,183,37\"\n"); return 1; }
        for (int c : q) if (c < 0 || c >= ncyl) { printf("Cylinder %d is outside 0..%d\n", c, ncyl - 1); return 1; }
        if (head < 0 || head >= ncyl) { printf("Head %d is outside 0..%d\n", head, ncyl - 1); return 1; }
        return queueMode(q, head, ncyl, dir);
    }

    // Sanity check against the standard textbook queue (200 cylinders, head at 53)
    {
        Disk td{200, 0, 1, 0};
        int q[] = {98, 183, 37, 122, 14, 124, 65, 67};
        vector<Req> v;
        for (int i = 0; i < 8; i++) v.push_back({i, 0.0, q[i], -1});
        printf("Textbook check (head=53, 0..199):\n");
        for (int a = 0; a < NALGO; a++) {
            int sd = (a == SCAN) ? -1 : +1;
            vector<int> order;
            Stats s = simulate(v, (Algo)a, td, 53, sd, nullptr, &order);
            printf("  %-8s total head movement = %4.0f   order:", ALGO_NAME[a], s.totalMove);
            for (int c : order) printf(" %d", c);
            printf("\n");
        }
    }

    Disk d{5000, 0.8, 0.0028, 4.2};
    const int SEEDS = 20, HEAD0 = 2500;

    FILE* f = fopen("results.csv", "w");
    FILE* fm = fopen("modes.csv", "w");
    fprintf(f, "workload,algorithm,avg_seek_cyl,mean_resp_ms,p95_resp_ms,max_resp_ms,sd_resp_ms,throughput_rps\n");
    fprintf(fm, "workload,sstf_pct,look_pct,cluster_pct,rescue_pct,rescues,switches\n");

    for (int w = 0; w < NWORK; w++) {
        printf("\n== %s ==\n", WORK_NAME[w]);
        printf("  %-9s %9s %10s %10s %10s %10s %8s\n", "algo", "avgSeek", "meanResp", "p95", "max", "stddev", "thr/s");
        for (int a = 0; a < NALGO; a++) {
            Stats acc;
            for (int sd = 0; sd < SEEDS; sd++) {
                Stats s = simulate(makeWorkload(w, d.ncyl, sd), (Algo)a, d, HEAD0);
                acc.avgSeek += s.avgSeek; acc.meanResp += s.meanResp; acc.p95 += s.p95;
                acc.maxResp += s.maxResp; acc.sdResp += s.sdResp; acc.thr += s.thr;
                acc.rescues += s.rescues; acc.switches += s.switches;
                for (int m = 0; m < Scheduler::NMODE; m++) acc.modeShare[m] += s.modeShare[m];
            }
            double k = 1.0 / SEEDS;
            printf("  %-9s %9.1f %10.1f %10.1f %10.1f %10.1f %8.1f\n", ALGO_NAME[a], acc.avgSeek * k,
                   acc.meanResp * k, acc.p95 * k, acc.maxResp * k, acc.sdResp * k, acc.thr * k);
            fprintf(f, "%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", WORK_NAME[w], ALGO_NAME[a], acc.avgSeek * k,
                    acc.meanResp * k, acc.p95 * k, acc.maxResp * k, acc.sdResp * k, acc.thr * k);
            if (a == ADAPT) {
                printf("  adaptive modes: SSTF %.1f%%  LOOK %.1f%%  CLUSTER %.1f%%  RESCUE %.1f%%"
                       "  | rescues %.1f  switches %.1f\n",
                       acc.modeShare[0] * k, acc.modeShare[1] * k, acc.modeShare[2] * k, acc.modeShare[3] * k,
                       acc.rescues * k, acc.switches * k);
                fprintf(fm, "%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", WORK_NAME[w], acc.modeShare[0] * k,
                        acc.modeShare[1] * k, acc.modeShare[2] * k, acc.modeShare[3] * k,
                        acc.rescues * k, acc.switches * k);
            }
        }
    }
    fclose(f);
    fclose(fm);

    // Sensitivity: how the starvation limit trades mean response against worst case
    {
        FILE* fs = fopen("sweep.csv", "w");
        fprintf(fs, "age_factor,mean_resp_ms,p95_resp_ms,max_resp_ms\n");
        printf("\n== Starvation-guard sweep (Starvation stress workload) ==\n");
        for (double af : {2.0, 3.0, 4.0, 6.0, 8.0}) {
            AGE_OVERRIDE = af;
            double m = 0, p = 0, x = 0;
            for (int sd = 0; sd < SEEDS; sd++) {
                Stats s = simulate(makeWorkload(4, d.ncyl, sd), ADAPT, d, HEAD0);
                m += s.meanResp; p += s.p95; x += s.maxResp;
            }
            printf("  ageFactor %.0f: mean %.1f  p95 %.1f  max %.1f\n", af, m / SEEDS, p / SEEDS, x / SEEDS);
            fprintf(fs, "%.0f,%.3f,%.3f,%.3f\n", af, m / SEEDS, p / SEEDS, x / SEEDS);
        }
        AGE_OVERRIDE = -1;
        fclose(fs);
    }

    // Mode timeline for one phase-changing run (for the mode-switching chart)
    vector<TimelinePoint> tl;
    simulate(makeWorkload(5, d.ncyl, 0), ADAPT, d, HEAD0, +1, &tl);
    FILE* ft = fopen("timeline.csv", "w");
    fprintf(ft, "time_ms,queue_len,mode\n");
    for (auto& p : tl) fprintf(ft, "%.1f,%d,%d\n", p.t, p.q, p.mode);
    fclose(ft);
    return 0;
}
