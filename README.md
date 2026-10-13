# Adaptive Disk Request Scheduler

**Team CTRL Freaks · Kernel Ops hackathon · Problem statement OSH022**

A disk scheduler that watches the request queue and changes its strategy as the load changes. It stays as fast as the best classic algorithm and makes sure no request waits forever. The project also includes an interactive website that shows this happening on an animated hard disk.

![Welcome page](screenshots/1-welcome.png)

## Try it

Download **`index.html`** and open it in Chrome, Edge or Firefox. There is nothing to install, and it works without internet.

| Page | What you see |
|---|---|
| Watch the disks race | Two animated hard disks run the same requests side by side, for example FCFS against our Adaptive scheduler, with optional disk sounds. |
| Results | Head movement, average wait and worst wait for all 7 algorithms, plus a head-movement chart for each. |
| Starvation | One request that SSTF keeps ignoring, and how our scheduler rescues it. |
| Live challenges | One-click setups for the challenge scenarios (C1 to C5). |
| Step by step | Go one request at a time and see why the scheduler made each choice. You can add a request mid-run. |
| Event log | Every arrival, dispatch, completion and rescue, with a counting table. |
| How it works | The rules, settings and disk model in plain words. |

## The problem in three lines

1. A disk has one arm, and it can only be at one cylinder at a time.
2. The order in which requests are served decides how far the arm travels and how long each program waits.
3. Every classic rule has a weakness. FCFS travels too far. SSTF is fast but can starve far-away requests. SCAN and C-SCAN go all the way to the edge even when no request is there.

## How our scheduler decides

Before every request it serves, the scheduler looks at the queue and picks the first rule that applies:

| Order | Situation | What it does |
|---|---|---|
| 1 | A request has a deadline that is about to be missed | Serves the most urgent one (earliest deadline first), but only a few times in a row, so other requests are not starved. |
| 2 | A request has waited too long (more than max(80 ms, 4 × the usual response time)) | **Rescue sweep:** moves toward that request and serves everything on the way. The batch is frozen, so new arrivals cannot delay the rescue. |
| 3 | The queue is short (light load) | **SSTF:** serves the nearest request. |
| 4 | The queue is busy, and most requests are in one small area of the disk | **Cluster-SSTF:** stays inside the busy area. |
| 5 | The queue is busy and spread out | **LOOK:** sweeps back and forth like an elevator. |

A hysteresis band stops it flipping between rules on every request: it switches to "busy" when the smoothed queue length goes above 5, and back to "light" only when it drops below 3.

## Results

These numbers come from the C++ simulator: 5,000 cylinders, 2,000 requests per run, averaged over 20 seeds. Each cell is the **average response time in ms, so lower is better.**

| Workload | FCFS | SSTF | SCAN | C-SCAN | LOOK | C-LOOK | **Adaptive** |
|---|---:|---:|---:|---:|---:|---:|---:|
| Light uniform | 13.2 | 12.5 | 19.9 | 28.2 | 12.5 | 12.9 | **12.5** |
| Heavy uniform | 1,340.8 | 27.8 | 35.7 | 47.7 | 28.2 | 35.9 | **27.8** |
| Hot zone | 17.0 | 13.5 | 35.6 | 47.7 | 14.2 | 15.2 | **13.5** |
| Bursty | 207.3 | 115.2 | 116.6 | 127.9 | 115.4 | 117.6 | **115.4** |
| Starvation stress | 883.5 | 58.8 | 132.4 | 176.8 | 101.5 | 130.3 | **80.3** |
| Phase-changing | 208.9 | 42.0 | 51.7 | 62.3 | 42.4 | 45.1 | **41.9** |
| Sequential | 8.9 | 8.9 | 13.5 | 13.9 | 8.9 | 8.9 | **8.9** |
| Random | 28.9 | 17.4 | 26.2 | 36.0 | 17.7 | 20.1 | **17.4** |
| Mixed | 39.5 | 17.9 | 28.9 | 36.2 | 18.4 | 20.4 | **18.0** |

- **Fast:** the Adaptive average is within 1% of the best algorithm on 8 of the 9 workloads.
- **Fair:** under starvation stress the longest wait is **593 ms, compared with 1,279 ms for SSTF (54% lower)**. The average is 80.3 ms, which is 21% better than LOOK.
- **Honest trade-off:** in that same starvation test, Adaptive's average is 37% higher than SSTF's. That extra time is spent rescuing the requests SSTF would leave waiting.

**Textbook check.** Queue 98, 183, 37, 122, 14, 124, 65, 67, head at 53, 200 cylinders:

| | FCFS | SSTF | SCAN (down) | C-SCAN (up) | LOOK (down) | C-LOOK (up) |
|---|---:|---:|---:|---:|---:|---:|
| Head movement (cylinders) | 640 | 236 | 236 | 382 | 208 | 322 |

## Live challenges

| Challenge | What it tests | Result |
|---|---|---|
| C1 | The classic algorithms on the textbook queue | Matches the textbook values above. |
| C2 | One far request (cylinder 4900) while new requests keep arriving at the head | It waits 88.2 ms with the frozen rescue batch. Without freezing it waits 130.2 ms, the same as plain SSTF. |
| C3 | Queue size limit when too many requests arrive | Defer mode: all 64 served, 32 waited outside, longest time outside 167 ms. Reject mode: 32 served and 32 rejected. |
| C4 | New requests keep arriving along the rescue path | The rescue finishes in 2 dispatches with freezing, compared with 21 without. |
| C5 | Requests with deadlines | The urgent request goes first and misses by 1.7 ms, compared with 6.7 ms under SSTF. The other request meets its deadline at 23.4 ms. |
| C5b | A flood of deadline requests while one far request is starving | A limit of 3 deadline jumps in a row cuts the far request's wait from 256.2 ms to 96.6 ms. The trade-off: 59 of 61 deadlines are met instead of 61. |

## What is in this repo

| File / folder | What it is |
|---|---|
| `index.html` | The finished website in one file. Open this. |
| `proto/engine.js` | The scheduler: all 7 algorithms, the simulation loop, workloads, metrics and challenge features. **Start reading here.** |
| `proto/app.src.html` | The website interface: disk animation, charts, pages. |
| `proto/build.js` | Pastes `engine.js` into `app.src.html` and writes `index.html`. |
| `adaptive_disk_sched.cpp` | The C++17 simulator used for the benchmark numbers above. |
| `tests/verify.js` | Runs 600 random cases through both engines (JS and C++) and checks they give exactly the same results. |
| `tests/challenges.js` | Runs the challenge scenarios and prints the results. |
| `results/` | The CSV files the C++ simulator produced: results, mode use, aging sweep, timeline. |
| `docs/CTRLFreaks_OSH022_DD.pdf` | Design document. |
| `screenshots/` | Pictures of the website. |

## Run it yourself

You need [Node.js](https://nodejs.org) and a C++ compiler (g++).

```bash
# Rebuild the website after changing anything in proto/
node proto/build.js

# Build and run the C++ simulator
g++ -O2 -std=c++17 adaptive_disk_sched.cpp -o disksim
./disksim                  # full benchmark; writes results.csv, modes.csv, sweep.csv, timeline.csv
./disksim --queue "98,183,37,122,14,124,65,67" --head 53 --cyl 200 --dir down

# Checks
node tests/verify.js       # JS engine vs C++ engine (build disksim first)
node tests/challenges.js   # challenge scenarios
```

Expected output of `verify.js`: `cases 600, algorithm runs compared 4200, identical 4200, different 0`

## Assumptions and limits

- This is a simulation. No kernel code was changed, and no real disk was used.
- The disk model: seek time = 0.8 ms + 0.0028 ms per cylinder, plus 4.2 ms for rotation and transfer.
- The thresholds (queue length 3 to 5, 70% cluster, aging factor 4) were tuned on our own synthetic workloads, not on real disk traces.
- The model is a single hard disk. On SSDs, seek order matters much less.

## Team CTRL Freaks

- Durga Vaishnavi Ch (Team Leader)
- K Bhuvana Royal
- Asleshita Tuduku

## AI usage

As declared in our presentation, we used an AI assistant (Claude) for about 90% of the code and documents. We checked every result in this README by running the tests above.

## More screenshots

| Home | Disk race |
|---|---|
| ![Home](screenshots/2-home.png) | ![Simulation](screenshots/3-simulation.png) |
| **Results** | **Live challenges** |
| ![Compare](screenshots/4-compare.png) | ![Challenges](screenshots/5-challenges.png) |
