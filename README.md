# TrafficFlowSim

Simulatore microscopico di traffico veicolare in tempo reale con editor integrato di reti stradali 2D, per Windows.

Modella la dinamica longitudinale con **IDM** (Intelligent Driver Model) combinato a un processo stocastico di **Ornstein-Uhlenbeck** e ritardo percettivo (per l'innesco e la propagazione di *phantom traffic jam* / onde stop-and-go), cambio corsia **MOBIL**, incroci a precedenza/stop e semaforizzati a più fasi, generazione di traffico casuale (Poisson) e inserimento manuale di veicoli.

## Funzionalità

- **Motore fisico** a 100 Hz su thread dedicato, disaccoppiato dal rendering (snapshot atomico lock-free)
- **IDM + MOBIL** con rumore stocastico OU e ritardo percettivo configurabile (con backstop anti-collisione per la reazione istantanea)
- **Incroci**: regola di precedenza (priorità/destra), stop, oppure semaforo con generazione automatica delle fasi (colorazione greedy del grafo dei conflitti — tante fasi quante servono per garantire zero movimenti in conflitto nella stessa fase)
- **Editor di rete stradale** interattivo: nodi, strade rettilinee/curve (Bézier cubiche) multicorsia, generatori di traffico, inserimento manuale veicoli, selezione/ispezione, cancellazione
- **Persistenza JSON** selettiva: solo infrastruttura oppure stato completo (inclusi tutti i veicoli attivi)
- **Due scenari precaricati**: anello a senso unico per code fantasma, incrocio a 4 vie semaforizzato

## Requisiti

- Windows 10/11 x64
- Visual Studio 2026 (toolset MSVC `v145`)
- [vcpkg](https://github.com/microsoft/vcpkg) in modalità manifest (integrato in VS2026, non serve installarlo a parte)

## Build

Apri `TrafficFlowSim.slnx` in Visual Studio 2026 e compila (`Debug|x64` o `Release|x64`) — vcpkg risolve e compila automaticamente le dipendenze (SFML, nlohmann-json, spdlog) al primo build.

Da riga di comando:

```bash
msbuild TrafficFlowSim.slnx /p:Configuration=Release /p:Platform=x64
```

L'eseguibile e gli asset (font, scenari) vengono copiati in `build/bin/<Debug|Release>/`.

> **Nota su ImGui**: Dear ImGui e ImGui-SFML sono inclusi come sorgente in `external/imgui/` (versione 1.91.9), non tramite vcpkg — la combinazione vcpkg con ImGui 1.92+ ha un bug noto di corruzione dei glifi nell'aggiornamento incrementale della texture atlas.

## Esecuzione

Avvia `build/bin/Release/app.exe` (o `Debug/app.exe`). Al primo avvio viene caricato uno scenario di default con un breve rettilineo a 2 corsie per direzione.

### Controlli vista

- **Rotellina**: zoom centrato sul cursore
- **Trascina con tasto centrale/destro**: pan

### Pannelli

- **Simulation Control & Phantom Jam Lab**: play/pause/step, scala temporale, rumore OU, ritardo di reazione, frenata di disturbo, caricamento scenari precaricati, statistiche live
- **Editor Toolbar & Tool Settings**: selezione strumento editor (mostra solo i controlli pertinenti allo strumento attivo) — Select/Inspect, Add Node, Add Straight/Curved Road, Place Spawner, Spawn Single Vehicle, Delete Element
- **Inspector**: dettagli e modifica di nodo/strada/veicolo selezionato
- **Persistence & Telemetry**: salvataggio/caricamento JSON (InfrastructureOnly / FullState), grafico velocità media

## Struttura del progetto

```text
modules/
├── core/         Vec2, Bézier cubiche, generatore stocastico, logger
├── network/      Grafo stradale: nodi, strade, corsie, connettori, semafori
├── sim/          IDM, MOBIL, risoluzione incroci, mondo simulato, thread fisico
├── persistence/  Serializzazione JSON selettiva
└── ui/           Camera, rendering, editor interattivo, pannelli ImGui
app/              Entry point ed event loop
external/imgui/   Dear ImGui + ImGui-SFML vendorizzati (v1.91.9)
assets/           Font e scenari precaricati
```

Ogni modulo dipende solo da quelli sottostanti nella gerarchia (`core → network → sim → persistence → ui → app`).

## Limitazioni note

- Nessun docking dei pannelli ImGui (non disponibile nella versione vendorizzata) — i pannelli sono comunque liberamente trascinabili e ridimensionabili
- In presenza di più connettori in forte conflitto geometrico a distanza ravvicinata (bordi di campionamento del rilevamento conflitti) possono verificarsi rare sovrapposizioni residue tra veicoli
